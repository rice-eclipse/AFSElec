#include "EKF_test.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "AltitudeEKF.h"
#include "NavEKF.h"

namespace {

bool g_verbose = false;
#define VLOG(...) do { if (g_verbose) printf(__VA_ARGS__); } while (0)

struct Checker {
    int pass = 0, fail = 0;
    bool operator()(const char* name, bool ok) {
        if (!ok || g_verbose) printf("[EKF] %s: %s\n", ok ? "PASS" : "FAIL", name);
        ok ? pass++ : fail++;
        return ok;
    }
};

// Deterministic noise (same numbers every run, on every platform)
struct Rng {
    uint64_t s = 0x9E3779B97F4A7C15ull;
    double uniform() {   // (0, 1)
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return ((s >> 11) + 0.5) * (1.0 / 9007199254740992.0);
    }
    double gauss() {
        double u1 = uniform(), u2 = uniform();
        return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
    }
};

// ---------------------------------------------------------------------------
// Matrix checks
// ---------------------------------------------------------------------------
bool matrixTest() {
    using ekf::Mat;
    Mat<3, 3> A;
    const float v[3][3] = {{4, 1, 0.5f}, {1, 3, 0.2f}, {0.5f, 0.2f, 2}};
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) A(r, c) = v[r][c];
    Mat<3, 3> inv;
    if (!ekf::choleskyInverse(A, inv)) return false;
    Mat<3, 3> I = A * inv;
    float err = 0;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) err = fmaxf(err, fabsf(I(r, c) - (r == c ? 1.0f : 0.0f)));
    Mat<2, 2> notPd;
    notPd(0, 0) = 1; notPd(0, 1) = 2; notPd(1, 0) = 2; notPd(1, 1) = 1;   // eigenvalues 3, -1
    Mat<2, 2> dummy;
    return err < 1e-5f && !ekf::choleskyInverse(notPd, dummy);
}

// ---------------------------------------------------------------------------
// Simulated flight
// ---------------------------------------------------------------------------
constexpr double G = 9.80665;
constexpr double PAD_P = 1000.0;      // mbar
constexpr double PAD_T = 293.15;      // K
constexpr double LAUNCH_T = 5.0;      // s
constexpr double BURN_T = 3.0;        // s
constexpr double THRUST_ACC = 100.0;  // m/s^2 (~10 g)
constexpr double DRAG_K = 0.0005;     // 1/m, a_drag = -k v|v|
constexpr double DROGUE_K = G / (25.0 * 25.0);   // 25 m/s descent
constexpr double END_T = 45.0;
constexpr double DT = 1e-4;           // truth integration step

constexpr double ACC_BIAS = 0.3;      // m/s^2
constexpr double ACC_SCALE = 1.01;
constexpr double ACC_NOISE = 0.15;    // m/s^2 per sample
constexpr double ACC_PERIOD = 1.0 / 1600;
constexpr double BARO_NOISE = 0.08;   // mbar
constexpr double BARO_PERIOD = 1.0 / 200;
constexpr double OUTLIER_T = 20.0;    // one bad baro reading here

double isaPressure(double h) {
    return PAD_P * pow(1.0 - 0.0065 * h / PAD_T, 5.25588);
}

struct Truth {
    double h = 0, v = 0, a = 0;
    bool apogeeSeen = false;
    double apogeeT = 0, apogeeH = 0;
    bool drogue = false;
};

void stepTruth(Truth& s, double t) {
    double thrust = (t >= LAUNCH_T && t < LAUNCH_T + BURN_T) ? THRUST_ACC : 0.0;
    double k = s.drogue ? DROGUE_K : DRAG_K;
    double a = (t >= LAUNCH_T) ? thrust - G - k * s.v * fabs(s.v) : 0.0;
    if (t >= LAUNCH_T && s.h <= 0 && a < 0 && s.v <= 0) a = 0;   // on the ground
    s.a = a;
    s.v += a * DT;
    s.h += s.v * DT;
    if (t > LAUNCH_T + BURN_T && !s.apogeeSeen && s.v < 0) {
        s.apogeeSeen = true;
        s.apogeeT = t;
        s.apogeeH = s.h;
    }
    if (s.apogeeSeen && t > s.apogeeT + 1.0) s.drogue = true;
}

struct PendingBaro { uint64_t sampleUs, arriveUs; float mbar; bool used; };

// ===========================================================================
// 3D simulated flight for NavEKF
// ===========================================================================
// Truth attitude is a closed-form function of time (so the gyro truth is exact), in double.
struct DQ {
    double w = 1, x = 0, y = 0, z = 0;
    DQ operator*(const DQ& b) const {
        return {w * b.w - x * b.x - y * b.y - z * b.z, w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y - x * b.z + y * b.w + z * b.x, w * b.z + x * b.y - y * b.x + z * b.w};
    }
    DQ conj() const { return {w, -x, -y, -z}; }
    static DQ euler(double r, double p, double yw) {   // aerospace ZYX
        const double cr = cos(r / 2), sr = sin(r / 2), cp = cos(p / 2), sp = sin(p / 2);
        const double cy = cos(yw / 2), sy = sin(yw / 2);
        return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy};
    }
    void rotate(const double v[3], double out[3]) const {   // body -> world
        DQ p{0, v[0], v[1], v[2]};
        DQ r = (*this) * p * conj();
        out[0] = r.x; out[1] = r.y; out[2] = r.z;
    }
    void rotateInv(const double v[3], double out[3]) const { conj().rotate(v, out); }
};

constexpr double NAV_DT = 1.0 / 3200;        // truth step; IMU every 2nd step (1600 Hz)
constexpr double NAV_BEGIN_T = 0.5;          // begin() after averaging 0.5 s at rest
constexpr double NAV_STATIONARY_END = 4.0;   // stationary() updates until here
constexpr double D2R = 3.14159265358979323846 / 180.0;

// Attitude: on a rail at 85 deg elevation, azimuth 30 deg; after launch it rolls at 0.5 rev/s and
// pitches over slowly; around apogee it flips nose-down, then swings and spins under the drogue.
DQ navAttitude(double t) {
    double el, az = 30.0, roll = 20.0;
    if (t < LAUNCH_T) {
        el = 85.0;
    } else if (t < 22.0) {
        el = 85.0 - 1.5 * (t - LAUNCH_T);
        roll += 180.0 * (t - LAUNCH_T);
    } else {
        roll += 180.0 * (22.0 - LAUNCH_T) + 30.0 * (t - 22.0);
        if (t < 25.0) {
            el = 59.5 + (-70.0 - 59.5) * (t - 22.0) / 3.0;
        } else {
            el = -70.0 + 15.0 * sin(2 * 3.14159265358979 * 0.4 * (t - 25.0));
            az += 10.0 * (t - 25.0);
        }
    }
    return DQ::euler(roll * D2R, el * D2R, az * D2R);
}

// Body angular rate [rad/s] by central difference of the closed-form attitude
void navGyroTruth(double t, double w[3]) {
    const double h = 1e-5;
    DQ d = navAttitude(t - h).conj() * navAttitude(t + h);
    if (d.w < 0) { d.w = -d.w; d.x = -d.x; d.y = -d.y; d.z = -d.z; }
    const double k = 2.0 / (2.0 * h);   // small angle: rotation vector ~ 2 * vec(d)
    w[0] = d.x * k; w[1] = d.y * k; w[2] = d.z * k;
}

ekf::Quat toQuat(const DQ& q) { return ekf::Quat::of((float)q.w, (float)q.x, (float)q.y, (float)q.z); }

void navFlightTest(Checker& check) {
    VLOG("\n----- NavEKF (3D) simulated flight -----\n");
    Rng rng;
    static NavEKF nav;   // keep the ~2.5 kB filter off the stack
    NavEKF::Config cfg;
    cfg.padTemperatureK = (float)PAD_T;

    // Accel bias: 0.2 m/s^2 along the rocket axis (learned on the pad), and ~5 mg across it, which is
    // what's left after a 6-position offset calibration. A cross-axis bias can't be told apart from
    // tilt on a still pad, so it sets the tilt alignment accuracy (bias / g, here ~0.4 deg).
    const double ACC_B[3] = {0.20, -0.05, 0.04};       // m/s^2
    const double GYR_B[3] = {0.010, -0.008, 0.012};    // rad/s (~0.5 deg/s)
    const double ACC_N = 0.10, GYR_N = 0.003;          // per sample at 1600 Hz
    const double ACC_SC = 1.005;
    const double MAG_NED[3] = {0.20, -0.03, 0.45};     // Gauss, mid-latitude field
    const double MAG_N = 0.004;
    cfg.magDeclinationDeg = (float)(atan2(MAG_NED[1], MAG_NED[0]) / D2R);   // so yaw is relative to true north
    cfg.magNoise = 0.01f;   // simulated direction noise is ~0.008 (the 0.02 default allows for calibration error)

    double p[3] = {0, 0, 0}, v[3] = {0, 0, 0}, aNed[3] = {0, 0, 0};
    bool drogue = false, apogeeSeen = false;
    double apogeeT = 0, apogeeH = 0;
    bool started = false;
    double accSum[3] = {0, 0, 0}, magSum[3] = {0, 0, 0};
    int accN = 0, magN = 0;
    PendingBaro pending[8] = {};
    bool outlier = false;
    double nextBaro = BARO_PERIOD, nextMag = 0.01;

    double sqAtt = 0, maxAtt = 0, sqH = 0, maxH = 0, maxV = 0, maxHoriz = 0, maxHorizSigma = 0;
    double tiltErrPad = -1, headErrPad = -1, maxGyroBiasErr = -1;
    double estApogeeT = -1, estApogeeH = 0, maxRange = 0;
    uint32_t nErr = 0;

    const long steps = (long)(END_T / NAV_DT);
    for (long i = 1; i <= steps; i++) {
        const double t = i * NAV_DT;
        const uint64_t nowUs = (uint64_t)llround(t * 1e6);
        const DQ q = navAttitude(t);

        // ---- Truth translation: thrust along the rocket axis, drag, gravity ----
        if (t >= LAUNCH_T) {
            double axisB[3] = {1, 0, 0}, axisN[3];
            q.rotate(axisB, axisN);
            const double thrust = (t < LAUNCH_T + BURN_T) ? THRUST_ACC : 0.0;
            const double k = drogue ? DROGUE_K : DRAG_K;
            const double sp = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            for (int j = 0; j < 3; j++) aNed[j] = thrust * axisN[j] - k * sp * v[j];
            aNed[2] += G;
            for (int j = 0; j < 3; j++) {
                v[j] += aNed[j] * NAV_DT;
                p[j] += v[j] * NAV_DT;
            }
            if (t > LAUNCH_T + BURN_T && !apogeeSeen && v[2] > 0) {
                apogeeSeen = true;
                apogeeT = t;
                apogeeH = -p[2];
            }
            if (apogeeSeen && t > apogeeT + 1.0) drogue = true;
        }

        // ---- IMU at 1600 Hz ----
        if (i % 2 == 0) {
            double fN[3] = {aNed[0], aNed[1], aNed[2] - G}, fB[3], w[3];
            if (t < LAUNCH_T) { fN[0] = fN[1] = 0; fN[2] = -G; }
            q.rotateInv(fN, fB);
            navGyroTruth(t, w);
            ekf::Vec3 acc, gyr;
            for (int j = 0; j < 3; j++) {
                acc[j] = (float)(fB[j] * ACC_SC + ACC_B[j] + ACC_N * rng.gauss());
                gyr[j] = (float)(w[j] + GYR_B[j] + GYR_N * rng.gauss());
            }
            if (!started) {
                for (int j = 0; j < 3; j++) accSum[j] += acc[j];
                accN++;
            } else {
                nav.imu(nowUs, acc, gyr);
            }
        }

        // ---- Magnetometer at 100 Hz ----
        if (t >= nextMag) {
            nextMag += 0.01;
            double mB[3];
            q.rotateInv(MAG_NED, mB);
            ekf::Vec3 m;
            for (int j = 0; j < 3; j++) m[j] = (float)(mB[j] + MAG_N * rng.gauss());
            if (!started) {
                for (int j = 0; j < 3; j++) magSum[j] += m[j];
                magN++;
            } else {
                nav.magnetometer(nowUs, m);
            }
        }

        // ---- begin() after averaging at rest ----
        if (!started && t >= NAV_BEGIN_T) {
            ekf::Vec3 a = ekf::vec3((float)(accSum[0] / accN), (float)(accSum[1] / accN), (float)(accSum[2] / accN));
            ekf::Vec3 m = ekf::vec3((float)(magSum[0] / magN), (float)(magSum[1] / magN), (float)(magSum[2] / magN));
            started = nav.begin((float)PAD_P, a, m, nowUs, cfg);
            if (!started) break;
            continue;
        }
        if (!started) continue;

        // ---- Barometer at 200 Hz, delivered 0-3 ms late ----
        if (t >= nextBaro) {
            nextBaro += BARO_PERIOD;
            double pr = isaPressure(-p[2]) + BARO_NOISE * rng.gauss();
            if (!outlier && t >= OUTLIER_T) { pr += 30.0; outlier = true; }
            for (PendingBaro& b : pending)
                if (!b.used) { b = {nowUs, nowUs + (uint64_t)(rng.uniform() * 3000.0), (float)pr, true}; break; }
        }
        for (PendingBaro& b : pending)
            if (b.used && b.arriveUs <= nowUs) { nav.baroPressure(b.sampleUs, b.mbar); b.used = false; }

        // ---- Pad alignment at 100 Hz ----
        if (t < NAV_STATIONARY_END && i % 32 == 0) nav.stationary(nowUs);

        if (tiltErrPad < 0 && t >= LAUNCH_T - 0.01) {
            // World-frame attitude error: z component = heading error, x/y = tilt error
            const ekf::Vec3 e = (nav.quaternion() * toQuat(q).conj()).toRotationVector();
            tiltErrPad = sqrt((double)(e[0] * e[0] + e[1] * e[1])) / D2R;
            headErrPad = fabs((double)e[2]) / D2R;
            maxGyroBiasErr = 0;
            for (int j = 0; j < 3; j++) maxGyroBiasErr = fmax(maxGyroBiasErr, fabs(nav.gyroBias()[j] - GYR_B[j]));
            VLOG("  at launch: accel bias (%.3f %.3f %.3f), true (%.3f %.3f %.3f) + scale; tilt err N/E (%.2f %.2f) deg\n",
                 nav.accelBias()[0], nav.accelBias()[1], nav.accelBias()[2], ACC_B[0], ACC_B[1], ACC_B[2],
                 e[0] / D2R, e[1] / D2R);
        }

        // ---- Errors in flight, every 10 ms ----
        if (t >= LAUNCH_T && i % 32 == 0) {
            const double att = ekf::Quat::angleBetween(toQuat(q), nav.quaternion()) / D2R;
            const double eh = nav.altitude() + p[2];
            const double en = nav.north() - p[0], ee = nav.east() - p[1];
            double ev = 0;
            for (int j = 0; j < 3; j++) ev = fmax(ev, fabs(nav.velocityNED()[j] - v[j]));
            sqAtt += att * att; maxAtt = fmax(maxAtt, att);
            sqH += eh * eh; maxH = fmax(maxH, fabs(eh));
            maxV = fmax(maxV, ev);
            maxHoriz = fmax(maxHoriz, sqrt(en * en + ee * ee));
            maxHorizSigma = fmax(maxHorizSigma, sqrt(en * en + ee * ee) / fmax(1e-3, (double)nav.horizontalStd()));
            maxRange = fmax(maxRange, sqrt(p[0] * p[0] + p[1] * p[1]));
            nErr++;
        }
        if (estApogeeT < 0 && t > LAUNCH_T + BURN_T && nav.verticalSpeed() < 0) {
            estApogeeT = t;
            estApogeeH = nav.altitude();
        }
    }

    const ekf::AsyncEKF<NavEKF::NSTATES>& f = nav.filter();
    const double n = nErr ? nErr : 1;
    VLOG("  %lu predictions, %lu accepted, %lu gated, %lu stale, %lu numerical\n", (unsigned long)f.predictions(),
         (unsigned long)f.acceptedUpdates(), (unsigned long)f.gatedUpdates(), (unsigned long)nav.staleUpdates(),
         (unsigned long)f.numericalFailures());
    VLOG("  on the pad: tilt error %.3f deg, heading error %.2f deg, gyro bias error %.5f rad/s\n", tiltErrPad,
         headErrPad, maxGyroBiasErr);
    VLOG("  attitude error: RMS %.2f deg, max %.2f deg (1-sigma at end: tilt %.2f, heading %.2f)\n",
         sqrt(sqAtt / n), maxAtt, nav.tiltStd(), nav.headingStd());
    VLOG("  altitude error: RMS %.2f m, max %.2f m; max velocity error %.2f m/s\n", sqrt(sqH / n), maxH, maxV);
    VLOG("  horizontal position error max %.1f m (max range from pad %.0f m), at most %.1f x its own 1-sigma\n",
         maxHoriz, maxRange, maxHorizSigma);
    VLOG("  apogee: true %.0f m at %.2f s, detected %.0f m at %.2f s\n", apogeeH, apogeeT, estApogeeH, estApogeeT);
    VLOG("  mean NIS: baro %.2f (ideal ~1), mag %.2f (ideal ~3)\n", nav.meanBaroNis(), nav.meanMagNis());

    check("3D: filter initialized from gravity + magnetometer", started);
    check("3D: tilt aligned on the pad (< 0.5 deg)", tiltErrPad >= 0 && tiltErrPad < 0.5);
    check("3D: heading aligned on the pad (< 2 deg)", headErrPad >= 0 && headErrPad < 2.0);
    check("3D: gyro bias learned on the pad (< 0.002 rad/s)", maxGyroBiasErr >= 0 && maxGyroBiasErr < 0.002);
    check("3D: launch detected", nav.inFlight());
    check("3D: attitude RMS error < 2 deg through boost, spin, flip and swing", sqrt(sqAtt / n) < 2.0);
    check("3D: max attitude error < 5 deg", maxAtt < 5.0);
    check("3D: altitude RMS error < 3 m", sqrt(sqH / n) < 3.0);
    check("3D: max velocity error < 5 m/s (any axis)", maxV < 5.0);
    // Without GPS the horizontal position drifts: a tilt error e during the boost gives a velocity
    // error of ~e * a_boost * t_burn. The filter must know it (error within 3 sigma of its own estimate).
    check("3D: horizontal error within 3 sigma of the filter's own uncertainty (no GPS)", maxHorizSigma < 3.0);
    check("3D: apogee detected within 0.5 s", estApogeeT > 0 && fabs(estApogeeT - apogeeT) < 0.5);
    check("3D: apogee altitude within 5 m", fabs(estApogeeH - apogeeH) < 5.0);
    check("3D: baro spike rejected", f.gatedUpdates() >= 1);
    check("3D: no stale or numerical failures", nav.staleUpdates() == 0 && f.numericalFailures() == 0);
    check("3D: filter consistent (mean NIS: baro 0.3-3, mag 0.5-10)",
          nav.meanBaroNis() > 0.3f && nav.meanBaroNis() < 3.0f && nav.meanMagNis() > 0.5f && nav.meanMagNis() < 10.0f);
}

} // namespace

bool EKF_test(bool verbose) {
    g_verbose = verbose;
    Checker check;
    VLOG("\n===== EKF test =====\n");

    check("Cholesky inverse (A * A^-1 = I) and rejects non-positive-definite", matrixTest());

    Rng rng;
    Truth truth;
    AltitudeEKF ekf;
    AltitudeEKF::Config cfg;
    cfg.padTemperatureK = (float)PAD_T;
    cfg.baroNoiseMbar = 0.1f;
    ekf.begin((float)PAD_P, 0, cfg);

    PendingBaro pending[8] = {};
    double nextAcc = ACC_PERIOD, nextBaro = BARO_PERIOD;
    bool outlierInjected = false;

    double sqErrH = 0, maxErrH = 0, maxErrV = 0;
    uint32_t nErr = 0;
    double biasAtLaunch = NAN;
    double estApogeeT = -1, estApogeeH = 0;
    double errHAfterOutlier = 0;
    uint32_t accSamples = 0, baroSamples = 0;
    const double effBias = ACC_BIAS + (ACC_SCALE - 1.0) * G;   // what the filter can see on the pad

    const long steps = (long)(END_T / DT);
    for (long i = 1; i <= steps; i++) {
        const double t = i * DT;
        stepTruth(truth, t);
        const uint64_t nowUs = (uint64_t)llround(t * 1e6);

        // Barometer: sampled now, delivered 0-3 ms later
        if (t >= nextBaro) {
            nextBaro += BARO_PERIOD;
            double p = isaPressure(truth.h) + BARO_NOISE * rng.gauss();
            if (!outlierInjected && t >= OUTLIER_T) {
                p += 30.0;
                outlierInjected = true;
            }
            for (PendingBaro& b : pending) {
                if (!b.used) {
                    b = {nowUs, nowUs + (uint64_t)(rng.uniform() * 3000.0), (float)p, true};
                    break;
                }
            }
        }

        // Accelerometer: sampled and processed now (with +-20 us jitter on the schedule)
        if (t >= nextAcc) {
            nextAcc += ACC_PERIOD + 20e-6 * (rng.uniform() - 0.5);
            double f = (truth.a + G) * ACC_SCALE + ACC_BIAS + ACC_NOISE * rng.gauss();
            ekf.accel(nowUs, (float)f);
            accSamples++;
        }

        // Deliver baro readings that have "arrived" (their timestamp is in the past)
        for (PendingBaro& b : pending) {
            if (b.used && b.arriveUs <= nowUs) {
                ekf.baroPressure(b.sampleUs, b.mbar);
                b.used = false;
                baroSamples++;
            }
        }

        // On the pad: zero-velocity updates at 100 Hz until 1 s before launch
        if (t < LAUNCH_T - 1.0 && i % 100 == 0) ekf.zeroVelocity(nowUs);

        if (isnan(biasAtLaunch) && t >= LAUNCH_T - 0.01) biasAtLaunch = ekf.accelBias();

        // Error statistics in flight, every 10 ms
        if (t >= LAUNCH_T && i % 100 == 0) {
            double eh = ekf.altitude() - truth.h, ev = ekf.velocity() - truth.v;
            sqErrH += eh * eh;
            nErr++;
            maxErrH = fmax(maxErrH, fabs(eh));
            maxErrV = fmax(maxErrV, fabs(ev));
        }
        if (fabs(t - (OUTLIER_T + 0.5)) < DT / 2) errHAfterOutlier = fabs(ekf.altitude() - truth.h);

        // Apogee as the flight software would see it: velocity crosses zero after burnout
        if (estApogeeT < 0 && t > LAUNCH_T + BURN_T && ekf.velocity() < 0) {
            estApogeeT = t;
            estApogeeH = ekf.altitude();
        }
    }

    const ekf::AsyncEKF<3>& f = ekf.filter();
    const double rmsH = sqrt(sqErrH / (nErr ? nErr : 1));
    VLOG("  %lu accel + %lu baro samples, %lu predictions, %lu accepted, %lu gated, %lu stale, %lu numerical\n",
         (unsigned long)accSamples, (unsigned long)baroSamples, (unsigned long)f.predictions(),
         (unsigned long)f.acceptedUpdates(), (unsigned long)f.gatedUpdates(), (unsigned long)f.staleUpdates(),
         (unsigned long)f.numericalFailures());
    VLOG("  bias at launch %.3f m/s^2 (true effective %.3f)\n", biasAtLaunch, effBias);
    VLOG("  altitude error: RMS %.2f m, max %.2f m; max velocity error %.2f m/s\n", rmsH, maxErrH, maxErrV);
    VLOG("  apogee: true %.0f m at %.2f s, detected %.0f m at %.2f s\n", truth.apogeeH, truth.apogeeT, estApogeeH,
         estApogeeT);
    VLOG("  mean baro NIS %.2f (ideal ~1)\n", ekf.meanBaroNis());

    check("simulated flight reached apogee > 1000 m", truth.apogeeSeen && truth.apogeeH > 1000);
    check("accel bias learned on the pad (within 0.05 m/s^2)", fabs(biasAtLaunch - effBias) < 0.05);
    check("launch detected (flight noise level in use)", ekf.inFlight());
    check("altitude RMS error < 3 m over the flight", rmsH < 3.0);
    check("max altitude error < 10 m", maxErrH < 10.0);
    check("max velocity error < 5 m/s", maxErrV < 5.0);
    check("apogee detected within 0.5 s of the true apogee",
          estApogeeT > 0 && fabs(estApogeeT - truth.apogeeT) < 0.5);
    check("apogee altitude within 5 m", fabs(estApogeeH - truth.apogeeH) < 5.0);
    check("30 mbar baro spike rejected by the gate", f.gatedUpdates() >= 1 && errHAfterOutlier < 5.0);
    check("late baro readings (0-3 ms) fused, none stale", f.staleUpdates() == 0);
    check("filter consistent (mean baro NIS 0.3 - 3)", ekf.meanBaroNis() > 0.3f && ekf.meanBaroNis() < 3.0f);
    check("no numerical failures", f.numericalFailures() == 0);
    check("pressure/altitude model round-trips", fabsf(ekf.altitudeAt(ekf.pressureAt(1234.5f)) - 1234.5f) < 0.5f);

    navFlightTest(check);

    printf("[EKF] %s: %d passed, %d failed\n", check.fail ? "FAIL" : "PASS", check.pass, check.fail);
    return check.fail == 0;
}
