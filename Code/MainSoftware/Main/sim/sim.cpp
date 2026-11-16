/**
 * sim.cpp - Replays a flight's nav CSV through the flight software with different settings.
 *
 *   navsim <flight nav csv> [output csv]
 *
 * Only the sensor data is taken from the input:
 *   gyro           p_dps, q_dps, r_dps + bg_*_dps    (the log has the bias removed; it is added back)
 *   accelerometer  ax_mps2, ay_mps2, az_mps2 + ba_*  (already the KX134 where the board used it)
 *   barometer      pressure_mbar                     (fed whenever it changes)
 *   KX134 |accel|  high_g_accel_g                    (for the high-g switch)
 * Everything else (EKF, flight states, barometer lockout, pad hold, roll test, servo power, log
 * rate) is recomputed with the real firmware code (NavEKF, FlightStateMachine, RollTest) and the
 * settings in sim/config.h + sim/sim_settings.h. The output has the same columns as the board's log.
 *
 * The main loop below follows navTask() in loop.cpp step by step; keep them in sync.
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

#include "config.h"
#include "sim_settings.h"
#include "flight_state.h"
#include "roll_test.h"
#include "NavEKF.h"

using ekf::Vec3;

enum class NavPhase : uint8_t { Aligning = 1, Navigating = 2 };   // as in shared.h

// ============================================================================
// Input
// ============================================================================
struct InRow {
    uint64_t t = 0;
    bool imuValid = false;                 // false: no IMU sample in this row (all zeros) or missing values
    Vec3 accel = Vec3::zeros();            // raw accelerometer, m/s^2 (bias added back)
    Vec3 gyro = Vec3::zeros();             // raw gyro, rad/s (bias added back)
    float pressure = NAN;                  // mbar
    float highGMag = 0.0f;                 // KX134 |accel|, g (0 = high-g takeover was off on the board)
    float q[4] = {NAN, NAN, NAN, NAN};     // logged attitude (only for SIM_MAG_FROM_LOGGED_ATTITUDE)
    float mach = NAN, speed = NAN, alt = NAN;   // only to recover the pad temperature
    int state = -1, rollTest = -1;         // the board's own results, for comparison
};

static std::vector<std::string> splitCsv(const std::string& line)
{
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        size_t c = line.find(',', start);
        std::string f = line.substr(start, c == std::string::npos ? std::string::npos : c - start);
        while (!f.empty() && (f.back() == '\r' || f.back() == '\n' || f.back() == ' ')) f.pop_back();
        while (!f.empty() && f.front() == ' ') f.erase(f.begin());
        out.push_back(f);
        if (c == std::string::npos) break;
        start = c + 1;
    }
    return out;
}

static float field(const std::vector<std::string>& f, int idx)
{
    if (idx < 0 || idx >= (int)f.size() || f[idx].empty()) return NAN;   // the board writes NaN as empty
    char* end;
    const double v = strtod(f[idx].c_str(), &end);
    return end == f[idx].c_str() ? NAN : (float)v;
}

static bool readFlight(const char* path, std::vector<InRow>& rows)
{
    FILE* fp = fopen(path, "rb");
    if (!fp) { fprintf(stderr, "Can't open %s\n", path); return false; }

    std::string line;
    std::vector<std::string> header;
    int col[64];
    const char* names[] = {"t_us", "p_dps", "q_dps", "r_dps", "ax_mps2", "ay_mps2", "az_mps2",
                           "bg_x_dps", "bg_y_dps", "bg_z_dps", "ba_x_mps2", "ba_y_mps2", "ba_z_mps2",
                           "pressure_mbar", "high_g_accel_g", "qw", "qx", "qy", "qz",
                           "mach", "speed_mps", "alt_m", "flight_state", "roll_test"};
    enum { T, P, Q, R, AX, AY, AZ, BGX, BGY, BGZ, BAX, BAY, BAZ, PRES, HG, QW, QX, QY, QZ, MACH, SPD, ALT,
           STATE, ROLL, NCOL };
    const bool required[NCOL] = {true, true, true, true, true, true, true, false, false, false, false, false,
                                 false, true};

    char buf[4096];
    bool haveHeader = false;
    size_t lineNo = 0, bad = 0;
    while (fgets(buf, sizeof buf, fp)) {
        lineNo++;
        line = buf;
        if (line.find_first_not_of(" \r\n") == std::string::npos) continue;
        const std::vector<std::string> f = splitCsv(line);
        if (!haveHeader) {
            header = f;
            for (int c = 0; c < NCOL; c++) {
                col[c] = -1;
                for (size_t h = 0; h < header.size(); h++)
                    if (header[h] == names[c]) col[c] = (int)h;
                if (col[c] < 0 && required[c]) {
                    fprintf(stderr, "%s: no \"%s\" column. Is this a nav_*.csv from the board?\n", path, names[c]);
                    fclose(fp);
                    return false;
                }
            }
            haveHeader = true;
            continue;
        }
        if (col[T] >= (int)f.size() || f[col[T]].empty()) { bad++; continue; }   // e.g. a row cut off by a power loss

        InRow r;
        r.t = strtoull(f[col[T]].c_str(), nullptr, 10);
        const float rate[3] = {field(f, col[P]), field(f, col[Q]), field(f, col[R])};
        const float acc[3] = {field(f, col[AX]), field(f, col[AY]), field(f, col[AZ])};
        const float bg[3] = {field(f, col[BGX]), field(f, col[BGY]), field(f, col[BGZ])};
        const float ba[3] = {field(f, col[BAX]), field(f, col[BAY]), field(f, col[BAZ])};
        bool finite = true, allZero = true;
        for (int i = 0; i < 3; i++) {
            finite &= std::isfinite(rate[i]) && std::isfinite(acc[i]);
            allZero &= rate[i] == 0.0f && acc[i] == 0.0f;
            r.gyro[i] = (rate[i] + (std::isfinite(bg[i]) ? bg[i] : 0.0f)) * NavEKF::DEG2RAD;
            r.accel[i] = acc[i] + (std::isfinite(ba[i]) ? ba[i] : 0.0f);
        }
        r.imuValid = finite && !allZero;   // all zero = no IMU sample since the previous row
        r.pressure = field(f, col[PRES]);
        const float hg = field(f, col[HG]);
        r.highGMag = std::isfinite(hg) ? hg : 0.0f;
        r.q[0] = field(f, col[QW]); r.q[1] = field(f, col[QX]); r.q[2] = field(f, col[QY]); r.q[3] = field(f, col[QZ]);
        r.mach = field(f, col[MACH]); r.speed = field(f, col[SPD]); r.alt = field(f, col[ALT]);
        const float st = field(f, col[STATE]), rt = field(f, col[ROLL]);
        r.state = std::isfinite(st) ? (int)st : -1;
        r.rollTest = std::isfinite(rt) ? (int)rt : -1;

        if (!rows.empty() && r.t <= rows.back().t) { bad++; continue; }   // time must go forward
        rows.push_back(r);
    }
    fclose(fp);
    if (bad) printf("Skipped %zu unusable rows (incomplete or out of order)\n", bad);
    return haveHeader;
}

/** Pad temperature the board used, from its own mach and speed columns (Mach = speed / sqrt(1.4 R T)). */
static bool padTempFromLog(const std::vector<InRow>& rows, float& padTempK)
{
    std::vector<float> est;
    for (const InRow& r : rows) {
        // Fast rows only: mach is logged with 3 decimals, so slow rows give a poor estimate
        if (!(r.speed > 40.0f && r.mach > 0.1f && std::isfinite(r.alt))) continue;
        const float tK = (r.speed / r.mach) * (r.speed / r.mach) / (1.4f * 287.05f);
        if (tK <= 216.7f) continue;   // clamped at the tropopause: the pad value can't be recovered
        est.push_back(tK + 0.0065f * (r.alt > 0.0f ? r.alt : 0.0f));
    }
    if (est.empty()) return false;
    std::nth_element(est.begin(), est.begin() + est.size() / 2, est.end());
    padTempK = est[est.size() / 2];
    return true;
}

/** Synthetic magnetometer reading from a logged attitude (body -> NED quaternion). */
static bool magFromAttitude(const float q[4], Vec3& m)
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    if (!(std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) return false;
    const float R[3][3] = {
        {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
        {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
        {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)},
    };
    const float inc = SIM_MAG_INCLINATION_DEG * NavEKF::DEG2RAD;
    const float ned[3] = {cosf(inc), 0.0f, sinf(inc)};
    for (int i = 0; i < 3; i++) m[i] = R[0][i] * ned[0] + R[1][i] * ned[1] + R[2][i] * ned[2];   // R^T * ned
    return true;
}

// ============================================================================
// Servos (servo.cpp without the hardware)
// ============================================================================
static uint32_t s_pulseUs[SERVO_COUNT];

static void servoWriteUs(int i, uint32_t us)
{
    if (us < servoMinUs(i)) us = servoMinUs(i);
    if (us > servoMaxUs(i)) us = servoMaxUs(i);
    s_pulseUs[i] = us;
}
static void servoWriteDeg(int i, float deg)
{
    const float us = (float)SERVO_CENTER_US[i] + deg * SERVO_US_PER_DEG;
    servoWriteUs(i, us < 0.0f ? 0u : (uint32_t)lroundf(us));
}
static void servoCenterAll() { for (int i = 0; i < SERVO_COUNT; i++) servoWriteUs(i, SERVO_CENTER_US[i]); }
static void servoReleaseAll() { for (int i = 0; i < SERVO_COUNT; i++) s_pulseUs[i] = 0; }

// ============================================================================
// Output (same columns and number formats as writeNavHeader() / writeNavRow() in loop.cpp)
// ============================================================================
static FILE* out;
static bool firstField = true;

static void sep() { if (!firstField) fputc(',', out); firstField = false; }
static void putF(float v, int dec) { sep(); if (!std::isnan(v)) fprintf(out, "%.*f", dec, (double)v); }
static void putU(unsigned long long v) { sep(); fprintf(out, "%llu", v); }
static void endRow() { fputc('\n', out); firstField = true; }

static void writeHeader()
{
    fputs("t_us,seq,phase,in_flight,flight_state,mach,baro_locked,high_g,high_g_accel_g,"
          "p_dps,q_dps,r_dps,roll_rate_dps,ax_mps2,ay_mps2,az_mps2,"
          "speed_mps,pressure_mbar,qbar_pa,tab_deg,tab_us,roll_test,doublet,"
          "qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,tilt_deg,tilt_az_deg,"
          "north_m,east_m,alt_m,vn_mps,ve_mps,vd_mps,baro_alt_m,"
          "tilt_std_deg,heading_std_deg,alt_std_m,horiz_std_m,"
          "ba_x_mps2,ba_y_mps2,ba_z_mps2,bg_x_dps,bg_y_dps,bg_z_dps,"
          "ekf_load_pct,ekf_max_us,imu_n,baro_n,mag_n,dropped\n", out);
}

// ============================================================================
// Simulated nav task (navTask() in loop.cpp)
// ============================================================================
static NavEKF nav;
static FlightStateMachine flight;
static RollTest rollTest;
static float navLastPressure = NAN, navPadTempK = 288.15f, navMach = 0.0f;
static bool navBaroLocked = false, navImuNew = false;
static Vec3 navAccel = Vec3::zeros(), navGyro = Vec3::zeros();
static bool navHighGOk = false, navUsingHighG = false;
static float navHighGMag = 0.0f;
static Vec3 navRateSum = Vec3::zeros(), navAccSum = Vec3::zeros();
static float navRollRateSum = 0.0f;
static uint32_t navAvgN = 0;
struct NavCounts { uint32_t imu = 0, baro = 0, mag = 0, rows = 0; } counts;

struct Event { uint64_t t; std::string what; };
static std::vector<Event> simEvents;

static void navFeed(uint64_t now, const InRow& r, bool rowEnd)
{
    navImuNew = false;
    if (r.imuValid) {
        Vec3 a = r.accel;
        // High-g takeover. The log only has the accelerometer the board actually used (the KX134's
        // values are already in ax..az where the board switched), so this switch decides the
        // high_g column and nothing else.
        if (navHighGOk) {
            navHighGMag = r.highGMag;
            if (!navUsingHighG && navHighGMag > HIGHG_SWITCH_G) navUsingHighG = true;
            else if (navUsingHighG && navHighGMag < HIGHG_RETURN_G) navUsingHighG = false;
        } else {
            navUsingHighG = false;
        }
        nav.imu(now, a, r.gyro);
        counts.imu++;
        navAccel = a;
        navGyro = r.gyro;
        navImuNew = true;
    }
    Vec3 m;
    if (SIM_MAG_FROM_LOGGED_ATTITUDE && rowEnd && magFromAttitude(r.q, m)) {
        nav.magnetometer(now, m);
        counts.mag++;
    }
    if (rowEnd && std::isfinite(r.pressure) && r.pressure != navLastPressure) {   // a new barometer sample
        navLastPressure = r.pressure;
        if (!navBaroLocked) {
            nav.baroPressure(now, r.pressure);
            counts.baro++;
        }
    }
}

static void navPublish(uint64_t tUs, NavPhase phase)
{
    const float inv = navAvgN ? 1.0f / navAvgN : 0.0f;
    const float speed = nav.speed();
    float tK = navPadTempK - 0.0065f * (nav.altitude() > 0.0f ? nav.altitude() : 0.0f);
    const float rho = std::isfinite(navLastPressure) ? navLastPressure * 100.0f / (287.05f * tK) : NAN;
    const ekf::Quat& q = nav.quaternion();
    const NavEKF::RocketAngles ang = nav.rocketAngles();
    const Vec3& v = nav.velocityNED();
    const Vec3& ba = nav.accelBias();
    const Vec3& bg = nav.gyroBias();

    putU(tUs); putU(counts.rows++); putU((unsigned)phase); putU(nav.inFlight());
    putU((unsigned)flight.state()); putF(navMach, 3); putU(navBaroLocked);
    putU(navUsingHighG); putF(navHighGMag, 2);
    putF(navRateSum[0] * inv * NavEKF::RAD2DEG, 2); putF(navRateSum[1] * inv * NavEKF::RAD2DEG, 2);
    putF(navRateSum[2] * inv * NavEKF::RAD2DEG, 2); putF(navRollRateSum * inv, 2);
    putF(navAccSum[0] * inv, 3); putF(navAccSum[1] * inv, 3); putF(navAccSum[2] * inv, 3);
    putF(speed, 2); putF(navLastPressure, 3); putF(0.5f * rho * speed * speed, 1);
    putF(rollTest.deflection(), 2); putU(s_pulseUs[ROLL_TEST_SERVOS[0]]);
    putU((unsigned)rollTest.status()); putU(rollTest.doublet());
    putF(q.w, 6); putF(q.x, 6); putF(q.y, 6); putF(q.z, 6);
    putF(ang.roll, 2); putF(ang.pitch, 2); putF(ang.yaw, 2); putF(nav.tilt(), 2); putF(nav.tiltAzimuth(), 1);
    putF(nav.north(), 3); putF(nav.east(), 3); putF(nav.altitude(), 3);
    putF(v[0], 3); putF(v[1], 3); putF(v[2], 3);
    putF(std::isfinite(navLastPressure) ? nav.altitudeAt(navLastPressure) : NAN, 2);
    putF(nav.tiltStd(), 3); putF(nav.headingStd(), 3); putF(nav.altitudeStd(), 3); putF(nav.horizontalStd(), 3);
    putF(ba[0], 4); putF(ba[1], 4); putF(ba[2], 4);
    putF(bg[0] * NavEKF::RAD2DEG, 4); putF(bg[1] * NavEKF::RAD2DEG, 4); putF(bg[2] * NavEKF::RAD2DEG, 4);
    putF(0.0f, 1); putU(0);   // ekf_load_pct, ekf_max_us: board timing, not simulated
    putU(counts.imu); putU(counts.baro); putU(counts.mag); putU(0);   // dropped: none in the sim
    endRow();

    navRateSum = navAccSum = Vec3::zeros();
    navRollRateSum = 0.0f;
    navAvgN = 0;
}

static void printEvents(const std::vector<InRow>& rows)
{
    // The board's own state and roll-test changes, from the input
    std::vector<Event> orig;
    int lastState = 0, lastRoll = 0;
    for (const InRow& r : rows) {
        if (r.state >= 0 && r.state != lastState) {
            orig.push_back({r.t, std::string("STATE -> ") + flightStateName((FlightState)r.state)});
            lastState = r.state;
        }
        if (r.rollTest >= 0 && r.rollTest != lastRoll) {
            orig.push_back({r.t, std::string("ROLL TEST -> ") + rollTestStatusName((RollTest::Status)r.rollTest)});
            lastRoll = r.rollTest;
        }
    }
    auto list = [](const char* title, const std::vector<Event>& ev) {
        printf("%s\n", title);
        if (ev.empty()) printf("  (none)\n");
        for (const Event& e : ev) printf("  %9.3f s  %s\n", e.t * 1e-6, e.what.c_str());
    };
    list("\nOriginal flight (from the input CSV):", orig);
    list("Simulation:", simEvents);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        printf("usage: navsim <flight nav csv> [output csv]\n");
        return 1;
    }
    std::string outPath = argc >= 3 ? argv[2] : argv[1];
    if (argc < 3) {
        const size_t dot = outPath.rfind('.');
        if (dot != std::string::npos && dot > outPath.find_last_of("/\\") + 1) outPath.erase(dot);
        outPath += "_sim.csv";
    }

    std::vector<InRow> rows;
    if (!readFlight(argv[1], rows)) return 1;
    if (rows.size() < 2) { fprintf(stderr, "%s: not enough rows\n", argv[1]); return 1; }
    printf("Read %zu rows, %.1f s of data\n", rows.size(), (rows.back().t - rows.front().t) * 1e-6);

    if (NAV_SERIAL_STATE_CMD && NAV_SERIAL_STATE_MANUAL)
        printf("WARNING: NAV_SERIAL_STATE_CMD and NAV_SERIAL_STATE_MANUAL are on in sim/config.h: the automatic\n"
               "         state detection is paused and there are no \"state\" commands in a replay, so the\n"
               "         state will stay LAUNCHPAD. Turn NAV_SERIAL_STATE_MANUAL off to simulate the detectors.\n");

    // ---- Pad temperature ----
    if (SIM_PAD_TEMP_FROM_LOG && padTempFromLog(rows, navPadTempK)) {
        printf("Pad temperature from the log's mach/speed columns: %.1f C\n", navPadTempK - 273.15f);
    } else {
        navPadTempK = 273.15f + SIM_PAD_TEMP_C;
        printf("Pad temperature: %.1f C (SIM_PAD_TEMP_C%s)\n", SIM_PAD_TEMP_C,
               SIM_PAD_TEMP_FROM_LOG ? "; the log had no usable mach/speed values" : "");
    }

    // ---- 1. Average the sensors at rest, then start the filter ----
    // The board averages for NAV_ALIGN_AVG_MS before its first row; those samples aren't logged, so
    // the sim averages the first NAV_ALIGN_AVG_MS of rows instead (the board is still then too).
    const uint64_t t0 = rows.front().t;
    Vec3 accSum = Vec3::zeros(), magSum = Vec3::zeros();
    double pSum = 0.0;
    uint32_t accN = 0, magN = 0, pN = 0;
    for (const InRow& r : rows) {
        if (r.t - t0 > 5'000'000 || (r.t - t0 > (uint64_t)NAV_ALIGN_AVG_MS * 1000 && accN > 0)) break;
        if (r.imuValid) { accSum = accSum + r.accel; accN++; }
        if (std::isfinite(r.pressure)) { pSum += r.pressure; pN++; navLastPressure = r.pressure; }
        Vec3 m;
        if (SIM_MAG_FROM_LOGGED_ATTITUDE && magFromAttitude(r.q, m)) { magSum = magSum + m; magN++; }
    }
    if (accN == 0) { fprintf(stderr, "No usable IMU values in the first 5 s of the log\n"); return 1; }

    NavEKF::Config cfg;
    cfg.padTemperatureK = navPadTempK;
    cfg.magDeclinationDeg = MAG_DECLINATION_DEG;
    for (int i = 0; i < 3; i++) {
        cfg.rocketAxis[i] = ROCKET_AXIS[i];
        cfg.rocketRollRefAxis[i] = ROCKET_ROLL_REF_AXIS[i];
    }
    simEkfConfig(cfg);
    const float padP = pN ? (float)(pSum / pN) : 1013.25f;
    const Vec3 magAvg = magN ? magSum * (1.0f / magN) : Vec3::zeros();
    if (!nav.begin(padP, accSum * (1.0f / accN), magAvg, t0, cfg)) {
        fprintf(stderr, "NavEKF did not start: unusable accelerometer average\n");
        return 1;
    }

    // High-g takeover: the board's pad cross-check result isn't logged, but high_g_accel_g is only
    // non-zero when it passed
    bool anyHighG = false;
    for (const InRow& r : rows) anyHighG |= r.highGMag > 0.0f;
    navHighGOk = HIGHG_ENABLE && anyHighG;
    if (HIGHG_ENABLE && !anyHighG) printf("High-g: the log has no KX134 values (takeover was off on the board)\n");

    out = fopen(outPath.c_str(), "wb");
    if (!out) { fprintf(stderr, "Can't create %s\n", outPath.c_str()); return 1; }
    writeHeader();

    // ---- 2. Align while still, then 3. navigate ----
    servoCenterAll();   // servoInit() in startNavigation()
    const uint64_t start = 0;   // t_us counts from the nav task's start
    const uint64_t alignEnd = start + (uint64_t)NAV_ALIGN_MS * 1000;
    const uint64_t idleLogPeriod = 1'000'000 / NAV_LOG_IDLE_HZ;
    const uint64_t flightLogPeriod = 1'000'000 / NAV_LOG_FLIGHT_HZ;
    const uint64_t stationaryPeriod = 1'000'000 / NAV_STATIONARY_HZ;
    const float stationaryDt = 1.0f / NAV_STATIONARY_HZ;
    uint64_t nextLog = t0, nextStationary = t0;
    NavPhase phase = NavPhase::Aligning;
    float padPressure = padP;
    float maxGyroDps = 0.0f, maxAccelDevG = 0.0f;
    bool servosHeld = true;
    const uint64_t servosHeldSinceUs = start;
    RollTest::Status lastRollStatus = RollTest::Status::Waiting;

    for (size_t i = 1; i < rows.size(); i++) {
        const InRow& r = rows[i];
        const uint64_t tPrev = rows[i - 1].t;
        const uint64_t span = r.t - tPrev;
        const uint64_t steps = SIM_STEP_US ? std::max<uint64_t>(1, (span + SIM_STEP_US - 1) / SIM_STEP_US) : 1;

        for (uint64_t k = 1; k <= steps; k++) {
            const uint64_t now = tPrev + span * k / steps;
            navFeed(now, r, k == steps);

            navMach = machNumber(nav.speed(), nav.altitude(), navPadTempK);
            navBaroLocked = navMach >= BARO_LOCKOUT_MACH_MIN && navMach <= BARO_LOCKOUT_MACH_MAX;

            bool stateChanged = false;
            if (navImuNew) {
                const Vec3 a = navAccel - nav.accelBias();
                const Vec3 w = navGyro - nav.gyroBias();
                FlightInputs in;
                in.accelG = ekf::norm(a) / NavEKF::G;
                in.axialG = (a[0] * ROCKET_AXIS[0] + a[1] * ROCKET_AXIS[1] + a[2] * ROCKET_AXIS[2]) / NavEKF::G;
                in.gyroDps = ekf::norm(w) * NavEKF::RAD2DEG;
                in.altM = nav.altitude();
                in.vertSpeed = nav.verticalSpeed();

                if (in.gyroDps > maxGyroDps) maxGyroDps = in.gyroDps;
                if (fabsf(in.accelG - 1.0f) > maxAccelDevG) maxAccelDevG = fabsf(in.accelG - 1.0f);

                const float rollRateDps = (w[0] * ROCKET_AXIS[0] + w[1] * ROCKET_AXIS[1] + w[2] * ROCKET_AXIS[2]) * NavEKF::RAD2DEG;
                navRateSum = navRateSum + w;
                navAccSum = navAccSum + a;
                navRollRateSum += rollRateDps;
                navAvgN++;

                if (phase == NavPhase::Navigating) {
                    if (!(NAV_SERIAL_STATE_CMD && NAV_SERIAL_STATE_MANUAL))
                        stateChanged |= flight.update(now, in);
                    if (flight.state() != FlightState::Launchpad && !nav.inFlight()) nav.setInFlight(true);

                    if (ROLL_TEST_ENABLE) {
                        const float before = rollTest.deflection();
                        const float deg = rollTest.update(now, flight.state(), flight.stateSinceUs(), in.vertSpeed, rollRateDps);
                        if (deg != before)
                            for (int s : ROLL_TEST_SERVOS) servoWriteDeg(s, deg);
                    }

                    if (stateChanged) {
                        char msg[128];
                        snprintf(msg, sizeof msg, "STATE -> %s (alt %.1f m, vs %.1f m/s, M %.2f)",
                                 flightStateName(flight.state()), in.altM, in.vertSpeed, navMach);
                        simEvents.push_back({now, msg});
                    }
                    if (rollTest.status() != lastRollStatus) {
                        lastRollStatus = rollTest.status();
                        char msg[128];
                        snprintf(msg, sizeof msg, "ROLL TEST -> %s (doublet %u, speed %.1f m/s)",
                                 rollTestStatusName(lastRollStatus), rollTest.doublet(), nav.speed());
                        simEvents.push_back({now, msg});
                    }
                }
            }

            // Servo power
            if (SERVO_RELEASE_ON_PAD) {
                const FlightState fs = flight.state();
                const bool onGround = phase == NavPhase::Aligning || fs == FlightState::Launchpad ||
                                      (SERVO_RELEASE_AFTER_LANDING && fs == FlightState::Landed);
                if (onGround) {
                    if (servosHeld && now - servosHeldSinceUs >= (uint64_t)SERVO_PAD_CENTER_MS * 1000) {
                        servoReleaseAll();
                        servosHeld = false;
                    }
                } else if (!servosHeld) {
                    servoCenterAll();
                    servosHeld = true;
                }
            }

            // Pad hold
            if (PAD_HOLD && phase == NavPhase::Navigating && flight.state() == FlightState::Launchpad &&
                now >= nextStationary) {
                nextStationary += stationaryPeriod;
                if (nextStationary < now) nextStationary = now + stationaryPeriod;
                if (maxGyroDps < PAD_STILL_GYRO_DPS && maxAccelDevG < PAD_STILL_ACCEL_G) {
                    nav.stationary(now);
                    nav.horizontalPosition(now, 0.0f, 0.0f, PAD_HOLD_POS_STD_M);
                    if (std::isfinite(navLastPressure)) {
                        padPressure += (navLastPressure - padPressure) * (stationaryDt / PAD_PRESSURE_TAU_S);
                        nav.setPadPressure(padPressure);
                    }
                } else {
                    nav.clearStationaryWindow();
                }
                maxGyroDps = maxAccelDevG = 0.0f;
            }

            if (phase == NavPhase::Aligning) {
                if (now >= alignEnd) {
                    phase = NavPhase::Navigating;
                    if (NAV_FLIGHT_MODE_AFTER_ALIGN) nav.setInFlight(true);
                    simEvents.push_back({now, "aligned, navigating"});
                } else if (now >= nextStationary) {
                    nextStationary += stationaryPeriod;
                    nav.stationary(now);
                    maxGyroDps = maxAccelDevG = 0.0f;
                }
            }

            // Log rate
            const FlightState fs = flight.state();
            const bool flying = phase == NavPhase::Navigating && fs != FlightState::Launchpad && fs != FlightState::Landed;
            const uint64_t logPeriod = flying ? flightLogPeriod : idleLogPeriod;
            if (stateChanged) nextLog = now;
            if (now >= nextLog) {
                nextLog += logPeriod;
                if (nextLog < now) nextLog = now + logPeriod;
                navPublish(now - start, phase);
            }
        }
    }

    fclose(out);
    if (SIM_PRINT_EVENTS) printEvents(rows);
    printf("\nWrote %u rows to %s\n", counts.rows, outPath.c_str());
    return 0;
}
