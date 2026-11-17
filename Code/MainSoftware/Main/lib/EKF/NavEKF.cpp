#include "NavEKF.h"
#include <math.h>

using ekf::Mat;
using ekf::Mat3;
using ekf::Quat;
using ekf::Vec;
using ekf::Vec3;

namespace {
constexpr int N = NavEKF::NSTATES;

// ISA troposphere
constexpr float LAPSE = 0.0065f;       // [K/m]
constexpr float BARO_EXP = 5.25588f;   // g*M / (R*L)

/** Writes the 3x3 block B into M at (r, c). */
template <int R, int C>
void setBlock(Mat<R, C>& M, int r, int c, const Mat3& B) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) M(r + i, c + j) = B(i, j);
}
template <int R, int C>
void setDiag3(Mat<R, C>& M, int r, int c, float v) {
    for (int i = 0; i < 3; i++) M(r + i, c + i) = v;
}
bool finite3(const Vec3& v) { return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]); }
} // namespace

// ============================================================================
// Initialization
// ============================================================================
bool NavEKF::begin(float padPressureMbar, const Vec3& accelBody, const Vec3& magBody, uint64_t tUs,
                   const Config& config) {
    cfg_ = config;
    padPressure_ = padPressureMbar;
    model_.nav = this;
    axis_ = ekf::normalized(ekf::vec3(cfg_.rocketAxis[0], cfg_.rocketAxis[1], cfg_.rocketAxis[2]));
    if (ekf::norm(axis_) == 0.0f) axis_ = ekf::vec3(1, 0, 0);
    {   // Roll reference: the configured axis with its nose component removed
        Vec3 ref = ekf::vec3(cfg_.rocketRollRefAxis[0], cfg_.rocketRollRefAxis[1], cfg_.rocketRollRefAxis[2]);
        ref = ref - axis_ * ekf::dot(ref, axis_);
        if (ekf::norm(ref) < 1e-3f) {   // missing or parallel to the nose: use any perpendicular axis
            ref = fabsf(axis_[0]) < 0.9f ? ekf::vec3(1, 0, 0) : ekf::vec3(0, 1, 0);
            ref = ref - axis_ * ekf::dot(ref, axis_);
        }
        rollRef_ = ekf::normalized(ref);
    }

    if (!finite3(accelBody) || ekf::norm(accelBody) < 0.5f * G) return false;

    // ---- Attitude from gravity and the magnetic field (TRIAD) ----
    // At rest the accelerometer measures +G pointing up, so "down" in the body frame is -accel.
    const Vec3 down = ekf::normalized(accelBody * -1.0f);
    magEnabled_ = finite3(magBody) && ekf::norm(magBody) > 0.0f;
    Vec3 east = magEnabled_ ? ekf::cross(down, magBody) : Vec3::zeros();
    if (ekf::norm(east) < 1e-3f * ekf::norm(magBody)) {   // no mag (or field parallel to gravity)
        magEnabled_ = false;
        // Pick a heading: the horizontal projection of the body axis least aligned with gravity
        Vec3 ref = fabsf(down[0]) < 0.9f ? ekf::vec3(1, 0, 0) : ekf::vec3(0, 1, 0);
        Vec3 northGuess = ref - down * ekf::dot(ref, down);
        east = ekf::cross(down, northGuess);
    }
    east = ekf::normalized(east);
    const Vec3 north = ekf::cross(east, down);
    Mat3 Rmag;   // rows = NED axes in body coordinates -> maps body to (magnetic) NED
    for (int j = 0; j < 3; j++) {
        Rmag(0, j) = north[j];
        Rmag(1, j) = east[j];
        Rmag(2, j) = down[j];
    }
    // Magnetic north lies at azimuth +declination: rotate by the declination about down
    const float d = cfg_.magDeclinationDeg * DEG2RAD;
    Mat3 Rz = Mat3::identity();
    Rz(0, 0) = cosf(d); Rz(0, 1) = -sinf(d);
    Rz(1, 0) = sinf(d); Rz(1, 1) = cosf(d);
    q_ = Quat::fromMatrix(Rz * Rmag);
    const Mat3 R = q_.toMatrix();

    if (magEnabled_) {
        magRef_ = R * ekf::normalized(magBody);   // field direction in NED, consistent with q_
        magStrength_ = ekf::norm(magBody);
    }

    // ---- Nominal state ----
    p_ = v_ = Vec3::zeros();
    ba_ = bg_ = Vec3::zeros();
    tUs_ = tUs;
    accel_ = accelBody;
    gyro_ = Vec3::zeros();
    haveImu_ = false;
    covDt_ = 0.0f;
    covDv_ = covDa_ = Vec3::zeros();
    gyroSum_ = Vec3::zeros();
    gyroCount_ = 0;
    magAvg_ = Vec3::zeros();
    magCount_ = 0;
    inFlight_ = false;
    stale_ = 0;
    magDisturbed_ = 0;
    baroNisSum_ = magNisSum_ = 0.0f;
    baroNisN_ = magNisN_ = 0;

    // ---- Initial covariance ----
    Mat<N, N> P0 = Mat<N, N>::zeros();
    setDiag3(P0, PN, PN, cfg_.initialPositionStd * cfg_.initialPositionStd);
    setDiag3(P0, VN, VN, cfg_.initialVelocityStd * cfg_.initialVelocityStd);
    // Attitude: tilt / heading uncertainty is defined in NED; the error state is in the body frame
    const float tilt = cfg_.initialTiltStdDeg * DEG2RAD;
    const float head = (magEnabled_ ? cfg_.initialHeadingStdDeg : 180.0f) * DEG2RAD;
    const float pNed[3] = {tilt * tilt, tilt * tilt, head * head};
    setBlock(P0, TX, TX, R.T() * Mat3::diagonal(pNed) * R);
    setDiag3(P0, BAX, BAX, cfg_.initialAccelBiasStd * cfg_.initialAccelBiasStd);
    setDiag3(P0, BGX, BGX, cfg_.initialGyroBiasStd * cfg_.initialGyroBiasStd);

    ekf_.reset(Vec<N>::zeros(), P0, tUs);
    ekf_.setJosephForm(false);
    ekf_.setMinVariance(1e-10f);
    ekf_.setSoftGate(cfg_.softGates, cfg_.softGateExponent);
    return true;
}

bool NavEKF::begin(float padPressureMbar, const Vec3& accelBody, const Vec3& magBody, uint64_t tUs) {
    return begin(padPressureMbar, accelBody, magBody, tUs, Config());
}

// ============================================================================
// Prediction
// ============================================================================
void NavEKF::integrate(float dt) {
    const Vec3 a = accel_ - ba_;           // bias-corrected specific force (body)
    const Vec3 dTheta = (gyro_ - bg_) * dt;
    const Mat3 R = q_.toMatrix();

    // Specific force rotated with the mid-interval attitude (R * (I + [dTheta/2]x))
    const Vec3 aMid = a + ekf::cross(dTheta, a) * 0.5f;
    Vec3 accNed = R * aMid;
    accNed[2] += G;

    p_ = p_ + v_ * dt + accNed * (0.5f * dt * dt);
    v_ = v_ + accNed * dt;
    q_ = (q_ * Quat::fromRotationVector(dTheta)).normalized();

    covDt_ += dt;
    covDv_ = covDv_ + a * dt;
    covDa_ = covDa_ + dTheta;
    if (covDt_ >= cfg_.covarianceStepS) flushCovariance();
}

void NavEKF::advanceTo(uint64_t tUs) {
    if (tUs <= tUs_) return;
    float remaining = (float)(tUs - tUs_) * 1e-6f;
    const float maxStep = cfg_.covarianceStepS > 0.0f ? cfg_.covarianceStepS : 0.005f;
    while (remaining > 0.0f) {   // long gaps (missing IMU) are integrated in short steps
        const float dt = remaining > maxStep ? maxStep : remaining;
        integrate(dt);
        remaining -= dt;
    }
    tUs_ = tUs;
}

void NavEKF::flushCovariance() {
    if (covDt_ <= 0.0f) return;
    ekf_.predict(model_, covDt_);
    covDt_ = 0.0f;
    covDv_ = covDa_ = Vec3::zeros();
}

void NavEKF::predictTo(uint64_t tUs) { advanceTo(tUs); }

void NavEKF::imu(uint64_t tUs, const Vec3& accel, const Vec3& gyro) {
    // The previous sample applies up to now (zero-order hold), the new one from now on.
    advanceTo(tUs);
    if (!finite3(accel) || !finite3(gyro)) return;
    accel_ = accel;
    gyro_ = gyro;
    haveImu_ = true;
    gyroSum_ = gyroSum_ + gyro;
    gyroCount_++;

    if (!inFlight_ && -accelerationNED()[2] > cfg_.launchAccel) setInFlight(true);
}

void NavEKF::setInFlight(bool flying) {
    if (flying && !inFlight_ && magEnabled_ && magCount_ > 0) {
        // Re-anchor the field reference to the final pad attitude (tilt from gravity, heading
        // already corrected by the mag), so 3D mag fusion in flight doesn't fight the pad alignment.
        magRef_ = q_.toMatrix() * ekf::normalized(magAvg_);
        magStrength_ = ekf::norm(magAvg_);
    }
    inFlight_ = flying;
}

/*
 * Error-state dynamics (body-frame attitude error), over an interval T with
 * dV = sum (f - ba) dt  and  dA = sum (w - bg) dt  (body frame):
 *   dp' = dv
 *   dv' = -R [a]x dtheta - R dba
 *   dtheta' = -[w]x dtheta - dbg
 *   dba' = noise, dbg' = noise
 */
void NavEKF::CovModel::jacobian(const Vec<N>&, float T, Mat<N, N>& F) const {
    const Mat3 R = nav->q_.toMatrix();
    const Mat3 RskewDv = R * ekf::skew(nav->covDv_);
    F = Mat<N, N>::identity();
    setDiag3(F, PN, VN, T);
    setBlock(F, PN, TX, RskewDv * (-0.5f * T));
    setBlock(F, PN, BAX, R * (-0.5f * T * T));
    setBlock(F, VN, TX, RskewDv * -1.0f);
    setBlock(F, VN, BAX, R * -T);
    setBlock(F, TX, TX, Mat3::identity() - ekf::skew(nav->covDa_));
    setDiag3(F, TX, BGX, -T);
}

void NavEKF::CovModel::processNoise(const Vec<N>&, float T, Mat<N, N>& Q) const {
    const Config& c = nav->cfg_;
    const float an = nav->inFlight_ ? c.accelNoiseDensity : c.padAccelNoiseDensity;
    Q = Mat<N, N>::zeros();
    // White accel noise integrated into v (and p); isotropic, so R Q R^T = Q
    setDiag3(Q, PN, PN, an * an * T * T * T / 3.0f);
    setDiag3(Q, PN, VN, an * an * T * T / 2.0f);
    setDiag3(Q, VN, PN, an * an * T * T / 2.0f);
    setDiag3(Q, VN, VN, an * an * T);
    setDiag3(Q, TX, TX, c.gyroNoiseDensity * c.gyroNoiseDensity * T);
    setDiag3(Q, BAX, BAX, c.accelBiasWalk * c.accelBiasWalk * T);
    setDiag3(Q, BGX, BGX, c.gyroBiasWalk * c.gyroBiasWalk * T);
}

// ============================================================================
// Updates
// ============================================================================
bool NavEKF::sync(uint64_t tUs, float& lagS) {
    lagS = 0.0f;
    if (tUs >= tUs_) {
        advanceTo(tUs);
    } else if (tUs_ - tUs > cfg_.maxLagUs) {
        stale_++;
        return false;
    } else {
        lagS = (float)(tUs_ - tUs) * 1e-6f;
    }
    flushCovariance();   // P must be current before an update
    return true;
}

void NavEKF::injectError() {
    Vec<N>& x = ekf_.mutableX();
    for (int i = 0; i < 3; i++) {
        p_[i] += x[PN + i];
        v_[i] += x[VN + i];
        ba_[i] += x[BAX + i];
        bg_[i] += x[BGX + i];
    }
    q_ = (q_ * Quat::fromRotationVector(ekf::vec3(x[TX], x[TY], x[TZ]))).normalized();
    x = Vec<N>::zeros();
}

void NavEKF::accumulateNis(const ekf::UpdateResult& r, float& sum, uint32_t& n) {
    if (!r.accepted()) return;
    injectError();
    if (r.status == ekf::UpdateStatus::Deweighted) return;   // outliers would swamp the consistency average
    sum += r.nis;
    n++;
}

float NavEKF::pressureAt(float h) const {
    float base = 1.0f - LAPSE * h / cfg_.padTemperatureK;
    if (base < 0.05f) base = 0.05f;
    return padPressure_ * powf(base, BARO_EXP);
}

float NavEKF::altitudeAt(float mbar) const {
    return cfg_.padTemperatureK / LAPSE * (1.0f - powf(mbar / padPressure_, 1.0f / BARO_EXP));
}

ekf::UpdateResult NavEKF::baroPressure(uint64_t tUs, float mbar) {
    float lag;
    if (!isfinite(mbar) || !sync(tUs, lag)) return {ekf::UpdateStatus::Stale, NAN};
    const float h = altitude() - verticalSpeed() * lag;   // altitude when the sample was taken
    float base = 1.0f - LAPSE * h / cfg_.padTemperatureK;
    if (base < 0.05f) base = 0.05f;
    const float predicted = padPressure_ * powf(base, BARO_EXP);
    const float dpdh = -padPressure_ * BARO_EXP * powf(base, BARO_EXP - 1.0f) * LAPSE / cfg_.padTemperatureK;

    Mat<1, N> H = Mat<1, N>::zeros();
    H(0, PD) = -dpdh;   // h = -down
    float sigma = cfg_.baroNoiseMbar;
    if (cfg_.baroLockoutSpeed > 0.0f && fabsf(v_[2]) > cfg_.baroLockoutSpeed) sigma *= cfg_.baroLockoutScale;

    ekf::UpdateResult r;
    if (cfg_.baroVerticalOnly) {
        static constexpr bool MASK[N] = {false, false, true,  false, false, true,  false, false, false,
                                         true,  true,  true,  false, false, false};   // PD, VD, accel bias
        Vec<1> y;
        y[0] = mbar - predicted;
        Mat<1, 1> R;
        R(0, 0) = sigma * sigma;
        r = ekf_.updateMasked<1>(y, H, R, MASK, cfg_.baroGate);
    } else {
        r = ekf_.update1(mbar - predicted, H, sigma * sigma, cfg_.baroGate);
    }
    accumulateNis(r, baroNisSum_, baroNisN_);
    return r;
}

ekf::UpdateResult NavEKF::magnetometer(uint64_t tUs, const Vec3& field) {
    if (!magEnabled_ || !finite3(field)) return {ekf::UpdateStatus::Stale, NAN};
    // Disturbed field (motor, pyro wiring, ...): wrong strength. With soft gates the reading is
    // trusted less the further off it is (noise x deviation / tolerance); without, it is rejected.
    const float strength = ekf::norm(field);
    const float allowed = cfg_.magStrengthTolerance * magStrength_;
    const float deviation = fabsf(strength - magStrength_);
    float noiseScale = 1.0f;
    if (deviation > allowed) {
        magDisturbed_++;
        if (!cfg_.softGates || !(allowed > 0.0f)) return {ekf::UpdateStatus::Gated, NAN};
        noiseScale = deviation / allowed;
    }
    float lag;
    if (!sync(tUs, lag)) return {ekf::UpdateStatus::Stale, NAN};
    if (!inFlight_) return padHeading(field, noiseScale);

    // Predicted direction in the body frame: m = R^T m_ref. With q_true = q exp(dtheta):
    // m_true = (I - [dtheta]x) m = m + [m]x dtheta  ->  H = [m]x on the attitude error.
    // For a late sample the body was rotated back by w*lag: m_then = m + (w*lag) x m.
    Vec3 predicted = q_.toMatrix().T() * magRef_;
    predicted = predicted + ekf::cross((gyro_ - bg_) * lag, predicted);
    const Vec3 y = ekf::normalized(field) - predicted;
    Mat<3, N> H = Mat<3, N>::zeros();
    setBlock(H, 0, TX, ekf::skew(predicted));
    const float sigma = cfg_.magNoise * noiseScale;
    const float r2 = sigma * sigma;
    const float rd[3] = {r2, r2, r2};

    ekf::UpdateResult r = ekf_.update<3>(y, H, Mat3::diagonal(rd), cfg_.magGate);
    accumulateNis(r, magNisSum_, magNisN_);
    return r;
}

ekf::UpdateResult NavEKF::stationary(uint64_t tUs) {
    float lag;
    if (!sync(tUs, lag)) return {ekf::UpdateStatus::Stale, NAN};
    // 6-D measurement: velocity = 0, and mean gyro since the last call = gyro bias (rate = 0)
    Vec<6> y;
    Mat<6, N> H = Mat<6, N>::zeros();
    const Vec3 meanGyro = gyroCount_ ? gyroSum_ * (1.0f / (float)gyroCount_) : bg_;
    for (int i = 0; i < 3; i++) {
        y[i] = -v_[i];
        y[3 + i] = meanGyro[i] - bg_[i];
        H(i, VN + i) = 1.0f;
        H(3 + i, BGX + i) = 1.0f;
    }
    const float zv = cfg_.zeroVelocityStd * cfg_.zeroVelocityStd, zr = cfg_.zeroRateStd * cfg_.zeroRateStd;
    const float rd[6] = {zv, zv, zv, zr, zr, zr};
    gyroSum_ = Vec3::zeros();
    gyroCount_ = 0;

    ekf::UpdateResult r = ekf_.update<6>(y, H, Mat<6, 6>::diagonal(rd));
    if (r.accepted()) injectError();
    return r;
}

/*
 * Heading-only magnetometer update (on the pad). Project the measured field into the world frame
 * with the current attitude and compare its azimuth with the reference's. With the world-frame
 * attitude error phi = R dtheta (true = Exp(phi) * estimate), the projected field is rotated by
 * -phi, so  azimuth(measured) - azimuth(reference) = -phi_z = -R.row(2) . dtheta.
 */
ekf::UpdateResult NavEKF::padHeading(const Vec3& field, float noiseScale) {
    magAvg_ = magCount_ == 0 ? field : magAvg_ + (field - magAvg_) * 0.02f;   // ~50-sample average
    magCount_++;

    const Mat3 R = q_.toMatrix();
    const Vec3 mw = R * ekf::normalized(field);
    const float hMeas = sqrtf(mw[0] * mw[0] + mw[1] * mw[1]);
    const float hRef = sqrtf(magRef_[0] * magRef_[0] + magRef_[1] * magRef_[1]);
    if (hMeas < 0.05f || hRef < 0.05f) return {ekf::UpdateStatus::Skipped, NAN};   // field ~vertical

    float y = atan2f(mw[1], mw[0]) - atan2f(magRef_[1], magRef_[0]);
    if (y > 3.14159265f) y -= 6.28318531f;
    if (y < -3.14159265f) y += 6.28318531f;
    Mat<1, N> H = Mat<1, N>::zeros();
    for (int j = 0; j < 3; j++) H(0, TX + j) = -R(2, j);
    const float sigma = cfg_.magNoise * noiseScale / hRef;   // direction noise -> azimuth noise

    ekf::UpdateResult r = ekf_.update1(y, H, sigma * sigma, ekf::GATE_1DOF_999);
    if (r.accepted()) injectError();
    return r;
}

ekf::UpdateResult NavEKF::horizontalPosition(uint64_t tUs, float north, float east, float stdM) {
    float lag;
    if (!isfinite(north) || !isfinite(east) || !sync(tUs, lag)) return {ekf::UpdateStatus::Stale, NAN};
    Vec<2> y;
    y[0] = north - (p_[0] - v_[0] * lag);
    y[1] = east - (p_[1] - v_[1] * lag);
    Mat<2, N> H = Mat<2, N>::zeros();
    H(0, PN) = 1.0f;
    H(1, PE) = 1.0f;
    const float r2 = stdM * stdM;
    const float rd[2] = {r2, r2};
    ekf::UpdateResult r = ekf_.update<2>(y, H, Mat<2, 2>::diagonal(rd));
    if (r.accepted()) injectError();
    return r;
}

/*
 * Nose direction in NED from rocketAngles(): yaw = atan2(north, up), pitch = asin(east), so
 *   nose = (cos p sin y, sin p, -cos p cos y).
 * The north and east components are measured. With q_true = q exp(dtheta), the true nose is
 * R (I + [dtheta]x) a = R a - R [a]x dtheta, so H = -(R [a]x) (rows N, E) on the attitude error.
 */
ekf::UpdateResult NavEKF::noseDirection(uint64_t tUs, float pitchDeg, float yawDeg, float stdDeg) {
    float lag;
    if (!isfinite(pitchDeg) || !isfinite(yawDeg) || !(stdDeg > 0.0f) || !sync(tUs, lag))
        return {ekf::UpdateStatus::Stale, NAN};
    const float p = pitchDeg * DEG2RAD, y = yawDeg * DEG2RAD;
    const Mat3 R = q_.toMatrix();
    const Vec3 nose = R * axis_;
    Vec<2> innov;
    innov[0] = cosf(p) * sinf(y) - nose[0];
    innov[1] = sinf(p) - nose[1];
    const Mat3 HR = R * ekf::skew(axis_) * -1.0f;
    Mat<2, N> H = Mat<2, N>::zeros();
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++) H(i, TX + j) = HR(i, j);
    const float s = stdDeg * DEG2RAD;   // small angles: a unit-vector component ~ an angle in rad
    const float rd[2] = {s * s, s * s};
    ekf::UpdateResult r = ekf_.update<2>(innov, H, Mat<2, 2>::diagonal(rd));
    if (r.accepted()) injectError();
    return r;
}

/*
 * Leveling: at rest the specific force points straight up, i.e. (0, 0, -1) in NED. Predicted in the
 * body frame: u = R^T (0, 0, -1). With q_true = q exp(dtheta): u_true = u + [u]x dtheta, so
 * H = [u]x on the attitude error (rotation about the vertical is unobservable, as it should be).
 */
ekf::UpdateResult NavEKF::gravityDirection(uint64_t tUs, const Vec3& accelBody, float stdDeg) {
    float lag;
    if (!finite3(accelBody) || !(stdDeg > 0.0f) || !sync(tUs, lag)) return {ekf::UpdateStatus::Stale, NAN};
    const Vec3 f = accelBody - ba_;
    if (ekf::norm(f) < 0.5f * G) return {ekf::UpdateStatus::Skipped, NAN};   // not a gravity reading
    const Vec3 predicted = q_.toMatrix().T() * ekf::vec3(0.0f, 0.0f, -1.0f);
    const Vec3 y = ekf::normalized(f) - predicted;
    Mat<3, N> H = Mat<3, N>::zeros();
    setBlock(H, 0, TX, ekf::skew(predicted));
    const float s = stdDeg * DEG2RAD;
    const float rd[3] = {s * s, s * s, s * s};
    ekf::UpdateResult r = ekf_.update<3>(y, H, Mat3::diagonal(rd), ekf::GATE_3DOF_999);
    if (r.accepted()) injectError();
    return r;
}

ekf::UpdateResult NavEKF::horizontalVelocity(uint64_t tUs, float vn, float ve, float stdMps) {
    float lag;
    if (!isfinite(vn) || !isfinite(ve) || !(stdMps > 0.0f) || !sync(tUs, lag))
        return {ekf::UpdateStatus::Stale, NAN};
    Vec<2> y;
    y[0] = vn - v_[0];
    y[1] = ve - v_[1];
    Mat<2, N> H = Mat<2, N>::zeros();
    H(0, VN) = 1.0f;
    H(1, VE) = 1.0f;
    const float r2 = stdMps * stdMps;
    const float rd[2] = {r2, r2};
    ekf::UpdateResult r = ekf_.update<2>(y, H, Mat<2, 2>::diagonal(rd));   // a weak prior: never gated
    if (r.accepted()) injectError();
    return r;
}

void NavEKF::setPadPressure(float mbar) {
    if (isfinite(mbar) && mbar > 0.0f) padPressure_ = mbar;
}

ekf::UpdateResult NavEKF::position(uint64_t tUs, float north, float east, float up, float stdM) {
    float lag;
    if (!isfinite(north) || !isfinite(east) || !isfinite(up) || !sync(tUs, lag)) return {ekf::UpdateStatus::Stale, NAN};
    const Vec3 y = ekf::vec3(north, east, -up) - (p_ - v_ * lag);
    Mat<3, N> H = Mat<3, N>::zeros();
    setDiag3(H, 0, PN, 1.0f);
    const float r2 = stdM * stdM;
    const float rd[3] = {r2, r2, r2};
    ekf::UpdateResult r = ekf_.update<3>(y, H, Mat3::diagonal(rd), ekf::GATE_3DOF_999);
    if (r.accepted()) injectError();
    return r;
}

// ============================================================================
// Outputs
// ============================================================================
Vec3 NavEKF::accelerationNED() const {
    Vec3 a = q_.toMatrix() * (accel_ - ba_);
    a[2] += G;
    return a;
}

float NavEKF::roll() const {
    const Mat3 R = q_.toMatrix();
    return atan2f(R(2, 1), R(2, 2)) * RAD2DEG;
}

float NavEKF::pitch() const {
    const Mat3 R = q_.toMatrix();
    float s = -R(2, 0);
    s = s > 1.0f ? 1.0f : (s < -1.0f ? -1.0f : s);
    return asinf(s) * RAD2DEG;
}

float NavEKF::yaw() const {
    const Mat3 R = q_.toMatrix();
    float y = atan2f(R(1, 0), R(0, 0)) * RAD2DEG;
    return y < 0.0f ? y + 360.0f : y;
}

NavEKF::RocketAngles NavEKF::rocketAngles() const {
    // Rocket frame {nose, roll ref, nose x ref} in NED, expressed in the reference frame
    // V = {up, north, west}: C(i, j) = V_i . rocket_j, where up = -D, north = N, west = -E.
    const Mat3 R = q_.toMatrix();
    const Vec3 nose = R * axis_;
    const Vec3 ref = R * rollRef_;
    const Vec3 third = ekf::cross(nose, ref);
    const float c00 = -nose[2], c10 = nose[0], c20 = -nose[1];
    const float c21 = -ref[1], c22 = -third[1];
    float s = -c20;
    s = s > 1.0f ? 1.0f : (s < -1.0f ? -1.0f : s);
    RocketAngles a;
    a.roll = atan2f(c21, c22) * RAD2DEG;
    a.pitch = asinf(s) * RAD2DEG;
    a.yaw = atan2f(c10, c00) * RAD2DEG;
    return a;
}

Vec3 NavEKF::axisNED() const { return q_.toMatrix() * axis_; }

float NavEKF::tilt() const {
    float up = -axisNED()[2];
    up = up > 1.0f ? 1.0f : (up < -1.0f ? -1.0f : up);
    return acosf(up) * RAD2DEG;
}

float NavEKF::tiltAzimuth() const {
    const Vec3 a = axisNED();
    float az = atan2f(a[1], a[0]) * RAD2DEG;
    return az < 0.0f ? az + 360.0f : az;
}

float NavEKF::tiltStd() const {
    const Mat3 R = q_.toMatrix();
    Mat3 Pb;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) Pb(i, j) = ekf_.P()(TX + i, TX + j);
    const Mat3 Pn = R * Pb * R.T();
    return sqrtf(Pn(0, 0) + Pn(1, 1)) * RAD2DEG;
}

float NavEKF::headingStd() const {
    const Mat3 R = q_.toMatrix();
    Mat3 Pb;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) Pb(i, j) = ekf_.P()(TX + i, TX + j);
    const Mat3 Pn = R * Pb * R.T();
    return sqrtf(Pn(2, 2)) * RAD2DEG;
}
