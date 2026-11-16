#include "AltitudeEKF.h"
#include <math.h>

using ekf::Mat;
using ekf::Vec;

namespace {
// ISA troposphere
constexpr float LAPSE = 0.0065f;            // temperature lapse rate [K/m]
constexpr float BARO_EXP = 5.25588f;        // g*M / (R*L)
constexpr float INV_BARO_EXP = 1.0f / BARO_EXP;
} // namespace

// ============================================================================
// Process model: constant acceleration over dt, driven by the measured specific force
// ============================================================================
void AltitudeEKF::Model::propagate(Vec<3>& x, float dt) const {
    const float a = input - x[B] - G;   // vertical acceleration
    x[H] += x[V] * dt + 0.5f * a * dt * dt;
    x[V] += a * dt;
    // bias: random walk, unchanged by propagation
}

void AltitudeEKF::Model::jacobian(const Vec<3>&, float dt, Mat<3, 3>& F) const {
    F = Mat<3, 3>::identity();
    F(H, V) = dt;
    F(H, B) = -0.5f * dt * dt;
    F(V, B) = -dt;
}

void AltitudeEKF::Model::processNoise(const Vec<3>&, float dt, Mat<3, 3>& Q) const {
    // White acceleration noise (density qAccel) integrated into h and v, plus bias random walk
    const float dt2 = dt * dt;
    Q = Mat<3, 3>::zeros();
    Q(H, H) = qAccel * dt2 * dt / 3.0f;
    Q(H, V) = Q(V, H) = qAccel * dt2 / 2.0f;
    Q(V, V) = qAccel * dt;
    Q(B, B) = qBias * dt;
}

// ============================================================================
// Filter
// ============================================================================
void AltitudeEKF::begin(float padPressureMbar, uint64_t tUs, const Config& config) {
    cfg_ = config;
    padPressure_ = padPressureMbar;
    model_.input = G;
    setInFlight(false);
    model_.qBias = cfg_.accelBiasWalk * cfg_.accelBiasWalk;

    Vec<3> x0 = Vec<3>::zeros();
    const float p0[3] = {cfg_.initialAltitudeStd * cfg_.initialAltitudeStd,
                         cfg_.initialVelocityStd * cfg_.initialVelocityStd,
                         cfg_.initialBiasStd * cfg_.initialBiasStd};
    ekf_.reset(x0, Mat<3, 3>::diagonal(p0), tUs);
    ekf_.setMaxPredictStep(0.01f);
    baroNisSum_ = 0.0f;
    baroNisCount_ = 0;
}

void AltitudeEKF::begin(float padPressureMbar, uint64_t tUs) {
    begin(padPressureMbar, tUs, Config());
}

void AltitudeEKF::setInFlight(bool flying) {
    inFlight_ = flying;
    const float d = flying ? cfg_.accelNoiseDensity : cfg_.padAccelNoiseDensity;
    model_.qAccel = d * d;
}

void AltitudeEKF::accel(uint64_t tUs, float specificForce) {
    // The previous sample applies up to now (zero-order hold); the new one from now on.
    // A late sample (older than the state) only replaces the held input.
    ekf_.predictTo(model_, tUs);
    if (!isfinite(specificForce)) return;
    model_.input = specificForce;
    if (!inFlight_ && acceleration() > cfg_.launchAccel) setInFlight(true);
}

float AltitudeEKF::pressureAt(float h) const {
    float base = 1.0f - LAPSE * h / cfg_.padTemperatureK;
    if (base < 0.05f) base = 0.05f;   // far outside the model; keep it finite
    return padPressure_ * powf(base, BARO_EXP);
}

float AltitudeEKF::altitudeAt(float mbar) const {
    return cfg_.padTemperatureK / LAPSE * (1.0f - powf(mbar / padPressure_, INV_BARO_EXP));
}

ekf::UpdateResult AltitudeEKF::baroPressure(uint64_t tUs, float mbar) {
    if (!isfinite(mbar) || !ekf_.syncTo(model_, tUs, cfg_.maxLagUs)) return {ekf::UpdateStatus::Stale, NAN};

    // h(x) = p(h) and its derivative dp/dh (the EKF linearization), at the time the sample was
    // taken: a late sample (behind the state) saw the altitude h - v*lag
    const float lag = tUs < ekf_.timeUs() ? (float)(ekf_.timeUs() - tUs) * 1e-6f : 0.0f;
    const float h = ekf_.x(H) - ekf_.x(V) * lag;
    float base = 1.0f - LAPSE * h / cfg_.padTemperatureK;
    if (base < 0.05f) base = 0.05f;
    const float predicted = padPressure_ * powf(base, BARO_EXP);
    const float dpdh = -padPressure_ * BARO_EXP * powf(base, BARO_EXP - 1.0f) * LAPSE / cfg_.padTemperatureK;

    Mat<1, 3> Hm = Mat<1, 3>::zeros();
    Hm(0, H) = dpdh;

    float sigma = cfg_.baroNoiseMbar;
    if (cfg_.baroLockoutSpeed > 0.0f && fabsf(ekf_.x(V)) > cfg_.baroLockoutSpeed) sigma *= cfg_.baroLockoutScale;

    ekf::UpdateResult r = ekf_.update1(mbar - predicted, Hm, sigma * sigma, cfg_.baroGate);
    if (r.accepted()) {
        baroNisSum_ += r.nis;
        baroNisCount_++;
    }
    return r;
}

ekf::UpdateResult AltitudeEKF::altitude(uint64_t tUs, float meters, float stdM) {
    if (!isfinite(meters) || !ekf_.syncTo(model_, tUs, cfg_.maxLagUs)) return {ekf::UpdateStatus::Stale, NAN};
    Mat<1, 3> Hm = Mat<1, 3>::zeros();
    Hm(0, H) = 1.0f;
    return ekf_.update1(meters - ekf_.x(H), Hm, stdM * stdM, ekf::GATE_1DOF_999);
}

ekf::UpdateResult AltitudeEKF::zeroVelocity(uint64_t tUs) {
    if (!ekf_.syncTo(model_, tUs, cfg_.maxLagUs)) return {ekf::UpdateStatus::Stale, NAN};
    Mat<1, 3> Hm = Mat<1, 3>::zeros();
    Hm(0, V) = 1.0f;
    return ekf_.update1(0.0f - ekf_.x(V), Hm, cfg_.zeroVelocityStd * cfg_.zeroVelocityStd);
}
