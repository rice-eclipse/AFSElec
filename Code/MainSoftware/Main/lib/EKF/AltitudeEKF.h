/**
 * AltitudeEKF.h - Asynchronous EKF for rocket altitude and vertical velocity.
 *
 * State (3):  h  altitude above the pad [m]
 *             v  vertical velocity, up positive [m/s]
 *             b  accelerometer bias along the vertical axis [m/s^2]
 *
 * Inputs and measurements, each fused whenever it arrives:
 *   accel(t, f)         vertical specific force from the accelerometer [m/s^2], +9.81 at rest.
 *                       Drives the prediction (vertical acceleration = f - b - g).
 *   baroPressure(t, p)  raw static pressure [mbar]. Nonlinear measurement through the
 *                       barometric formula (this is what makes it an *extended* KF).
 *   altitude(t, h, s)   any direct altitude measurement (e.g. GPS) with its std dev.
 *   zeroVelocity(t)     "the rocket is not moving" (on the pad); makes the accel bias observable.
 *
 *   AltitudeEKF ekf;
 *   ekf.begin(padPressureMbar, time_us_64());
 *   // IMU loop:   ekf.accel(tUs, axialAccel_g * AltitudeEKF::G);
 *   // baro ready: ekf.baroPressure(tUs, baro.getPressure());
 *   // on the pad: ekf.zeroVelocity(tUs);
 *   float h = ekf.altitude(), v = ekf.velocity();
 *
 * "Vertical" is whatever axis you feed in. For a near-vertical flight the rocket's long axis is a
 * good approximation (the error is 1 - cos(tilt): 1.5 % at 10 deg). For large tilts, rotate the
 * accelerometer vector into the earth frame with an attitude estimate first.
 *
 * The barometric model is the ISA troposphere (valid below 11 km), referenced to the pad pressure
 * and temperature given to begin().
 */
#pragma once

#include <stdint.h>
#include "AsyncEKF.h"

class AltitudeEKF {
public:
    static constexpr float G = 9.80665f;   // standard gravity [m/s^2]

    struct Config {
        /**
         * Accelerometer white-noise density in flight [(m/s^2)/sqrt(Hz)]. Also absorbs tilt,
         * misalignment and vibration error, so it is much larger than the sensor's own noise.
         */
        float accelNoiseDensity = 0.5f;
        /**
         * Noise density used on the pad, before launch is detected. Small, so zeroVelocity()
         * updates can pin down the accelerometer bias in a few seconds.
         */
        float padAccelNoiseDensity = 0.02f;
        /** Launch is detected (and the flight noise level used) when the vertical acceleration exceeds this [m/s^2]. */
        float launchAccel = 2.0f * 9.80665f;
        /** Accelerometer bias random walk [(m/s^2)/sqrt(s)]. */
        float accelBiasWalk = 0.02f;
        /** Barometer noise, 1 sigma [mbar]. MS5611: ~0.065 at OSR256, ~0.012 at OSR4096 (plus turbulence). */
        float baroNoiseMbar = 0.1f;
        /** Baro outlier gate (NIS, 1 dof). Default ~4 sigma. GATE_OFF disables. */
        float baroGate = 16.0f;
        /**
         * Above this |vertical speed| [m/s] the baro noise is multiplied by baroLockoutScale, since
         * transonic shock waves corrupt static pressure. 0 = never.
         */
        float baroLockoutSpeed = 0.0f;
        float baroLockoutScale = 100.0f;
        /** Measurements up to this late (behind the state time) are still fused [us]. */
        uint32_t maxLagUs = 50'000;
        /** Pad air temperature [K] for the barometric formula (e.g. 273.15 + MS5611 temperature). */
        float padTemperatureK = 288.15f;
        /** Initial uncertainty. */
        float initialAltitudeStd = 1.0f;
        float initialVelocityStd = 0.1f;
        float initialBiasStd = 0.5f;
        /** Std dev used by zeroVelocity() [m/s]. */
        float zeroVelocityStd = 0.02f;
    };

    enum : int { H = 0, V = 1, B = 2 };

    /**
     * Starts the filter at h = 0, v = 0 at time tUs.
     * @param padPressureMbar pressure at the pad (altitude 0), e.g. an average of baro readings
     */
    void begin(float padPressureMbar, uint64_t tUs, const Config& config);
    /** Same, with the default Config. */
    void begin(float padPressureMbar, uint64_t tUs);

    /** Accelerometer sample (vertical specific force, m/s^2, +G at rest) taken at tUs. */
    void accel(uint64_t tUs, float specificForce);

    /** Barometer pressure [mbar] sampled at tUs. */
    ekf::UpdateResult baroPressure(uint64_t tUs, float mbar);
    /** Direct altitude measurement above the pad [m] with 1-sigma `stdM`. */
    ekf::UpdateResult altitude(uint64_t tUs, float meters, float stdM);
    /** Zero-velocity pseudo-measurement (only while the rocket is certainly stationary). */
    ekf::UpdateResult zeroVelocity(uint64_t tUs);

    /** True once launch has been detected (acceleration above Config::launchAccel). Latches. */
    bool inFlight() const { return inFlight_; }
    /** Forces the flight / pad noise level (e.g. from your own launch detection). */
    void setInFlight(bool flying);

    /** Advances the state to tUs without a measurement (e.g. just before reading the outputs). */
    void predictTo(uint64_t tUs) { ekf_.predictTo(model_, tUs); }

    // ---------------- Outputs ----------------
    float altitude() const { return ekf_.x(H); }
    float velocity() const { return ekf_.x(V); }
    float accelBias() const { return ekf_.x(B); }
    /** Latest bias-corrected vertical acceleration (excluding gravity) [m/s^2]. */
    float acceleration() const { return model_.input - ekf_.x(B) - G; }
    float altitudeStd() const { return ekf_.stddev(H); }
    float velocityStd() const { return ekf_.stddev(V); }
    float accelBiasStd() const { return ekf_.stddev(B); }
    uint64_t timeUs() const { return ekf_.timeUs(); }

    /** Pressure the model predicts at altitude h [mbar]. */
    float pressureAt(float h) const;
    /** Altitude the model assigns to pressure p [mbar] (inverse of pressureAt). */
    float altitudeAt(float mbar) const;

    Config& config() { return cfg_; }
    const ekf::AsyncEKF<3>& filter() const { return ekf_; }
    /** Running sum / count of baro NIS (mean should be ~1 when the noise settings are right). */
    float meanBaroNis() const { return baroNisCount_ ? baroNisSum_ / baroNisCount_ : 0.0f; }

private:
    struct Model {
        float input = G;     // latest specific force (zero-order hold)
        float qAccel = 0.25f;
        float qBias = 4e-4f;
        void propagate(ekf::Vec<3>& x, float dt) const;
        void jacobian(const ekf::Vec<3>& x, float dt, ekf::Mat<3, 3>& F) const;
        void processNoise(const ekf::Vec<3>& x, float dt, ekf::Mat<3, 3>& Q) const;
    };

    Config cfg_;
    Model model_;
    ekf::AsyncEKF<3> ekf_;
    float padPressure_ = 1013.25f;
    bool inFlight_ = false;
    float baroNisSum_ = 0.0f;
    uint32_t baroNisCount_ = 0;
};
