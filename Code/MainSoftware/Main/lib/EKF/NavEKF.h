/**
 * NavEKF.h - Asynchronous 15-state error-state EKF (strapdown INS) for full rocket navigation:
 *            3D orientation, position relative to the pad, and altitude.
 *
 * World frame: NED (north, east, down) with its origin at the pad. Altitude = -down.
 * Body frame:  the IMU's own axes. Feed accel, gyro and magnetometer in this same frame (rotate the
 *              magnetometer's axes into the IMU's axes before calling magnetometer()).
 *
 * Nominal state (integrated at the full IMU rate):
 *     position p (NED, m), velocity v (NED, m/s), attitude q (body -> NED),
 *     accelerometer bias ba (body, m/s^2), gyro bias bg (body, rad/s)
 * Error state in the EKF (15): [dp(3) dv(3) dtheta(3) dba(3) dbg(3)]. dtheta is a small rotation in
 * the body frame (q_true = q * exp(dtheta)). After every update the error is folded back into the
 * nominal state and reset to zero (the standard "error-state" or "multiplicative" EKF).
 *
 * Inputs and measurements, each fused whenever it arrives:
 *   imu(t, accel, gyro)   accel [m/s^2] (+9.81 up at rest) and gyro [rad/s]. Drives the prediction.
 *   baroPressure(t, p)    static pressure [mbar], nonlinear ISA model -> altitude
 *   magnetometer(t, m)    magnetic field (any unit) -> heading and tilt (direction only)
 *   stationary(t)         "on the pad, not moving": zero velocity + zero rotation rate
 *                         (aligns tilt, learns gyro and vertical accel bias)
 *   position(t, n, e, up, std)  optional external fix (e.g. GPS, converted to local NED)
 *
 *   static NavEKF nav;                          // ~2.5 kB, keep it out of small stacks
 *   nav.begin(padMbar, accelAvg, magAvg, time_us_64());   // averaged on the pad, at rest
 *   // IMU loop:   nav.imu(tUs, accel_g * NavEKF::G, gyro_dps * NavEKF::DEG2RAD);
 *   // baro ready: nav.baroPressure(tUs, mbar);
 *   // mag ready:  nav.magnetometer(tUs, field);
 *   // pad, 100 Hz before arming: nav.stationary(tUs);
 *   nav.roll(); nav.pitch(); nav.yaw(); nav.tilt(); nav.north(); nav.east(); nav.altitude();
 *
 * Cost: the nominal state is integrated on every IMU sample (a few hundred flops). The 15x15
 * covariance is propagated every covarianceStepS (default 5 ms = 200 Hz, ~7 k multiply-adds) and
 * updates use the short covariance form (N^2 M), so the whole filter fits at full sensor rates.
 *
 * Stack: the 15x15 matrix temporaries need up to ~9.5 kB (stationary() -> 6-row update; a
 * covariance step needs ~6.5 kB). Call it from a task with >= 16 kB (4096 words) of stack.
 */
#pragma once

#include <stdint.h>
#include "AsyncEKF.h"
#include "EKFRotation.h"

class NavEKF {
public:
    static constexpr float G = 9.80665f;
    static constexpr float DEG2RAD = 0.017453292519943f;
    static constexpr float RAD2DEG = 57.29577951308232f;

    struct Config {
        // ---- Process noise ----
        /** Accelerometer noise density in flight [(m/s^2)/sqrt(Hz)]. Covers vibration, scale and misalignment errors. */
        float accelNoiseDensity = 0.3f;
        /** Accelerometer noise density on the pad (before launch is detected). */
        float padAccelNoiseDensity = 0.01f;
        /** Gyro noise density [(rad/s)/sqrt(Hz)]. */
        float gyroNoiseDensity = 0.002f;
        /** Bias random walks [(m/s^2)/sqrt(s)] and [(rad/s)/sqrt(s)]. */
        float accelBiasWalk = 0.01f;
        float gyroBiasWalk = 1e-4f;
        /** Launch is detected when the upward acceleration (excluding gravity) exceeds this [m/s^2]. */
        float launchAccel = 2.0f * 9.80665f;

        // ---- Barometer ----
        float baroNoiseMbar = 0.1f;        // 1 sigma [mbar]
        float baroGate = 16.0f;            // NIS gate (~4 sigma); ekf::GATE_OFF disables
        float baroLockoutSpeed = 0.0f;     // inflate baro noise above this |vertical speed| [m/s]; 0 = never
        float baroLockoutScale = 100.0f;
        float padTemperatureK = 288.15f;   // for the barometric formula
        /**
         * The baro corrects only altitude, vertical speed and the accel bias (not attitude or
         * horizontal states). Otherwise unmodeled vertical errors (accel scale factor, transonic
         * pressure) get "explained" by rotating the attitude, which also wrecks the horizontal
         * position. Leave it on unless you know your vertical channel is clean.
         */
        bool baroVerticalOnly = true;

        // ---- Magnetometer ----
        /** Noise of the normalized field direction, 1 sigma (0.02 ~ 1.1 deg). */
        float magNoise = 0.02f;
        float magGate = ekf::GATE_3DOF_999;
        /** Reject readings whose strength differs from the pad reference by more than this fraction. */
        float magStrengthTolerance = 0.3f;
        /** Magnetic declination [deg, east positive]. 0 = yaw is relative to magnetic north. */
        float magDeclinationDeg = 0.0f;

        // ---- Pad (stationary) pseudo-measurements ----
        float zeroVelocityStd = 0.02f;     // [m/s]
        float zeroRateStd = 0.002f;        // [rad/s], residual pad motion + gyro noise

        // ---- Geometry / timing ----
        /** The rocket's long axis (nose direction) in the body frame. Used by tilt() and launch logic. */
        float rocketAxis[3] = {1.0f, 0.0f, 0.0f};
        /** A body axis perpendicular to rocketAxis that sets rocket roll 0 (see rocketAngles()).
         *  Its component along rocketAxis is removed, so it only has to be roughly perpendicular. */
        float rocketRollRefAxis[3] = {0.0f, 1.0f, 0.0f};
        /** Measurements up to this late (behind the IMU time) are still fused [us]. */
        uint32_t maxLagUs = 50'000;
        /** Covariance propagation interval [s]. */
        float covarianceStepS = 0.005f;

        // ---- Initial uncertainty (1 sigma) ----
        float initialPositionStd = 0.5f;           // [m]
        float initialVelocityStd = 0.1f;           // [m/s]
        float initialTiltStdDeg = 2.0f;
        float initialHeadingStdDeg = 10.0f;        // 180 if there is no magnetometer
        float initialAccelBiasStd = 0.3f;          // [m/s^2]
        float initialGyroBiasStd = 0.02f;          // [rad/s]
    };

    enum : int {   // error-state indices
        PN = 0, PE, PD, VN, VE, VD, TX, TY, TZ, BAX, BAY, BAZ, BGX, BGY, BGZ, NSTATES
    };

    /**
     * Initializes at the pad (position 0, velocity 0) at time tUs. The attitude comes from gravity
     * (tilt) and the magnetometer (heading), so call it with the rocket at rest.
     * @param accelBody average accelerometer reading at rest [m/s^2]
     * @param magBody   average magnetometer reading (any unit), or zeros if there is no
     *                  magnetometer (yaw then starts at 0 and only drifts with the gyro)
     * @return false if the accelerometer reading is unusable (zero / not finite)
     */
    bool begin(float padPressureMbar, const ekf::Vec3& accelBody, const ekf::Vec3& magBody, uint64_t tUs,
               const Config& config);
    bool begin(float padPressureMbar, const ekf::Vec3& accelBody, const ekf::Vec3& magBody, uint64_t tUs);

    /** IMU sample taken at tUs: accel [m/s^2] and gyro [rad/s], body frame. */
    void imu(uint64_t tUs, const ekf::Vec3& accel, const ekf::Vec3& gyro);

    /** Barometer pressure [mbar] sampled at tUs. */
    ekf::UpdateResult baroPressure(uint64_t tUs, float mbar);
    /**
     * Magnetometer reading in the body frame (any unit; only the direction is fused).
     * Before launch it corrects heading only, so tilt comes from gravity alone. At launch the
     * field reference is re-anchored to the final pad attitude, and in flight the full 3D
     * direction is fused (heading and tilt about the axes perpendicular to the field).
     */
    ekf::UpdateResult magnetometer(uint64_t tUs, const ekf::Vec3& field);
    /** Zero-velocity + zero-rotation pseudo-measurement. Only while certainly at rest on the pad. */
    ekf::UpdateResult stationary(uint64_t tUs);
    /** External position fix relative to the pad (north, east, up) [m] with 1-sigma stdM per axis. */
    ekf::UpdateResult position(uint64_t tUs, float north, float east, float up, float stdM);
    /** Horizontal-only position fix (north, east) [m] with 1-sigma stdM per axis (e.g. "still on the pad"). */
    ekf::UpdateResult horizontalPosition(uint64_t tUs, float north, float east, float stdM);

    /**
     * Moves the barometric altitude reference: altitude 0 is now where the pressure is `mbar`.
     * Use it on the pad to follow slow weather pressure changes, so the altitude stays near 0.
     */
    void setPadPressure(float mbar);
    float padPressure() const { return padPressure_; }

    /** Drops the gyro samples collected for the next stationary() (call when the vehicle was moving). */
    void clearStationaryWindow() { gyroSum_ = ekf::Vec3::zeros(); gyroCount_ = 0; }

    /** True once launch has been detected. Latches. */
    bool inFlight() const { return inFlight_; }
    /**
     * Forces flight / pad mode (e.g. from your own launch detection). Flight mode uses the flight
     * accel noise and 3D magnetometer fusion; entering it re-anchors the magnetic reference.
     */
    void setInFlight(bool flying);

    /** Advances the state to tUs using the last IMU sample (e.g. just before reading the outputs). */
    void predictTo(uint64_t tUs);

    // ---------------- Orientation ----------------
    const ekf::Quat& quaternion() const { return q_; }   // body -> NED
    ekf::Mat3 rotation() const { return q_.toMatrix(); }
    /** Aerospace ZYX Euler angles [deg]. Pitch near +-90 deg (rocket vertical) makes roll/yaw ill-defined: use tilt(). */
    float roll() const;
    float pitch() const;
    float yaw() const;    // heading, 0 = north (true north if magDeclinationDeg is set), 90 = east
    /**
     * Euler angles of the rocket (not the IMU) [deg], made for a rocket that flies near vertical:
     * all three are 0 when the nose points straight up and the roll reference axis faces north.
     *   yaw   nose tilt toward north (rotation about the west axis)
     *   pitch nose tilt toward east
     *   roll  spin about the nose, right-handed (clockwise when seen from the tail looking toward the nose)
     * ZYX order relative to the frame {up, north, west}; only singular when the nose is horizontal
     * and points east or west (pitch = +-90), never near vertical.
     */
    struct RocketAngles { float roll, pitch, yaw; };
    RocketAngles rocketAngles() const;
    /** Angle between the rocket axis and straight up [deg]. 0 = vertical. */
    float tilt() const;
    /** Compass direction the rocket axis leans towards [deg], 0 = north. Undefined when tilt() ~ 0. */
    float tiltAzimuth() const;
    /** Rocket axis in NED (unit vector). */
    ekf::Vec3 axisNED() const;
    /** 1-sigma tilt and heading uncertainty [deg]. */
    float tiltStd() const;
    float headingStd() const;

    // ---------------- Position / velocity ----------------
    float north() const { return p_[0]; }
    float east() const { return p_[1]; }
    float altitude() const { return -p_[2]; }
    /** Horizontal distance from the pad [m]. */
    float range() const { return sqrtf(p_[0] * p_[0] + p_[1] * p_[1]); }
    const ekf::Vec3& positionNED() const { return p_; }
    const ekf::Vec3& velocityNED() const { return v_; }
    float verticalSpeed() const { return -v_[2]; }   // up positive
    float speed() const { return ekf::norm(v_); }
    /** Latest bias-corrected acceleration (excluding gravity) in NED [m/s^2]. */
    ekf::Vec3 accelerationNED() const;
    float altitudeStd() const { return ekf_.stddev(PD); }
    float horizontalStd() const { return sqrtf(ekf_.P()(PN, PN) + ekf_.P()(PE, PE)); }
    float verticalSpeedStd() const { return ekf_.stddev(VD); }

    // ---------------- Biases ----------------
    const ekf::Vec3& accelBias() const { return ba_; }   // body [m/s^2]
    const ekf::Vec3& gyroBias() const { return bg_; }    // body [rad/s]

    // ---------------- Misc ----------------
    uint64_t timeUs() const { return tUs_; }
    Config& config() { return cfg_; }
    const ekf::AsyncEKF<NSTATES>& filter() const { return ekf_; }
    uint32_t staleUpdates() const { return stale_; }
    float meanBaroNis() const { return baroNisN_ ? baroNisSum_ / baroNisN_ : 0.0f; }
    float meanMagNis() const { return magNisN_ ? magNisSum_ / magNisN_ : 0.0f; }   // ideal ~3
    /** Pressure the model predicts at altitude h [mbar], and its inverse. */
    float pressureAt(float h) const;
    float altitudeAt(float mbar) const;

private:
    /** Covariance-only process model for AsyncEKF (the nominal state is integrated separately). */
    struct CovModel {
        const NavEKF* nav = nullptr;
        void propagate(ekf::Vec<NSTATES>&, float) const {}   // error mean stays 0
        void jacobian(const ekf::Vec<NSTATES>&, float dt, ekf::Mat<NSTATES, NSTATES>& F) const;
        void processNoise(const ekf::Vec<NSTATES>&, float dt, ekf::Mat<NSTATES, NSTATES>& Q) const;
    };

    void integrate(float dt);          // nominal strapdown step with the held IMU sample
    void advanceTo(uint64_t tUs);      // integrate the held sample up to tUs
    void flushCovariance();
    /**
     * Brings the state to a measurement time. Returns false if stale. lagS is how far the
     * measurement is behind the state (0 if it was predicted forward to it); the measurement models
     * use it to predict what the sensor saw back then (h - v*lag, attitude rotated back by w*lag).
     */
    bool sync(uint64_t tUs, float& lagS);
    void injectError();                // fold the EKF error state into the nominal state
    ekf::UpdateResult padHeading(const ekf::Vec3& field);   // heading-only mag update (pad)
    void accumulateNis(const ekf::UpdateResult& r, float& sum, uint32_t& n);

    Config cfg_;
    ekf::AsyncEKF<NSTATES> ekf_;
    CovModel model_;

    // Nominal state
    ekf::Vec3 p_ = ekf::Vec3::zeros(), v_ = ekf::Vec3::zeros();
    ekf::Quat q_;
    ekf::Vec3 ba_ = ekf::Vec3::zeros(), bg_ = ekf::Vec3::zeros();
    uint64_t tUs_ = 0;

    // Held IMU sample (zero-order hold until the next one)
    ekf::Vec3 accel_ = ekf::Vec3::zeros(), gyro_ = ekf::Vec3::zeros();
    bool haveImu_ = false;

    // Covariance accumulators since the last covariance step (body frame, bias-corrected)
    float covDt_ = 0.0f;
    ekf::Vec3 covDv_ = ekf::Vec3::zeros(), covDa_ = ekf::Vec3::zeros();

    // Gyro average for stationary()
    ekf::Vec3 gyroSum_ = ekf::Vec3::zeros();
    uint32_t gyroCount_ = 0;

    // Magnetometer reference (unit vector in NED, and field strength)
    ekf::Vec3 magRef_ = ekf::Vec3::zeros();
    float magStrength_ = 0.0f;
    bool magEnabled_ = false;
    // Running average of the body-frame field on the pad; re-anchors magRef_ at launch so the
    // in-flight reference agrees with the gravity-aligned pad attitude.
    ekf::Vec3 magAvg_ = ekf::Vec3::zeros();
    uint32_t magCount_ = 0;

    ekf::Vec3 axis_ = ekf::Vec3::zeros();
    ekf::Vec3 rollRef_ = ekf::Vec3::zeros();   // unit, perpendicular to axis_
    float padPressure_ = 1013.25f;
    bool inFlight_ = false;
    uint32_t stale_ = 0;
    float baroNisSum_ = 0.0f, magNisSum_ = 0.0f;
    uint32_t baroNisN_ = 0, magNisN_ = 0;
};
