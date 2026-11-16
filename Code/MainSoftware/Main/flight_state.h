/**
 * flight_state.h - Rocket flight state machine with event detection.
 *
 *   LAUNCHPAD --launch--> POWERED_ASCENT --burnout--> CRUISING --apogee--> DESCENT --touchdown--> LANDED
 *
 * Detection (all thresholds and hold times are in config.h, "Flight state machine"):
 *   launch:    |specific force| > LAUNCH_ACCEL_G for LAUNCH_HOLD_MS
 *              (backup: altitude > LAUNCH_BACKUP_ALT_M and climbing > LAUNCH_BACKUP_VS_MPS)
 *   burnout:   thrust gone: axial specific force < BURNOUT_AXIAL_G for BURNOUT_HOLD_MS,
 *              no sooner than MIN_BURN_MS after launch
 *   apogee:    EKF vertical speed < 0 for APOGEE_HOLD_MS, or altitude APOGEE_ALT_DROP_M below the
 *              highest altitude reached (backup), no sooner than MIN_COAST_MS after burnout
 *   touchdown: |vertical speed| < LANDED_VS_MPS, |specific force| within LANDED_ACCEL_TOL_G of 1 g,
 *              rotation < LANDED_GYRO_DPS and altitude steady within LANDED_ALT_BAND_M, all for
 *              LANDED_HOLD_MS
 *
 * Altitude and vertical speed come from the NavEKF, which ignores the barometer while the rocket
 * is transonic (see machNumber() and navFeed() in loop.cpp).
 *
 * Plain C++ with no Pico SDK dependency: feed it timestamps in microseconds.
 */
#pragma once

#include <stdint.h>

enum class FlightState : uint8_t {
    Launchpad     = 0,
    PoweredAscent = 1,
    Cruising      = 2,   // coasting after burnout, still going up
    Descent       = 3,
    Landed        = 4,
};

/** "LAUNCHPAD", "POWERED_ASCENT", ... */
const char* flightStateName(FlightState s);

/** What the state machine looks at, once per IMU sample. */
struct FlightInputs {
    float accelG;      // |specific force| (bias-corrected accelerometer), g
    float axialG;      // specific force along the rocket's nose axis, g (thrust > 0, drag < 0)
    float gyroDps;     // |angular rate| (bias-corrected), deg/s
    float altM;        // EKF altitude above the pad, m
    float vertSpeed;   // EKF vertical speed, m/s, up positive
};

/** Mach number from speed (m/s), altitude above the pad (m) and pad air temperature (K). */
float machNumber(float speedMps, float altM, float padTempK);

class FlightStateMachine {
public:
    FlightState state() const { return state_; }

    /** Runs the detectors. Returns true if the state changed on this call. */
    bool update(uint64_t tUs, const FlightInputs& in);

    /** Moves to the next state by hand (e.g. a serial command for bench tests). False if already LANDED. */
    bool advance(uint64_t tUs);

    /** Time the current state was entered (0 = power-up). */
    uint64_t stateSinceUs() const { return stateSinceUs_; }

    /** Highest EKF altitude since launch, m. */
    float maxAltitude() const { return maxAlt_; }

private:
    /** True once `cond` has been true continuously for holdMs. */
    struct Hold {
        bool active = false;
        uint64_t sinceUs = 0;
        bool update(bool cond, uint64_t tUs, uint32_t holdMs);
        void reset() { active = false; }
    };

    void enter(FlightState s, uint64_t tUs);

    FlightState state_ = FlightState::Launchpad;
    uint64_t stateSinceUs_ = 0;
    float maxAlt_ = 0.0f;
    Hold launch_, launchBackup_, burnout_, apogee_, landed_;
    float landedAltMin_ = 0.0f, landedAltMax_ = 0.0f;   // altitude band while the landed hold runs
};
