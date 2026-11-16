/**
 * sim_settings.h - Settings that only exist in the simulator.
 *
 * Every flight-software setting (state machine thresholds, pad hold, high-g, roll test, log rates,
 * barometer lockout, servo centers, ...) is in sim/config.h, a copy of the firmware's config.h.
 * Edit that file to try different settings; the firmware's own config.h is never touched.
 */
#pragma once

#include <stdint.h>
#include "NavEKF.h"

// The flight's nav CSV holds the IMU averaged over each row (10 Hz on the pad, 300 Hz in flight).
// Each row is split into steps of this length, so the state machine hold timers, the 100 Hz pad
// hold ticks and the roll test schedule run at about the same resolution as on the board.
// 0 = one step per input row.
constexpr uint32_t SIM_STEP_US = 1000;

// Pad air temperature, used for the Mach number (barometer lockout) and qbar. The board measures
// it with the MS5611 during alignment, but that reading isn't logged.
//   true:  recover it from the flight's own mach and speed_mps columns (the value the board used)
//   false: use SIM_PAD_TEMP_C
constexpr bool  SIM_PAD_TEMP_FROM_LOG = true;
constexpr float SIM_PAD_TEMP_C        = 15.0f;

// The magnetometer isn't logged, so by default the sim runs without one: the rocket's heading starts
// at 0 and only follows the gyro. That changes yaw_deg / pitch_deg / tilt_az_deg / north_m / east_m
// (which way the rocket leans), but not tilt, altitude, speed or the flight states.
//   true: feed a synthetic magnetometer made from the flight's logged attitude (qw..qz), so the
//         heading follows what the board estimated. This uses an EKF output, not a raw sensor.
constexpr bool  SIM_MAG_FROM_LOGGED_ATTITUDE = false;
constexpr float SIM_MAG_INCLINATION_DEG      = 60.0f;   // dip of the synthetic field below horizontal

// Print every state change and roll-test change, next to the flight's original ones.
constexpr bool SIM_PRINT_EVENTS = true;

/**
 * EKF tuning. The firmware uses NavEKF::Config's defaults (lib/EKF/NavEKF.h) except for the pad
 * temperature, declination and axes, which the sim fills in from config.h before calling this.
 * Change any field here to try different tuning, for example:
 *     cfg.baroNoiseMbar = 0.2f;
 *     cfg.accelNoiseDensity = 0.5f;
 *     cfg.softGates = false;                       // old behaviour: outliers thrown away
 *     cfg.accelBiasWalk = 0.01f; cfg.gyroBiasWalk = 1e-4f;   // old bias walks
 *     cfg.softGateExponent = 1.0f;                 // de-weight outliers harder
 */
inline void simEkfConfig(NavEKF::Config& cfg)
{
    (void)cfg;
}
