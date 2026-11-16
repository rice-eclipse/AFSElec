/**
 * shared.h - Data passed between the boot, sensor and logger tasks.
 */
#pragma once

#include <stdint.h>
#include "rtos.h"

/** One snapshot of the latest value of every sensor. `fresh` marks which ones are new in this snapshot. */
struct Sample {
    uint64_t tUs;            // time since recording started
    uint32_t seq;            // snapshot number (gaps in the CSV = snapshots dropped by the logger)
    uint8_t  fresh;          // FRESH_* bits
    float imuAx, imuAy, imuAz, imuGx, imuGy, imuGz, imuTemp;
    float kxX, kxY, kxZ;
    float magX, magY, magZ;
    float pressure, baroTemp, altitude;
    float tmp117;
    float busV, currentMa, powerMw;
};

enum : uint8_t {
    FRESH_IMU = 1 << 0, FRESH_KX134 = 1 << 1, FRESH_MAG = 1 << 2, FRESH_BARO = 1 << 3,
    FRESH_TMP = 1 << 4, FRESH_INA = 1 << 5,
};

/** Which sensors started successfully (a failed sensor is skipped and its CSV columns stay empty). */
struct SensorStatus { bool imu, acc, mag, baro, temp, pwr; };

/** Counters filled in by the sensor and logger tasks, printed at the end of the run. */
struct RunStats {
    // sensor task
    uint64_t sensorUs = 0;
    uint32_t loops = 0;
    uint32_t imuNew = 0, accNew = 0, magNew = 0, baroNew = 0, tmpNew = 0, inaNew = 0;
    uint32_t produced = 0, dropped = 0;
    uint32_t maxQueued = 0;
    // logger task
    uint64_t loggerUs = 0;
    uint32_t logged = 0;
    uint32_t writeErrors = 0;
    bool fileOk = false;
};

/** Event bits set in g_events when each recording task (and the servo sweep) has finished. */
enum : uint32_t { EVT_SENSOR_DONE = 1 << 0, EVT_LOGGER_DONE = 1 << 1, EVT_SERVO_DONE = 1 << 2,
                  EVT_SERVO_TUNE_DONE = 1 << 3 };

extern SensorStatus          g_status;        // written by boot, read by the sensor task
extern RunStats              g_stats;
extern rtos::Queue<Sample>   g_samples;       // sensor task -> logger task
extern rtos::EventGroup      g_events;
extern volatile bool         g_sensorsDone;   // sensor loop finished (logger drains, then stops)
extern char                  g_logName[48];   // CSV file name on the SD card

// ============================================================================
// Navigation (after the speed test)
// ============================================================================
/** Navigation phase, logged in every row. */
enum class NavPhase : uint8_t { Aligning = 1, Navigating = 2 };

/** One navigation CSV row: the EKF state at one instant. */
struct NavRow {
    uint64_t tUs;                    // since navigation started
    uint32_t seq;
    uint8_t  phase, inFlight;
    uint8_t  flightState;            // FlightState (flight_state.h)
    uint8_t  baroLocked;             // 1 = barometer ignored (transonic)
    float    mach;                   // EKF speed / speed of sound
    uint8_t  highG;                  // 1 = KX134 is the accelerometer (ICM45686 ignored above HIGHG_SWITCH_G)
    float    highGAccel;             // KX134 |accel|, g
    // Roll test (roll_test.h) and the data to analyse it
    float    p, q, r;                // body rates (IMU axes, bias-corrected), deg/s, averaged since the previous row
    float    rollRate;               // rate about the rocket's nose axis, deg/s, averaged since the previous row
    float    ax, ay, az;             // specific force (IMU axes, bias-corrected), m/s^2, averaged since the previous row
    float    speed;                  // EKF |velocity|, m/s
    float    pressure;               // latest barometer reading, mbar
    float    qbar;                   // dynamic pressure 1/2 rho V^2, Pa (rho from pressure and ISA temperature)
    float    tabDeg;                 // roll-test tabs commanded deflection (same on every tab), deg from center
    uint16_t tabUs;                  // first roll-test tab (PWM1) commanded pulse width, us
    uint8_t  rollTestStatus;         // RollTest::Status: 0 waiting, 1 running, 2 done, 3 aborted
    uint8_t  rollTestDoublet;        // doublet number while running (1, 2, ...)
    float qw, qx, qy, qz;            // attitude quaternion (IMU body -> NED)
    float roll, pitch, yaw;          // deg
    float tilt, tiltAz;              // deg: rocket axis from vertical, and its compass direction
    float north, east, alt;          // m, relative to the start position
    float vn, ve, vd;                // m/s (NED)
    float baroAlt;                   // m: altitude from the barometer alone, for comparison
    float tiltStd, headingStd, altStd, horizStd;   // 1-sigma
    float bax, bay, baz;             // accel bias, m/s^2
    float bgx, bgy, bgz;             // gyro bias, deg/s
    float ekfLoad;                   // % of core 0 spent inside the EKF since the previous row
    uint32_t ekfMaxUs;               // longest single EKF call since the previous row
    uint32_t imuN, baroN, magN;      // samples fed to the filter so far
    uint32_t dropped;                // rows dropped so far (logger behind)
};

extern rtos::Queue<NavRow>*   g_navRows;      // nav task -> logger (created when navigation starts)
extern volatile bool          g_navFailed;    // the filter could not start (no IMU data)
