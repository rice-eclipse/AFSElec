/**
 * config.h - Every tunable setting for the flight computer firmware in one place.
 */
#pragma once

#include <stdint.h>
#include "FreeRTOS.h"

// ============================================================================
// Startup bench tests: set each one to true/false to run it at boot or skip it
// ============================================================================
constexpr bool RUN_ICM45686_TEST  = true;
constexpr bool RUN_KX134_TEST     = true;
constexpr bool RUN_MMC5983MA_TEST = true;
constexpr bool RUN_MS5611_TEST    = true;
constexpr bool RUN_TMP117_TEST    = true;
constexpr bool RUN_INA219_TEST    = true;
constexpr bool RUN_SDCARD_TEST    = true;    // non-destructive: works in a _sdtest folder, then deletes it
constexpr bool RUN_FREERTOS_TEST  = false;

// Verbose: true = print every PASS line and the sample readings,
//          false = print only FAIL lines, diagnostics and each test's one-line result
constexpr bool VERBOSE_ICM45686_TEST  = false;
constexpr bool VERBOSE_KX134_TEST     = false;
constexpr bool VERBOSE_MMC5983MA_TEST = false;
constexpr bool VERBOSE_MS5611_TEST    = false;
constexpr bool VERBOSE_TMP117_TEST    = false;
constexpr bool VERBOSE_INA219_TEST    = false;
constexpr bool VERBOSE_SDCARD_TEST    = false;
constexpr bool VERBOSE_FREERTOS_TEST  = false;

// ============================================================================
// Sensors
// ============================================================================
// INA219 shunt resistor and maximum expected current
constexpr float INA219_SHUNT_OHMS = 0.002f;
constexpr float INA219_MAX_AMPS   = 3.2f;

// SPI clocks for the speed test (datasheet maximums; lower them if reads start failing)
constexpr uint32_t IMU_SPI_HZ  = 24'000'000;   // ICM45686:  24 MHz max
constexpr uint32_t ACC_SPI_HZ  = 10'000'000;   // KX134:     10 MHz max
constexpr uint32_t MAG_SPI_HZ  = 10'000'000;   // MMC5983MA: 10 MHz max
constexpr uint32_t BARO_SPI_HZ = 20'000'000;   // MS5611:    20 MHz max

// The I2C sensors are slow to poll (~100 us per transaction at 400 kHz), so they are only checked
// this often, to leave the loop free for the fast SPI sensors
constexpr uint32_t INA219_POLL_US  = 500;    // INA219 converts every ~1.06 ms (12-bit shunt + bus)
constexpr uint32_t TMP117_POLL_US  = 4000;   // TMP117 converts every 15.5 ms (no averaging)
constexpr uint8_t  BARO_TEMP_EVERY = 10;     // MS5611: one temperature conversion per 10 pressure ones

// ============================================================================
// Recording
// ============================================================================
constexpr bool     RUN_SPEED_TEST     = false;   // false = skip the max-rate test and go straight to navigation
constexpr uint32_t RECORD_DURATION_MS = 10'000;  // sensor loop runs this long, then stops
constexpr uint32_t SAMPLE_QUEUE_LEN   = 512;     // snapshots buffered between the sensor and logger tasks

// Each run gets a new file (flights/flight_000.csv, flight_001.csv, ...), so nothing is overwritten
constexpr const char* LOG_PREFIX = "flights/flight_";
constexpr const char* LOG_SUFFIX = ".csv";

// ============================================================================
// Servos (PWM1-PWM8): swept +-SERVO_SWEEP_DEG around each center before navigation starts
// ============================================================================
constexpr bool     RUN_SERVO_SWEEP     = true;
constexpr bool     RUN_SERVO_TUNE      = false;    // serial servo tuning console after the sweep (waits for "Done.")
constexpr uint32_t SERVO_TUNE_IDLE_MS  = 300;     // a typed line also ends after this long with no characters
constexpr uint32_t SERVO_SWEEP_MS      = 180'000;  // total sweep time
constexpr float    SERVO_SWEEP_DEG     = 10.0f;   // sweep is center - this .. center + this
constexpr uint32_t SERVO_SWEEP_HALF_MS = 1000;    // time to go from one end of the sweep to the other
constexpr uint32_t SERVO_UPDATE_MS     = 20;      // pulse width update period (one 50 Hz frame)
constexpr uint32_t SERVO_PWM_HZ        = 50;

// Servo power during navigation: when navigation starts every servo is driven to its center for
// SERVO_PAD_CENTER_MS, then released (PWM pulses stopped, so it stops drawing holding current) for
// the time on the pad. At launch every servo is driven to center again and held there for the
// whole flight (the roll test moves ROLL_TEST_SERVOS during coast). After landing they are
// released again if SERVO_RELEASE_AFTER_LANDING.
constexpr bool     SERVO_RELEASE_ON_PAD        = true;
constexpr uint32_t SERVO_PAD_CENTER_MS         = 2000;   // time to reach center before releasing
constexpr bool     SERVO_RELEASE_AFTER_LANDING = true;

// Pulse width per degree: 1100-1900 us = 90 deg on our servos.
constexpr float    SERVO_US_PER_DEG    = 800.0f / 90.0f;

// Absolute limits: no servo is ever commanded outside these, whatever its center
constexpr uint32_t SERVO_MIN_US        = 1000;
constexpr uint32_t SERVO_MAX_US        = 2000;

// Each servo's own limits: its center +- SERVO_LIMIT_DEG (and inside the absolute limits above)
constexpr float    SERVO_LIMIT_DEG     = 30.0f;

// Center (neutral) pulse width of each servo, PWM1..PWM8 (PWM1-4 trimmed; PWM5-8 unused, left at 1500)
constexpr int      SERVO_COUNT = 8;
constexpr uint32_t SERVO_CENTER_US[SERVO_COUNT] = {
    1490,   // PWM1
    1535,   // PWM2
    1445,   // PWM3
    1510,   // PWM4
    1500,   // PWM5
    1500,   // PWM6
    1500,   // PWM7
    1500,   // PWM8
};

// Per-servo limits from the above (center +- SERVO_LIMIT_DEG, clipped to SERVO_MIN_US..SERVO_MAX_US)
constexpr uint32_t SERVO_LIMIT_US = (uint32_t)(SERVO_LIMIT_DEG * SERVO_US_PER_DEG + 0.5f);
constexpr uint32_t servoMinUs(int i) {
    return SERVO_CENTER_US[i] > SERVO_MIN_US + SERVO_LIMIT_US ? SERVO_CENTER_US[i] - SERVO_LIMIT_US : SERVO_MIN_US;
}
constexpr uint32_t servoMaxUs(int i) {
    return SERVO_CENTER_US[i] + SERVO_LIMIT_US < SERVO_MAX_US ? SERVO_CENTER_US[i] + SERVO_LIMIT_US : SERVO_MAX_US;
}

// ============================================================================
// Indicator LEDs (GPIO 0 blue, 1 red, 2 yellow, 3 green): all off by default
//   green = every enabled boot test passed and every sensor started
//   blue  = nav lock: navigating, with |pitch| and |yaw| both < NAV_LOCK_DEG
// ============================================================================
constexpr int      LED_COUNT       = 4;
constexpr bool     LED_ACTIVE_HIGH = true;   // true = GPIO high turns the LED on
constexpr float    NAV_LOCK_DEG    = 10.0f;

// ============================================================================
// Navigation (runs forever, after the speed test if it is on)
// ============================================================================
constexpr bool RUN_EKF_TEST     = false;    // simulated-flight EKF bench test at boot (~1 s)
constexpr bool VERBOSE_EKF_TEST = false;

// Alignment: the board must be still. Sensors are averaged for NAV_ALIGN_AVG_MS, the filter starts
// (tilt from gravity, heading from the magnetometer), then stationary() updates run until
// NAV_ALIGN_MS to learn the gyro biases. Navigation then runs forever.
constexpr uint32_t NAV_ALIGN_AVG_MS  = 500;
constexpr uint32_t NAV_ALIGN_MS      = 5000;
constexpr uint32_t NAV_STATIONARY_HZ = 100;

// true:  switch to flight mode as soon as alignment ends (flight process noise, 3D magnetometer).
//        Use this for bench tests where the board is moved by hand.
// false: stay in pad mode until NavEKF detects launch (> 2 g upward). Use this for real flights.
constexpr bool NAV_FLIGHT_MODE_AFTER_ALIGN = false;

// Pad hold: after alignment, while the flight state is LAUNCHPAD, the filter is told the rocket is
// not moving (zero velocity, zero rotation, horizontal position = pad), so small sensor offsets and
// noise don't make it drift. Each NAV_STATIONARY_HZ tick is only applied if the IMU really was
// still since the last one (rotation and |accel| - 1 g below the limits), so handling the rocket
// isn't absorbed as gyro bias. The altitude reference also follows slow weather pressure changes,
// so the altitude stays near 0 on the pad.
constexpr bool  PAD_HOLD              = true;
constexpr float PAD_STILL_GYRO_DPS    = 3.0f;    // max rotation rate counted as still
constexpr float PAD_STILL_ACCEL_G     = 0.05f;   // max | |accel| - 1 g | counted as still
constexpr float PAD_HOLD_POS_STD_M    = 0.05f;   // how tightly the horizontal position is held at 0
constexpr float PAD_PRESSURE_TAU_S    = 20.0f;   // time constant of the pad pressure (altitude zero) follower
// Pad hold also holds the rocket's pitch and yaw (logged pitch_deg / yaw_deg, i.e. which way the nose
// leans) at these angles; roll is left free. They must match how the rocket really stands: on a
// tilted rail set them to the rail's lean (yaw = toward north, pitch = toward east), otherwise the
// filter starts the flight with that much tilt error. PAD_HOLD_TILT_STD_DEG = how tightly.
constexpr bool  PAD_HOLD_TILT         = true;
constexpr float PAD_HOLD_PITCH_DEG    = 0.0f;
constexpr float PAD_HOLD_YAW_DEG      = 0.0f;
constexpr float PAD_HOLD_TILT_STD_DEG = 0.5f;

// After apogee (DESCENT and LANDED), nothing measures horizontal motion, so any tilt error makes
// gravity leak sideways and the horizontal velocity grows without limit. Two fixes:
//   Leveling: whenever the IMU is still (or in a steady descent) the accelerometer only measures
//             gravity, so its direction is fed to the EKF as "this way is up" (corrects tilt, so the
//             leak stops). Checked NAV_STATIONARY_HZ times a second with its own stillness limits.
//   Velocity prior: a weak "horizontal velocity is about 0" once a second, loose in descent (wind
//             drift is real) and tighter after landing. Pulls built-up velocity back to 0.
constexpr bool     LEVEL_WHEN_STILL      = true;
constexpr float    LEVEL_STILL_GYRO_DPS  = 5.0f;    // max rotation rate counted as still
constexpr float    LEVEL_STILL_ACCEL_G   = 0.05f;   // max | |accel| - 1 g | counted as still
constexpr float    LEVEL_STD_DEG         = 2.0f;    // how much each leveling update is trusted
constexpr bool     HVEL_PRIOR            = true;
constexpr float    HVEL_DESCENT_STD_MPS  = 10.0f;   // 1-sigma in DESCENT (covers wind drift)
constexpr float    HVEL_LANDED_STD_MPS   = 1.0f;    // 1-sigma in LANDED
constexpr uint32_t HVEL_PRIOR_HZ         = 1;

// Log rate (the filter always runs at the full IMU rate, and every sensor stays at its maximum rate):
//   LAUNCHPAD, LANDED and alignment: NAV_LOG_IDLE_HZ
//   POWERED_ASCENT, CRUISING, DESCENT: NAV_LOG_FLIGHT_HZ (raise it until the "dropped" count starts
//   growing in flight-rate bench tests; the SD card and CSV formatting on core 1 set the limit)
// A row is also logged the moment the state changes.
constexpr uint32_t    NAV_LOG_IDLE_HZ      = 10;
constexpr uint32_t    NAV_LOG_FLIGHT_HZ    = 300;
constexpr uint32_t    NAV_QUEUE_LEN        = 128;             // rows buffered for the logger (0.43 s at 300 Hz)
constexpr uint32_t    NAV_STATUS_PERIOD_MS = 10'000;          // one status line on the serial port this often (0 = never)
constexpr const char* NAV_LOG_PREFIX       = "flights/nav_";  // flights/nav_000.csv, nav_001.csv, ...

// ============================================================================
// Flight state machine (flight_state.h): LAUNCHPAD -> POWERED_ASCENT -> CRUISING -> DESCENT -> LANDED
// Runs on every IMU sample once alignment is done. "Hold" = condition must stay true this long.
// ============================================================================
constexpr float    LAUNCH_ACCEL_G        = 3.0f;    // launch: |specific force| above this ...
constexpr uint32_t LAUNCH_HOLD_MS        = 50;      // ... for this long
constexpr float    LAUNCH_BACKUP_ALT_M   = 30.0f;   // backup launch: above this altitude ...
constexpr float    LAUNCH_BACKUP_VS_MPS  = 15.0f;   // ... and climbing faster than this ...
constexpr uint32_t LAUNCH_BACKUP_HOLD_MS = 250;     // ... for this long
constexpr float    BURNOUT_AXIAL_G       = 0.5f;    // burnout: thrust along the nose below this ...
constexpr uint32_t BURNOUT_HOLD_MS       = 100;     // ... for this long
constexpr uint32_t MIN_BURN_MS           = 300;     // no burnout earlier than this after launch
constexpr uint32_t APOGEE_HOLD_MS        = 300;     // apogee: EKF vertical speed < 0 for this long ...
constexpr float    APOGEE_ALT_DROP_M     = 10.0f;   // ... or altitude this far below the maximum
constexpr uint32_t MIN_COAST_MS          = 1000;    // no apogee earlier than this after burnout
// Landed = (settled) OR (impact):
//   settled: |vertical speed|, |specific force| - 1 g and rotation all below their limits, held for LANDED_HOLD_MS
//   impact:  |specific force| above LANDED_IMPACT_G for LANDED_IMPACT_HOLD_MS, no sooner than
//            LANDED_IMPACT_AFTER_MS after apogee (so the parachute opening shock doesn't count).
//            With a dual-deploy (main opened low), make LANDED_IMPACT_AFTER_MS longer than the time
//            from apogee to main deployment.
constexpr float    LANDED_VS_MPS         = 1.0f;    // settled: |vertical speed| below this,
constexpr float    LANDED_ACCEL_TOL_G    = 0.2f;    //   |specific force| within 1 g +- this,
constexpr float    LANDED_GYRO_DPS       = 20.0f;   //   rotation below this,
constexpr uint32_t LANDED_HOLD_MS        = 3000;    //   all for this long
constexpr float    LANDED_IMPACT_G       = 4.0f;    // impact: |specific force| above this ...
constexpr uint32_t LANDED_IMPACT_HOLD_MS = 5;       //   ... for this long ...
constexpr uint32_t LANDED_IMPACT_AFTER_MS = 20'000; //   ... at least this long after apogee (DESCENT)

// High-g takeover: the ICM45686 accelerometer clips at +-16 g, so once the KX134 (+-64 g) reads
// more than HIGHG_SWITCH_G (|accel|), the ICM45686 accelerometer is ignored and the KX134 feeds the
// EKF and the state machine until it drops below HIGHG_RETURN_G. The gyro always comes from the
// ICM45686. On the pad both are averaged together: the KX134 gets the ICM45686's offset, and if
// they differ by more than HIGHG_PAD_MAX_DIFF_G the takeover is disabled (axis mapping wrong).
constexpr bool  HIGHG_ENABLE         = true;
constexpr float HIGHG_SWITCH_G       = 15.0f;
constexpr float HIGHG_RETURN_G       = 13.0f;
constexpr float HIGHG_PAD_MAX_DIFF_G = 0.25f;
// KX134 axes -> IMU (ICM45686) axes, like MAG_AXIS / MAG_SIGN below:  imu_i = HIGHG_SIGN[i] * kx[HIGHG_AXIS[i]]
//   KX134:    +x toward the nose (top of the board), +y left, +z out of the board
//   ICM45686: +x right, +y toward the nose, +z out of the board
//   imu_x = -kx_y, imu_y = kx_x, imu_z = kx_z   (both right-handed)
constexpr int   HIGHG_AXIS[3] = {1, 0, 2};
constexpr float HIGHG_SIGN[3] = {-1.0f, 1.0f, 1.0f};

// Serial state command (bench testing): during navigation, sending "state" on the serial port moves
// the flight state machine to the next state (LAUNCHPAD -> POWERED_ASCENT -> CRUISING -> DESCENT ->
// LANDED). Ignored while aligning. Turn both OFF for flight.
//   NAV_SERIAL_STATE_MANUAL = true:  the automatic detectors are paused, so the state only changes
//                                    on "state" (otherwise e.g. a still board in CRUISING "reaches
//                                    apogee" ~1.3 s later)
//   NAV_SERIAL_STATE_MANUAL = false: "state" and the automatic detectors both work
constexpr bool NAV_SERIAL_STATE_CMD    = false;
constexpr bool NAV_SERIAL_STATE_MANUAL = false;

// Roll test (roll_test.h): open-loop doublets on the fin tabs during coast, to measure roll
// acceleration vs. tab deflection at different speeds. Every servo in ROLL_TEST_SERVOS gets the
// same angle (the same angle on each tab rolls the same way); any other servo stays centered.
// Timeline with the defaults (116 m/s at burnout, ~7.7 s coast): starts 0.3 s after burnout, one
// doublet per second (+amp 0.3 s, -amp 0.3 s, center 0.4 s), amp 5/10/15 deg in turn, ends when
// the climb rate drops below 15 m/s (about 1.5 s before apogee) -> about 6 doublets, 2 per amp.
constexpr bool     ROLL_TEST_ENABLE      = true;
constexpr int      ROLL_TEST_SERVOS[]    = {0, 1, 2, 3};   // servo indices: 0..3 = PWM1..PWM4 (fin tabs)
constexpr uint32_t ROLL_TEST_START_MS    = 300;      // wait after burnout
constexpr uint32_t ROLL_TEST_STEP_MS     = 300;      // each half of a doublet (+amp, then -amp)
constexpr uint32_t ROLL_TEST_REST_MS     = 400;      // centered between doublets
constexpr float    ROLL_TEST_AMPS_DEG[]  = {5.0f, 10.0f, 15.0f};   // doublet amplitudes, used in turn
constexpr float    ROLL_TEST_STOP_VS_MPS = 15.0f;    // end (tab centered) below this climb rate, before apogee
constexpr uint32_t ROLL_TEST_MAX_MS      = 6500;     // end at the latest this long after burnout
constexpr float    ROLL_TEST_ABORT_DPS   = 360.0f;   // end for good if |roll rate| goes above this

// Transonic barometer lockout: the static-pressure port reads wrong near Mach 1 (shock waves), so
// the barometer is not fed to the EKF while the EKF's Mach number is in this range.
constexpr float BARO_LOCKOUT_MACH_MIN = 0.9f;
constexpr float BARO_LOCKOUT_MACH_MAX = 1.1f;

// Serial plotter output: altitude (z = up, m) and orientation (roll, pitch, yaw, deg), one line
// NAV_PLOT_HZ times per second (0 = off). While it is on, the status line
// above is not printed, so the plotter only ever sees data lines.
//   NAV_PLOT_TELEPLOT = false: "x:1.23,y:4.56,..."  (Arduino IDE 2 Serial Plotter, VS Code Serial Monitor/Plotter)
//   NAV_PLOT_TELEPLOT = true:  ">x:1.23" one per line (Teleplot VS Code extension; each value gets its own chart)
constexpr uint32_t NAV_PLOT_HZ       = 0;
constexpr bool     NAV_PLOT_TELEPLOT = false;

// Board geometry. The filter works in the ICM45686's axes:
//   ICM45686:  +x right, +y toward the nose (top of the board), +z out of the board
//   MMC5983MA: +x toward the tail (-x toward the nose), +y right, +z out of the board
// MAG_AXIS[i] / MAG_SIGN[i] say which magnetometer axis (and sign) lies along IMU axis i:
//   imu_x = mag_y, imu_y = -mag_x, imu_z = mag_z
// (The MMC5983MA is right-handed, so with -x up and +y right its +z must point out of the board.
//  Check: lying flat face-up in the northern hemisphere, the raw mag z reading should be negative.)
constexpr int   MAG_AXIS[3]    = {1, 0, 2};
constexpr float MAG_SIGN[3]    = {1.0f, -1.0f, 1.0f};
constexpr float ROCKET_AXIS[3] = {0.0f, 1.0f, 0.0f};   // nose direction in IMU axes (+y, used for tilt)

// Logged/plotted roll, pitch and yaw are the ROCKET's angles, all 0 with the nose straight up and the
// board's right edge (this axis) facing north:
//   yaw   = nose tilt toward north, pitch = nose tilt toward east (both small near vertical),
//   roll  = spin about the nose (clockwise seen from the tail looking toward the nose)
constexpr float ROCKET_ROLL_REF_AXIS[3] = {1.0f, 0.0f, 0.0f};   // IMU +x = board right
constexpr float MAG_DECLINATION_DEG = 0.0f;            // local declination, east positive (0 = magnetic north)

// ============================================================================
// Tasks (stack sizes are in words = 4 bytes; higher priority runs first)
// ============================================================================
constexpr uint32_t    BOOT_STACK_WORDS   = 4096;
constexpr uint32_t    SENSOR_STACK_WORDS = 2048;
constexpr uint32_t    LOGGER_STACK_WORDS = 4096;
constexpr uint32_t    SERVO_STACK_WORDS  = 1024;
constexpr UBaseType_t SERVO_PRIORITY     = 3;
constexpr int SERVO_CORE  = 1;
constexpr uint32_t    SERVO_TUNE_STACK_WORDS = 1024;
constexpr UBaseType_t SENSOR_PRIORITY    = 5;
constexpr UBaseType_t BOOT_PRIORITY      = 4;
constexpr UBaseType_t LOGGER_PRIORITY    = 3;
constexpr int SENSOR_CORE = 0;
constexpr int LOGGER_CORE = 1;
constexpr int BOOT_CORE   = 1;   // not core 0: the sensor loop keeps core 0 100% busy

// Navigation: the EKF task needs ~9.5 kB of stack for its 15x15 matrix temporaries. Its log rows
// are written by the boot task (core 1), which becomes the nav logger after the speed test.
constexpr uint32_t    NAV_STACK_WORDS = 4096;
constexpr UBaseType_t NAV_PRIORITY    = 5;
constexpr int NAV_CORE = 0;
