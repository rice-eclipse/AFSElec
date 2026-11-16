#include "loop.h"
#include <stdio.h>
#include <math.h>
#include <strings.h>
#include "config.h"
#include "devices.h"
#include "shared.h"
#include "boot.h"
#include "NavEKF.h"
#include "leds.h"
#include "flight_state.h"
#include "roll_test.h"
#include "servo.h"
#include "serial_in.h"

// ============================================================================
// Sensor task (core 0): poll everything as fast as possible
// ============================================================================
static Sample emptySample()
{
    Sample s{};
    s.imuAx = s.imuAy = s.imuAz = s.imuGx = s.imuGy = s.imuGz = s.imuTemp = NAN;
    s.kxX = s.kxY = s.kxZ = NAN;
    s.magX = s.magY = s.magZ = NAN;
    s.pressure = s.baroTemp = s.altitude = NAN;
    s.tmp117 = NAN;
    s.busV = s.currentMa = s.powerMw = NAN;
    return s;
}

// Each poll function reads the sensor only if it has a new sample, updates `s`, and returns its FRESH_ bit (or 0)

static uint8_t pollImu(Sample& s)
{
    ICM45686::Data d;
    if (!imu.dataReady() || !imu.read(d)) return 0;
    s.imuAx = d.ax; s.imuAy = d.ay; s.imuAz = d.az;
    s.imuGx = d.gx; s.imuGy = d.gy; s.imuGz = d.gz; s.imuTemp = d.temperature;
    g_stats.imuNew++;
    return FRESH_IMU;
}

static uint8_t pollAccel(Sample& s)
{
    if (!acc.dataReady()) return 0;
    KX134::Data d;
    bool ok = acc.read(d);
    acc.clearInterrupts();   // release the data-ready flag so the next one is a new sample
    if (!ok) return 0;
    s.kxX = d.x; s.kxY = d.y; s.kxZ = d.z;
    g_stats.accNew++;
    return FRESH_KX134;
}

static uint8_t pollMag(Sample& s)
{
    MMC5983MA::Data d;
    if (!mag.dataReady() || !mag.read(d)) return 0;   // reading the data clears Meas_M_Done
    s.magX = d.x; s.magY = d.y; s.magZ = d.z;
    g_stats.magNew++;
    return FRESH_MAG;
}

static uint8_t pollBaro(Sample& s)
{
    if (!baro.update(BARO_TEMP_EVERY)) return 0;   // non-blocking state machine
    s.pressure = baro.getPressure();
    s.baroTemp = baro.getTemperature();
    s.altitude = baro.getAltitude();
    g_stats.baroNew++;
    return FRESH_BARO;
}

static uint8_t pollPower(Sample& s)
{
    if (!pwr.conversionReady()) return 0;
    s.busV = pwr.readBusVoltage();
    s.currentMa = pwr.readCurrent_mA();
    s.powerMw = pwr.readPower_mW();   // reading power clears the ready flag
    g_stats.inaNew++;
    return FRESH_INA;
}

static uint8_t pollTemp(Sample& s)
{
    if (!temp.dataReady()) return 0;
    s.tmp117 = temp.readTemperature();
    g_stats.tmpNew++;
    return FRESH_TMP;
}

/** Sends a snapshot to the logger without ever blocking; counts it as dropped if the queue is full. */
static void publish(Sample& s, uint8_t fresh, uint64_t tUs)
{
    s.tUs = tUs;
    s.seq = g_stats.produced++;
    s.fresh = fresh;
    if (!g_samples.send(s, 0)) g_stats.dropped++;
    uint32_t queued = g_samples.count();
    if (queued > g_stats.maxQueued) g_stats.maxQueued = queued;
}

void sensorTask(void*)
{
    const SensorStatus& on = g_status;
    Sample s = emptySample();

    const uint64_t start = time_us_64();
    const uint64_t stop = start + (uint64_t)RECORD_DURATION_MS * 1000;
    uint64_t nextIna = start, nextTmp = start;
    uint64_t now = start;

    while (now < stop) {
        uint8_t fresh = 0;

        // Fast SPI sensors: checked every loop
        if (on.imu)  fresh |= pollImu(s);
        if (on.acc)  fresh |= pollAccel(s);
        if (on.mag)  fresh |= pollMag(s);
        if (on.baro) fresh |= pollBaro(s);

        // Slow I2C sensors: checked every INA219_POLL_US / TMP117_POLL_US
        now = time_us_64();
        if (on.pwr && now >= nextIna) {
            nextIna = now + INA219_POLL_US;
            fresh |= pollPower(s);
        }
        if (on.temp && now >= nextTmp) {
            nextTmp = now + TMP117_POLL_US;
            fresh |= pollTemp(s);
        }

        g_stats.loops++;
        if (fresh) publish(s, fresh, now - start);
        now = time_us_64();
    }

    g_stats.sensorUs = now - start;
    g_sensorsDone = true;
    g_events.set(EVT_SENSOR_DONE);
    vTaskDelete(nullptr);
}

// ============================================================================
// Logger task (core 1): write snapshots to the SD card as fast as possible
// ============================================================================
static void writeCsvRow(const Sample& s)
{
    logFile.add(s.tUs).add(s.seq).add((uint32_t)s.fresh)
           .add(s.imuAx, 4).add(s.imuAy, 4).add(s.imuAz, 4).add(s.imuGx, 2).add(s.imuGy, 2).add(s.imuGz, 2)
           .add(s.imuTemp, 2)
           .add(s.kxX, 3).add(s.kxY, 3).add(s.kxZ, 3)
           .add(s.magX, 5).add(s.magY, 5).add(s.magZ, 5)
           .add(s.pressure, 2).add(s.baroTemp, 2).add(s.altitude, 2)
           .add(s.tmp117, 3)
           .add(s.busV, 3).add(s.currentMa, 1).add(s.powerMw, 1)
           .endRow();
}

void loggerTask(void*)
{
    const bool toFile = g_stats.fileOk;
    const uint64_t start = time_us_64();

    Sample s;
    while (true) {
        if (g_samples.receive(s, 5)) {
            if (toFile) writeCsvRow(s);
            g_stats.logged++;
        } else if (g_sensorsDone && g_samples.empty()) {
            break;   // sensor loop finished and everything it produced has been written
        }
    }

    if (toFile) {
        g_stats.writeErrors = logFile.errorCount();
        if (!logFile.close()) g_stats.writeErrors++;
    }
    g_stats.loggerUs = time_us_64() - start;
    g_events.set(EVT_LOGGER_DONE);
    vTaskDelete(nullptr);
}

// ============================================================================
// End-of-run report
// ============================================================================
static void printRate(const char* name, uint32_t count, uint64_t us, const char* note)
{
    double hz = us ? count * 1e6 / (double)us : 0.0;
    printf("  %-26s %9.1f Hz  (%lu)%s\n", name, hz, (unsigned long)count, note);
}

void printRunReport()
{
    const RunStats& r = g_stats;
    const SensorStatus& on = g_status;
    const uint64_t us = r.sensorUs;

    if (r.fileOk) {
        printf("Ending write: %s, %lu rows%s\n", g_logName, (unsigned long)r.logged,
               r.writeErrors ? " (SD write errors, file may be incomplete)" : "");
    }

    printf("\n========== Max-rate test (%.1f s) ==========\n", us / 1e6);
    printf("Sensor task (core 0), new samples per second:\n");
    printRate("loop (all sensors polled)", r.loops, us, "");
    printRate("ICM45686 accel+gyro", r.imuNew, us, on.imu ? "  [ODR 6400 Hz]" : "  [not started]");
    printRate("KX134 accel", r.accNew, us, on.acc ? "  [ODR 25600 Hz]" : "  [not started]");
    printRate("MMC5983MA mag", r.magNew, us, on.mag ? "  [1000 Hz continuous]" : "  [not started]");
    printRate("MS5611 pressure", r.baroNew, us, on.baro ? "  [OSR 256]" : "  [not started]");
    printRate("TMP117 temperature", r.tmpNew, us, on.temp ? "  [15.5 ms conversion]" : "  [not started]");
    printRate("INA219 power", r.inaNew, us, on.pwr ? "  [12-bit shunt + bus]" : "  [not started]");
    printRate("snapshots produced", r.produced, us, "");

    printf("Logger task (core 1):\n");
    printRate(r.fileOk ? "rows written to SD" : "snapshots consumed (no SD)", r.logged, r.loggerUs, "");
    printf("  dropped (logger behind):   %lu of %lu (%.1f%%), max queue fill %lu / %lu\n",
           (unsigned long)r.dropped, (unsigned long)r.produced,
           r.produced ? 100.0 * r.dropped / r.produced : 0.0, (unsigned long)r.maxQueued,
           (unsigned long)SAMPLE_QUEUE_LEN);
    printf("============================================\n");
}

// ============================================================================
// Navigation: NavEKF at full sensor speed (core 0), forever
// ============================================================================
static NavEKF nav;                 // ~2.5 kB: static, not on the task stack
static float  navLastPressure = NAN;
static FlightStateMachine flight;
static float  navPadTempK = 288.15f;
static float  navMach = 0.0f;
static bool   navBaroLocked = false;   // transonic: barometer not fed to the EKF
static bool   navImuNew = false;       // navFeed() got a new IMU sample
static ekf::Vec3 navAccel = ekf::Vec3::zeros(), navGyro = ekf::Vec3::zeros();   // latest IMU sample fed to the EKF
// High-g takeover: above HIGHG_SWITCH_G (KX134) the ICM45686 accelerometer is ignored
static bool   navHighGOk = false;                          // KX134 passed the pad cross-check
static bool   navUsingHighG = false;                       // KX134 is the accelerometer right now
static float  navHighGMag = 0.0f;                          // latest KX134 |accel|, g
static ekf::Vec3 navHighGOffset = ekf::Vec3::zeros();      // KX134 -> ICM45686 offset from the pad, m/s^2
// Roll test, and the rate/accel averages logged with every row
static RollTest  rollTest;
static ekf::Vec3 navRateSum = ekf::Vec3::zeros(), navAccSum = ekf::Vec3::zeros();   // bias-corrected, since the last row
static float     navRollRateSum = 0.0f;
static uint32_t  navAvgN = 0;
// "state" serial command: the logger (core 1) counts requests, the nav task (core 0) applies them
static volatile uint32_t navStateRequests = 0;
static bool   navFileOk = false;
static char   navLogName[48];

// Sensor reads in the filter's (IMU) axes and units; each returns true only for a new sample
static bool navReadImu(ekf::Vec3& accel, ekf::Vec3& gyro)
{
    ICM45686::Data d;
    if (!imu.dataReady() || !imu.read(d)) return false;
    accel = ekf::vec3(d.ax, d.ay, d.az) * NavEKF::G;          // g -> m/s^2
    gyro = ekf::vec3(d.gx, d.gy, d.gz) * NavEKF::DEG2RAD;     // deg/s -> rad/s
    return true;
}

/** Latest KX134 reading in IMU axes, m/s^2, without the pad offset (no data-ready wait: newest sample). */
static bool navReadHighG(ekf::Vec3& accel)
{
    KX134::Data d;
    if (!acc.read(d)) return false;
    const float m[3] = {d.x, d.y, d.z};
    for (int i = 0; i < 3; i++) accel[i] = HIGHG_SIGN[i] * m[HIGHG_AXIS[i]] * NavEKF::G;   // KX134 -> IMU axes
    return true;
}

static bool navReadMag(ekf::Vec3& field)
{
    MMC5983MA::Data d;
    if (!mag.dataReady() || !mag.read(d)) return false;
    const float m[3] = {d.x, d.y, d.z};
    for (int i = 0; i < 3; i++) field[i] = MAG_SIGN[i] * m[MAG_AXIS[i]];   // magnetometer -> IMU axes
    return true;
}

static bool navReadBaro(float& mbar)
{
    if (!baro.update(BARO_TEMP_EVERY)) return false;
    mbar = baro.getPressure();
    return true;
}

/** Time spent inside the EKF, accumulated between log rows. */
struct EkfTimer {
    uint32_t busyUs = 0, maxUs = 0;
    void add(uint32_t t0) {
        const uint32_t us = time_us_32() - t0;
        busyUs += us;
        if (us > maxUs) maxUs = us;
    }
};

struct NavCounts { uint32_t imu = 0, baro = 0, mag = 0, rows = 0, dropped = 0; };

/** Feeds every sensor that has new data to the filter. */
static void navFeed(EkfTimer& timer, NavCounts& n)
{
    const SensorStatus& on = g_status;
    ekf::Vec3 a, g, m;
    float p;
    navImuNew = false;
    if (on.imu && navReadImu(a, g)) {
        // High-g takeover: once the KX134 sees more than HIGHG_SWITCH_G the ICM45686 accelerometer
        // (+-16 g) is near clipping, so the KX134 replaces it until it drops below HIGHG_RETURN_G.
        // The gyro always comes from the ICM45686.
        ekf::Vec3 h;
        if (navHighGOk && navReadHighG(h)) {
            h = h + navHighGOffset;
            navHighGMag = ekf::norm(h) / NavEKF::G;
            if (!navUsingHighG && navHighGMag > HIGHG_SWITCH_G) navUsingHighG = true;
            else if (navUsingHighG && navHighGMag < HIGHG_RETURN_G) navUsingHighG = false;
            if (navUsingHighG) a = h;
        } else {
            navUsingHighG = false;
        }
        const uint32_t t0 = time_us_32();
        nav.imu(time_us_64(), a, g);
        timer.add(t0);
        n.imu++;
        navAccel = a;
        navGyro = g;
        navImuNew = true;
    }
    if (on.mag && navReadMag(m)) {
        const uint32_t t0 = time_us_32();
        nav.magnetometer(time_us_64(), m);
        timer.add(t0);
        n.mag++;
    }
    if (on.baro && navReadBaro(p)) {
        navLastPressure = p;   // still logged (baro_alt_m) while locked out, for comparison
        if (!navBaroLocked) {
            const uint32_t t0 = time_us_32();
            nav.baroPressure(time_us_64(), p);
            timer.add(t0);
            n.baro++;
        }
    }
}

static void navPublish(uint64_t tUs, uint64_t periodUs, NavPhase phase, EkfTimer& timer, NavCounts& n)
{
    NavRow r{};
    r.tUs = tUs;
    r.seq = n.rows++;
    r.phase = (uint8_t)phase;
    r.inFlight = nav.inFlight();
    r.flightState = (uint8_t)flight.state();
    r.baroLocked = navBaroLocked;
    r.mach = navMach;
    r.highG = navUsingHighG;
    r.highGAccel = navHighGMag;

    // Body rates and specific force, averaged over the IMU samples since the previous row
    const float inv = navAvgN ? 1.0f / navAvgN : 0.0f;
    r.p = navRateSum[0] * inv * NavEKF::RAD2DEG;
    r.q = navRateSum[1] * inv * NavEKF::RAD2DEG;
    r.r = navRateSum[2] * inv * NavEKF::RAD2DEG;
    r.rollRate = navRollRateSum * inv;
    r.ax = navAccSum[0] * inv; r.ay = navAccSum[1] * inv; r.az = navAccSum[2] * inv;
    navRateSum = navAccSum = ekf::Vec3::zeros();
    navRollRateSum = 0.0f;
    navAvgN = 0;

    // Dynamic pressure from the barometer and the ISA temperature at this altitude
    r.speed = nav.speed();
    r.pressure = navLastPressure;
    float tK = navPadTempK - 0.0065f * (nav.altitude() > 0.0f ? nav.altitude() : 0.0f);
    const float rho = isfinite(navLastPressure) ? navLastPressure * 100.0f / (287.05f * tK) : NAN;
    r.qbar = 0.5f * rho * r.speed * r.speed;

    r.tabDeg = rollTest.deflection();
    r.tabUs = (uint16_t)servoPulseUs(ROLL_TEST_SERVOS[0]);   // first tab (the others differ only by their centers)
    r.rollTestStatus = (uint8_t)rollTest.status();
    r.rollTestDoublet = rollTest.doublet();
    const ekf::Quat& q = nav.quaternion();
    r.qw = q.w; r.qx = q.x; r.qy = q.y; r.qz = q.z;
    const NavEKF::RocketAngles ang = nav.rocketAngles();
    r.roll = ang.roll; r.pitch = ang.pitch; r.yaw = ang.yaw;
    r.tilt = nav.tilt(); r.tiltAz = nav.tiltAzimuth();
    r.north = nav.north(); r.east = nav.east(); r.alt = nav.altitude();
    const ekf::Vec3& v = nav.velocityNED();
    r.vn = v[0]; r.ve = v[1]; r.vd = v[2];
    r.baroAlt = isfinite(navLastPressure) ? nav.altitudeAt(navLastPressure) : NAN;
    r.tiltStd = nav.tiltStd(); r.headingStd = nav.headingStd();
    r.altStd = nav.altitudeStd(); r.horizStd = nav.horizontalStd();
    const ekf::Vec3& ba = nav.accelBias();
    const ekf::Vec3& bg = nav.gyroBias();
    r.bax = ba[0]; r.bay = ba[1]; r.baz = ba[2];
    r.bgx = bg[0] * NavEKF::RAD2DEG; r.bgy = bg[1] * NavEKF::RAD2DEG; r.bgz = bg[2] * NavEKF::RAD2DEG;
    r.ekfLoad = periodUs ? 100.0f * timer.busyUs / (float)periodUs : 0.0f;
    r.ekfMaxUs = timer.maxUs;
    r.imuN = n.imu; r.baroN = n.baro; r.magN = n.mag;
    r.dropped = n.dropped;
    timer = EkfTimer{};

    if (!g_navRows->send(r, 0)) n.dropped++;   // never block the filter
}

static void navTask(void*)
{
    const SensorStatus& on = g_status;
    const uint64_t start = time_us_64();

    // ---- 1. Average the sensors at rest, then start the filter ----
    ekf::Vec3 accSum = ekf::Vec3::zeros(), magSum = ekf::Vec3::zeros();
    ekf::Vec3 hgDiffSum = ekf::Vec3::zeros();   // ICM45686 - KX134, sampled together
    double pSum = 0.0, tSum = 0.0;
    uint32_t accN = 0, magN = 0, pN = 0, hgN = 0;
    uint64_t now = start;
    while ((now - start < (uint64_t)NAV_ALIGN_AVG_MS * 1000 || accN == 0) && now - start < 5'000'000) {
        ekf::Vec3 a, g, m, h;
        float p;
        if (on.imu && navReadImu(a, g)) {
            accSum = accSum + a;
            accN++;
            if (HIGHG_ENABLE && on.acc && navReadHighG(h)) { hgDiffSum = hgDiffSum + (a - h); hgN++; }
        }
        if (on.mag && navReadMag(m)) { magSum = magSum + m; magN++; }
        if (on.baro && navReadBaro(p)) { pSum += p; tSum += baro.getTemperature(); pN++; navLastPressure = p; }
        now = time_us_64();
    }

    NavEKF::Config cfg;
    cfg.padTemperatureK = pN ? 273.15f + (float)(tSum / pN) : 288.15f;
    cfg.magDeclinationDeg = MAG_DECLINATION_DEG;
    for (int i = 0; i < 3; i++) {
        cfg.rocketAxis[i] = ROCKET_AXIS[i];
        cfg.rocketRollRefAxis[i] = ROCKET_ROLL_REF_AXIS[i];
    }
    const ekf::Vec3 accAvg = accN ? accSum * (1.0f / accN) : ekf::Vec3::zeros();

    // High-g cross-check: at rest both accelerometers measure the same 1 g, so a large difference
    // means HIGHG_AXIS / HIGHG_SIGN are wrong (or a sensor is bad): then the ICM45686 is used alone.
    if (HIGHG_ENABLE && hgN > 0) {
        const ekf::Vec3 diff = hgDiffSum * (1.0f / hgN);
        const float diffG = ekf::norm(diff) / NavEKF::G;
        navHighGOk = diffG < HIGHG_PAD_MAX_DIFF_G;
        if (navHighGOk) navHighGOffset = diff;
        printf("High-g KX134 vs ICM45686 on the pad: diff (%.3f %.3f %.3f) g -> %s\n",
               diff[0] / NavEKF::G, diff[1] / NavEKF::G, diff[2] / NavEKF::G,
               navHighGOk ? "OK, high-g takeover enabled"
                          : "TOO LARGE: check HIGHG_AXIS/HIGHG_SIGN in config.h; high-g takeover disabled");
    } else if (HIGHG_ENABLE) {
        printf("High-g KX134 not available: ICM45686 accelerometer used alone\n");
    }
    const ekf::Vec3 magAvg = magN ? magSum * (1.0f / magN) : ekf::Vec3::zeros();   // zeros = no magnetometer
    const float padP = pN ? (float)(pSum / pN) : 1013.25f;
    navPadTempK = cfg.padTemperatureK;

    if (accN == 0 || !nav.begin(padP, accAvg, magAvg, time_us_64(), cfg)) {
        printf("NavEKF did not start: %s\n", on.imu ? "no usable accelerometer reading" : "IMU not available");
        g_navFailed = true;
        vTaskDelete(nullptr);
    }

    // ---- 2. Align while still, then 3. navigate forever ----
    const uint64_t alignEnd = start + (uint64_t)NAV_ALIGN_MS * 1000;
    const uint64_t idleLogPeriod = 1'000'000 / NAV_LOG_IDLE_HZ;
    const uint64_t flightLogPeriod = 1'000'000 / NAV_LOG_FLIGHT_HZ;
    const uint64_t stationaryPeriod = 1'000'000 / NAV_STATIONARY_HZ;
    const float stationaryDt = 1.0f / NAV_STATIONARY_HZ;
    uint64_t nextLog = time_us_64(), nextStationary = nextLog, lastLog = nextLog;
    NavPhase phase = NavPhase::Aligning;
    EkfTimer timer;
    NavCounts counts;
    float padPressure = padP;                    // pad hold: slowly follows the measured pressure
    float maxGyroDps = 0.0f, maxAccelDevG = 0.0f;   // pad hold: motion since the last stationary tick
    uint32_t stateRequestsDone = 0;                 // "state" commands applied
    bool servosHeld = true;                         // servos centered by startNavigation() just before this task
    const uint64_t servosHeldSinceUs = start;

    while (true) {
        now = time_us_64();
        navFeed(timer, counts);

        // Transonic barometer lockout, from the EKF's own speed
        navMach = machNumber(nav.speed(), nav.altitude(), navPadTempK);
        navBaroLocked = navMach >= BARO_LOCKOUT_MACH_MIN && navMach <= BARO_LOCKOUT_MACH_MAX;

        bool stateChanged = false;
        if (navImuNew) {
            const ekf::Vec3 a = navAccel - nav.accelBias();
            const ekf::Vec3 w = navGyro - nav.gyroBias();
            FlightInputs in;
            in.accelG = ekf::norm(a) / NavEKF::G;
            in.axialG = (a[0] * ROCKET_AXIS[0] + a[1] * ROCKET_AXIS[1] + a[2] * ROCKET_AXIS[2]) / NavEKF::G;
            in.gyroDps = ekf::norm(w) * NavEKF::RAD2DEG;
            in.altM = nav.altitude();
            in.vertSpeed = nav.verticalSpeed();

            if (in.gyroDps > maxGyroDps) maxGyroDps = in.gyroDps;
            if (fabsf(in.accelG - 1.0f) > maxAccelDevG) maxAccelDevG = fabsf(in.accelG - 1.0f);

            // Rates and accel for the log, averaged between rows
            const float rollRateDps = (w[0] * ROCKET_AXIS[0] + w[1] * ROCKET_AXIS[1] + w[2] * ROCKET_AXIS[2]) * NavEKF::RAD2DEG;
            navRateSum = navRateSum + w;
            navAccSum = navAccSum + a;
            navRollRateSum += rollRateDps;
            navAvgN++;

            // Flight state machine, once per IMU sample after alignment
            if (phase == NavPhase::Navigating) {
                // "state" serial command (bench testing): one step per request
                if (NAV_SERIAL_STATE_CMD && navStateRequests != stateRequestsDone) {
                    stateRequestsDone++;
                    stateChanged |= flight.advance(now);
                }
                if (!(NAV_SERIAL_STATE_CMD && NAV_SERIAL_STATE_MANUAL))   // manual mode: detectors paused
                    stateChanged |= flight.update(now, in);   // the logger prints state changes (no UART time here)
                if (flight.state() != FlightState::Launchpad && !nav.inFlight()) nav.setInFlight(true);

                // Roll test: every fin tab gets the same doublet angle during coast (register writes
                // on each change, so the command lines up with this IMU sample in the log)
                if (ROLL_TEST_ENABLE) {
                    const float before = rollTest.deflection();
                    const float deg = rollTest.update(now, flight.state(), flight.stateSinceUs(), in.vertSpeed, rollRateDps);
                    if (deg != before)
                        for (int s : ROLL_TEST_SERVOS) servoWriteDeg(s, deg);
                }
            }
        }

        // Servo power: on the ground (alignment, pad, and after landing if enabled) the servos are
        // released once they've had SERVO_PAD_CENTER_MS to reach center; from launch on they are
        // driven to center and held for the flight.
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
                servoCenterAll();   // launch: hold center actively for the flight
                servosHeld = true;
            }
        }

        // Pad hold: still on the pad -> zero velocity/rotation, horizontal position 0, altitude zero
        // follows the weather. Skipped (and the gyro window dropped) if the IMU moved.
        if (PAD_HOLD && phase == NavPhase::Navigating && flight.state() == FlightState::Launchpad &&
            now >= nextStationary) {
            nextStationary += stationaryPeriod;
            if (nextStationary < now) nextStationary = now + stationaryPeriod;
            const uint32_t t0 = time_us_32();
            if (maxGyroDps < PAD_STILL_GYRO_DPS && maxAccelDevG < PAD_STILL_ACCEL_G) {
                nav.stationary(now);
                nav.horizontalPosition(now, 0.0f, 0.0f, PAD_HOLD_POS_STD_M);
                if (isfinite(navLastPressure)) {
                    padPressure += (navLastPressure - padPressure) * (stationaryDt / PAD_PRESSURE_TAU_S);
                    nav.setPadPressure(padPressure);
                }
            } else {
                nav.clearStationaryWindow();
            }
            timer.add(t0);
            maxGyroDps = maxAccelDevG = 0.0f;
        }

        if (phase == NavPhase::Aligning) {
            if (now >= alignEnd) {
                phase = NavPhase::Navigating;
                if (NAV_FLIGHT_MODE_AFTER_ALIGN) nav.setInFlight(true);
                // One-off message (~10 ms of UART time; the filter catches up from the timestamps)
                printf("Aligned: roll %.1f pitch %.1f yaw %.1f deg, gyro bias (%.3f %.3f %.3f) deg/s. Navigating.\n",
                       nav.rocketAngles().roll, nav.rocketAngles().pitch, nav.rocketAngles().yaw, nav.gyroBias()[0] * NavEKF::RAD2DEG,
                       nav.gyroBias()[1] * NavEKF::RAD2DEG, nav.gyroBias()[2] * NavEKF::RAD2DEG);
            } else if (now >= nextStationary) {
                nextStationary += stationaryPeriod;
                const uint32_t t0 = time_us_32();
                nav.stationary(now);
                timer.add(t0);
                maxGyroDps = maxAccelDevG = 0.0f;
            }
        }

        // Log fast in flight, slowly on the pad / landed; a state change is logged immediately
        const FlightState fs = flight.state();
        const bool flying = phase == NavPhase::Navigating && fs != FlightState::Launchpad && fs != FlightState::Landed;
        const uint64_t logPeriod = flying ? flightLogPeriod : idleLogPeriod;
        if (stateChanged) nextLog = now;
        if (now >= nextLog) {
            nextLog += logPeriod;
            if (nextLog < now) nextLog = now + logPeriod;   // after a slow period or a state change
            navPublish(now - start, now - lastLog, phase, timer, counts);
            lastLog = now;
        }
    }
}

static void writeNavHeader()
{
    logFile.writeHeader({
        "t_us", "seq", "phase", "in_flight", "flight_state", "mach", "baro_locked", "high_g", "high_g_accel_g",
        "p_dps", "q_dps", "r_dps", "roll_rate_dps", "ax_mps2", "ay_mps2", "az_mps2",
        "speed_mps", "pressure_mbar", "qbar_pa", "tab_deg", "tab_us", "roll_test", "doublet",
        "qw", "qx", "qy", "qz", "roll_deg", "pitch_deg", "yaw_deg", "tilt_deg", "tilt_az_deg",
        "north_m", "east_m", "alt_m", "vn_mps", "ve_mps", "vd_mps", "baro_alt_m",
        "tilt_std_deg", "heading_std_deg", "alt_std_m", "horiz_std_m",
        "ba_x_mps2", "ba_y_mps2", "ba_z_mps2", "bg_x_dps", "bg_y_dps", "bg_z_dps",
        "ekf_load_pct", "ekf_max_us", "imu_n", "baro_n", "mag_n", "dropped",
    });
}

bool startNavigation()
{
    // Created here, after the speed test's tasks are gone, so their stacks are back in the heap
    if (!g_navRows) g_navRows = new rtos::Queue<NavRow>(NAV_QUEUE_LEN);
    if (!g_navRows->valid()) panic("not enough FreeRTOS heap for the navigation queue");

    navFileOk = createLogFile(NAV_LOG_PREFIX, navLogName, sizeof navLogName);
    if (navFileOk) writeNavHeader();

    servoInit();   // every servo to its center (the roll test moves ROLL_TEST_SERVOS during coast)
    printf("\nNavigation: keep the board still for %.1f s to align\n", NAV_ALIGN_MS / 1e3);
    if (NAV_SERIAL_STATE_CMD)
        printf("Serial command on: send \"state\" to step the flight state%s\n",
               NAV_SERIAL_STATE_MANUAL ? " (automatic detection paused)" : "");
    if (navFileOk) printf("Starting write: %s\n", navLogName);
    rtos::spawn(navTask, "nav", NAV_STACK_WORDS, NAV_PRIORITY, nullptr, NAV_CORE);
    return navFileOk;
}

// ============================================================================
// Navigation logger (the calling task, core 1), forever
// ============================================================================
static void writeNavRow(const NavRow& r)
{
    logFile.add(r.tUs).add(r.seq).add((uint32_t)r.phase).add((uint32_t)r.inFlight)
           .add((uint32_t)r.flightState).add(r.mach, 3).add((uint32_t)r.baroLocked)
           .add((uint32_t)r.highG).add(r.highGAccel, 2)
           .add(r.p, 2).add(r.q, 2).add(r.r, 2).add(r.rollRate, 2)
           .add(r.ax, 3).add(r.ay, 3).add(r.az, 3)
           .add(r.speed, 2).add(r.pressure, 3).add(r.qbar, 1)
           .add(r.tabDeg, 2).add((uint32_t)r.tabUs).add((uint32_t)r.rollTestStatus).add((uint32_t)r.rollTestDoublet)
           .add(r.qw, 6).add(r.qx, 6).add(r.qy, 6).add(r.qz, 6)
           .add(r.roll, 2).add(r.pitch, 2).add(r.yaw, 2).add(r.tilt, 2).add(r.tiltAz, 1)
           .add(r.north, 3).add(r.east, 3).add(r.alt, 3).add(r.vn, 3).add(r.ve, 3).add(r.vd, 3)
           .add(r.baroAlt, 2)
           .add(r.tiltStd, 3).add(r.headingStd, 3).add(r.altStd, 3).add(r.horizStd, 3)
           .add(r.bax, 4).add(r.bay, 4).add(r.baz, 4).add(r.bgx, 4).add(r.bgy, 4).add(r.bgz, 4)
           .add(r.ekfLoad, 1).add(r.ekfMaxUs).add(r.imuN).add(r.baroN).add(r.magN).add(r.dropped)
           .endRow();
}

/**
 * Total nav rows that never made it to the SD card so far: rows dropped because the logger fell
 * behind (queue full) plus failed SD writes.
 */
static unsigned long navDroppedTotal(const NavRow& r)
{
    return (unsigned long)r.dropped + (navFileOk ? (unsigned long)logFile.errorCount() : 0ul);
}

/** One status line: flight state, altitude, attitude, IMU rate and average EKF load since the previous status. */
static void printNavStatus(const NavRow& r, const NavRow& prev, float avgLoad)
{
    const double dt = (r.tUs - prev.tUs) * 1e-6;
    const double imuHz = dt > 0 ? (r.imuN - prev.imuN) / dt : 0.0;
    printf("[nav %7.1f s] %s | alt %7.2f m (1-sigma %.2f) vs %6.1f m/s M %.2f%s | roll %6.1f pitch %6.1f yaw %5.1f"
           " tilt %5.1f deg | IMU %.0f Hz, EKF %.0f%% of core 0 | SD rows dropped %lu (queue %lu, write errors %lu)\n",
           r.tUs * 1e-6, flightStateName((FlightState)r.flightState), r.alt, r.altStd, -r.vd, r.mach,
           r.baroLocked ? " (baro off)" : r.highG ? " (high-g)" : "", r.roll, r.pitch, r.yaw, r.tilt, imuHz,
           avgLoad, navDroppedTotal(r), (unsigned long)r.dropped,
           navFileOk ? (unsigned long)logFile.errorCount() : 0ul);
}

/** One serial-plotter line: altitude (z up, m), orientation (deg) and total SD rows dropped. */
static void printNavPlot(const NavRow& r)
{
    if constexpr (NAV_PLOT_TELEPLOT) {
        printf(">z:%.3f\n>roll:%.2f\n>pitch:%.2f\n>yaw:%.2f\n>dropped:%lu\n", r.alt, r.roll, r.pitch, r.yaw,
               navDroppedTotal(r));
    } else {
        printf("z:%.3f,roll:%.2f,pitch:%.2f,yaw:%.2f,dropped:%lu\n", r.alt, r.roll, r.pitch, r.yaw,
               navDroppedTotal(r));
    }
}

void navLoggerLoop()
{
    constexpr bool plotting = NAV_PLOT_HZ > 0;
    constexpr uint64_t plotPeriodUs = plotting ? 1'000'000ull / NAV_PLOT_HZ : 0;
    constexpr bool statusOn = NAV_STATUS_PERIOD_MS > 0 && !plotting;   // text lines would confuse the plotter

    NavRow r, prevStatus{};
    uint8_t lastFlightState = (uint8_t)FlightState::Launchpad;
    uint8_t lastRollTest = (uint8_t)RollTest::Status::Waiting;
    float loadSum = 0.0f;
    uint32_t loadN = 0;
    bool havePrev = false;
    uint64_t nextStatusUs = (uint64_t)NAV_STATUS_PERIOD_MS * 1000;
    uint64_t nextPlotUs = 0;

    while (true) {
        if (!g_navRows->receive(r, 100)) {
            if (g_navFailed && navFileOk) {   // nothing will ever arrive: close the empty log
                logFile.close();
                navFileOk = false;
            }
            continue;
        }
        if (navFileOk) writeNavRow(r);   // CSVWriter syncs to the card every second

        // "state" serial command (bench testing): ask the nav task for the next flight state
        if constexpr (NAV_SERIAL_STATE_CMD) {
            static SerialLineReader cmd(300, /*echo*/ false);
            char line[32];
            if (cmd.poll(line, sizeof line)) {
                if (strcasecmp(line, "state") != 0) {
                    printf("[nav] unknown command \"%s\" (send \"state\")\n", line);
                } else if (r.phase != (uint8_t)NavPhase::Navigating) {
                    printf("[nav] \"state\" ignored: still aligning\n");
                } else if (r.flightState == (uint8_t)FlightState::Landed) {
                    printf("[nav] \"state\" ignored: already LANDED\n");
                } else {
                    navStateRequests = navStateRequests + 1;
                    printf("[nav] \"state\" received: advancing from %s\n", flightStateName((FlightState)r.flightState));
                }
            }
        }

        if (r.flightState != lastFlightState) {
            lastFlightState = r.flightState;
            printf("[nav %7.1f s] STATE -> %s (alt %.1f m, vs %.1f m/s, M %.2f)\n", r.tUs * 1e-6,
                   flightStateName((FlightState)r.flightState), r.alt, -r.vd, r.mach);
        }
        if (r.rollTestStatus != lastRollTest) {
            lastRollTest = r.rollTestStatus;
            printf("[nav %7.1f s] ROLL TEST -> %s (doublet %u, speed %.1f m/s, qbar %.0f Pa, roll rate %.0f deg/s)\n",
                   r.tUs * 1e-6, rollTestStatusName((RollTest::Status)r.rollTestStatus), r.rollTestDoublet,
                   r.speed, r.qbar, r.rollRate);
        }

        // Blue LED = nav lock: aligned and navigating, with pitch and yaw both within NAV_LOCK_DEG
        ledSet(LED_BLUE, r.phase == (uint8_t)NavPhase::Navigating &&
                         fabsf(r.pitch) < NAV_LOCK_DEG && fabsf(r.yaw) < NAV_LOCK_DEG);

        loadSum += r.ekfLoad;
        loadN++;
        if (!havePrev) { prevStatus = r; havePrev = true; }
        if (plotting && r.phase == (uint8_t)NavPhase::Navigating && r.tUs >= nextPlotUs) {
            // Skip ahead rather than catching up if rows arrive late
            nextPlotUs = (nextPlotUs + plotPeriodUs > r.tUs) ? nextPlotUs + plotPeriodUs : r.tUs + plotPeriodUs;
            printNavPlot(r);
        }

        if (statusOn && r.tUs >= nextStatusUs) {
            nextStatusUs += (uint64_t)NAV_STATUS_PERIOD_MS * 1000;
            printNavStatus(r, prevStatus, loadN ? loadSum / loadN : 0.0f);
            prevStatus = r;
            loadSum = 0.0f;
            loadN = 0;
        }
    }
}
