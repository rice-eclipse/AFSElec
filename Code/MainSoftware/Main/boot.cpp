#include "boot.h"
#include <stdio.h>
#include "config.h"
#include "devices.h"

#include "ICM45686_test.h"
#include "KX134_test.h"
#include "MMC5983MA_test.h"
#include "MS5611_test.h"
#include "TMP117_test.h"
#include "INA219_test.h"
#include "SDCard_test.h"
#include "FreeRTOS_test.h"

// ============================================================================
// Startup tests
// ============================================================================
bool runStartupTests()
{
    struct Result { const char* name; bool ran; bool passed; };
    Result results[8] = {
        {"ICM45686", false, false}, {"KX134", false, false}, {"MMC5983MA", false, false},
        {"MS5611", false, false},   {"TMP117", false, false}, {"INA219", false, false},
        {"SDCard", false, false},   {"FreeRTOS", false, false},
    };

    if constexpr (RUN_ICM45686_TEST)  { results[0].ran = true; results[0].passed = ICM45686_test(imu, VERBOSE_ICM45686_TEST); }
    if constexpr (RUN_KX134_TEST)     { results[1].ran = true; results[1].passed = KX134_test(acc, VERBOSE_KX134_TEST); }
    if constexpr (RUN_MMC5983MA_TEST) { results[2].ran = true; results[2].passed = MMC5983MA_test(mag, VERBOSE_MMC5983MA_TEST); }
    if constexpr (RUN_MS5611_TEST)    { results[3].ran = true; results[3].passed = MS5611_test(baro, VERBOSE_MS5611_TEST); }
    if constexpr (RUN_TMP117_TEST)    { results[4].ran = true; results[4].passed = TMP117_test(temp, VERBOSE_TMP117_TEST); }
    if constexpr (RUN_INA219_TEST)    { results[5].ran = true; results[5].passed = INA219_test(pwr, VERBOSE_INA219_TEST); }
    if constexpr (RUN_SDCARD_TEST)    { results[6].ran = true; results[6].passed = SDCard_test(sd, VERBOSE_SDCARD_TEST); }
    if constexpr (RUN_FREERTOS_TEST)  { results[7].ran = true; results[7].passed = FreeRTOS_test(VERBOSE_FREERTOS_TEST); }

    printf("\n========== Startup test summary ==========\n");
    for (const Result& r : results) {
        printf("  %-10s %s\n", r.name, !r.ran ? "skipped" : r.passed ? "PASS" : "FAIL");
    }
    printf("==========================================\n\n");

    bool allPassed = true;
    for (const Result& r : results) if (r.ran && !r.passed) allPassed = false;
    return allPassed;
}

// ============================================================================
// Sensor setup for maximum rate
// ============================================================================
SensorStatus startSensorsMaxRate()
{
    // The bench tests leave each device at its defaults, and a disabled test never calls begin(),
    // so begin() is called again here either way.
    SensorStatus s{};

    imu.setSpiFrequency(IMU_SPI_HZ);
    s.imu = imu.begin();
    if (s.imu) {
        imu.setAccelODR(ICM45686::ODR::Hz6400);   // fastest low-noise rate
        imu.setGyroODR(ICM45686::ODR::Hz6400);
        imu.enableDataReadyInterrupt(true);       // needed for dataReady()
    }

    acc.setSpiFrequency(ACC_SPI_HZ);
    s.acc = acc.begin();
    if (s.acc) {
        acc.setRange(KX134::Range::G64);
        acc.setODR(KX134::ODR::Hz25600);          // fastest rate (high-performance mode)
        acc.enableDataReadyEngine(true);          // needed for dataReady()
    }

    mag.setSpiFrequency(MAG_SPI_HZ);
    s.mag = mag.begin();
    if (s.mag) {
        mag.calibrateOffset();                    // one-shot mode, before continuous
        mag.enableAutoSetReset(false);            // auto set/reset roughly halves the rate
        mag.setBandwidth(MMC5983MA::Bandwidth::Hz800);              // required for 1000 Hz
        mag.startContinuous(MMC5983MA::ContinuousRate::Hz1000);
    }

    baro.setSpiFrequency(BARO_SPI_HZ);
    s.baro = baro.begin();                        // also takes a first reading
    if (s.baro) {
        baro.setOversampling(MS5611::OSR::OSR256);   // fastest conversion (~0.6 ms)
        baro.read();
        baro.calibrateAltitude(0.0f);             // altitude is relative to power-on height
    }

    s.temp = temp.begin();
    if (s.temp) {
        temp.setAveraging(TMP117::Averaging::None);
        temp.setConversionCycle(TMP117::ConversionCycle::C0);    // new value every 15.5 ms
    }

    s.pwr = pwr.begin();
    if (s.pwr) {
        pwr.setAutoRecalibrate(false);            // skip the calibration write before every read
    }
    return s;
}

// ============================================================================
// Log file
// ============================================================================
static void writeCsvHeader()
{
    logFile.writeHeader({
        "t_us", "seq", "fresh",
        "imu_ax_g", "imu_ay_g", "imu_az_g", "imu_gx_dps", "imu_gy_dps", "imu_gz_dps", "imu_temp_c",
        "kx_ax_g", "kx_ay_g", "kx_az_g",
        "mag_x_G", "mag_y_G", "mag_z_G",
        "baro_mbar", "baro_temp_c", "baro_alt_m",
        "tmp117_c",
        "bus_v", "current_ma", "power_mw",
    });
}

bool createLogFile(const char* prefix, char* name, size_t nameLen)
{
    // The SD test already mounted the card; begin() again in case it was skipped or failed
    bool ok = (sd.isMounted() || sd.begin()) &&
              sd.nextFreeName(prefix, LOG_SUFFIX, name, nameLen) &&
              logFile.open(name);
    if (!ok) {
        printf("SD card not available (%s), running without a log\n", sd.lastError());
        return false;
    }
    logFile.setSyncIntervalMs(1000);
    return true;
}

bool openLogFile(char* name, size_t nameLen)
{
    if (!createLogFile(LOG_PREFIX, name, nameLen)) return false;
    writeCsvHeader();
    return true;
}
