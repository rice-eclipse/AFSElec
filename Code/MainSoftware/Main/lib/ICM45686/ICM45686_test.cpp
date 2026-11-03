#include "ICM45686_test.h"
#include <stdio.h>
#include <math.h>

namespace {

bool g_verbose = false;
#define VLOG(...) do { if (g_verbose) printf(__VA_ARGS__); } while (0)

struct Checker {
    int pass = 0, fail = 0;
    bool operator()(const char* name, bool ok) {
        if (!ok || g_verbose) printf("[ICM45686] %s: %s\n", ok ? "PASS" : "FAIL", name);
        ok ? pass++ : fail++;
        return ok;
    }
};

bool waitFor(ICM45686& imu, uint32_t timeoutMs) {
    absolute_time_t deadline = make_timeout_time_ms(timeoutMs);
    while (!time_reached(deadline)) {
        if (imu.dataReady()) return true;
        sleep_us(100);
    }
    return false;
}

} // namespace

bool ICM45686_test(ICM45686& imu, bool verbose, uint32_t samples) {
    g_verbose = verbose;
    Checker check;
    VLOG("\n===== ICM45686 test =====\n");

    // --- Communication ---
    if (!check("begin()", imu.begin())) {
        printf("[ICM45686] WHO_AM_I = 0x%02X (expected 0x%02X), aborting\n", imu.whoAmI(), ICM45686::WHO_AM_I_VALUE);
        return false;
    }
    check("WHO_AM_I == 0xE9", imu.whoAmI() == ICM45686::WHO_AM_I_VALUE);

    // --- Configuration readback ---
    imu.setAccelRange(ICM45686::AccelRange::G8);
    imu.setAccelODR(ICM45686::ODR::Hz400);
    check("ACCEL_CONFIG0 readback", imu.readRegister(ICM45686::REG_ACCEL_CONFIG0) ==
          (((uint8_t)ICM45686::AccelRange::G8 << 4) | (uint8_t)ICM45686::ODR::Hz400));

    imu.setGyroRange(ICM45686::GyroRange::DPS500);
    imu.setGyroODR(ICM45686::ODR::Hz400);
    check("GYRO_CONFIG0 readback", imu.readRegister(ICM45686::REG_GYRO_CONFIG0) ==
          (((uint8_t)ICM45686::GyroRange::DPS500 << 4) | (uint8_t)ICM45686::ODR::Hz400));

    check("PWR_MGMT0 = accel LN + gyro LN", (imu.readRegister(ICM45686::REG_PWR_MGMT0) & 0x0F) == 0x0F);
    check("set accel/gyro LPF (indirect write)",
          imu.setAccelLPF(ICM45686::LPF::ODR_4) && imu.setGyroLPF(ICM45686::LPF::ODR_4));
    uint8_t lpf = 0xFF;
    check("accel LPF indirect readback",
          imu.readIndirect(ICM45686::IREG_ACCEL_UI_LPF, lpf) && (lpf & 0x07) == (uint8_t)ICM45686::LPF::ODR_4);
    sleep_ms(50);

    // --- Data ready ---
    check("data ready within 20 ms", waitFor(imu, 20));

    // --- Readings ---
    float magSum = 0, tSum = 0;
    for (uint32_t i = 0; i < samples; i++) {
        waitFor(imu, 20);
        ICM45686::Data d;
        if (!imu.read(d)) { check("read()", false); break; }
        float mag = sqrtf(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
        magSum += mag;
        tSum += d.temperature;
        VLOG("  a = [%7.3f %7.3f %7.3f] g |a| = %.3f   g = [%8.2f %8.2f %8.2f] dps   T = %.2f C\n",
               d.ax, d.ay, d.az, mag, d.gx, d.gy, d.gz, d.temperature);
        sleep_ms(20);
    }
    float magAvg = magSum / samples, tAvg = tSum / samples;
    check("|accel| ~ 1 g while still (0.8 .. 1.2)", magAvg > 0.8f && magAvg < 1.2f);
    check("temperature plausible (0 .. 70 C)", tAvg > 0.0f && tAvg < 70.0f);

    // --- Range scaling consistency: |a| should not depend on range ---
    imu.setAccelRange(ICM45686::AccelRange::G32);
    sleep_ms(20);
    waitFor(imu, 20);
    float ax, ay, az;
    imu.readAccel(ax, ay, az);
    float mag32 = sqrtf(ax * ax + ay * ay + az * az);
    VLOG("  |a| at +-32 g = %.3f g (vs %.3f at +-8 g)\n", mag32, magAvg);
    check("range change keeps scaling consistent", fabsf(mag32 - magAvg) < 0.1f);

    // --- Gyro calibration ---
    check("calibrateGyro()", imu.calibrateGyro(200, 2500));
    float gx, gy, gz;
    imu.readGyro(gx, gy, gz);
    VLOG("  gyro after cal = [%.3f %.3f %.3f] dps\n", gx, gy, gz);
    check("gyro near zero after calibration (< 2 dps)", fabsf(gx) < 2 && fabsf(gy) < 2 && fabsf(gz) < 2);
    imu.setGyroOffset(0, 0, 0);

    // --- Interrupt pin ---
    if (imu.intPin() != ICM45686::NO_PIN) {
        imu.configureInterrupt1(true, true, true); // active high, latched, push-pull
        imu.enableDataReadyInterrupt(true);
        imu.getInterruptStatus(); // clear
        bool went = false;
        absolute_time_t deadline = make_timeout_time_ms(20);
        while (!time_reached(deadline)) { if (imu.interruptPinLevel()) { went = true; break; } }
        check("INT1 pin asserts on data ready", went);
        imu.getInterruptStatus(); // clear latch
        sleep_us(50);
        imu.enableDataReadyInterrupt(false);
    } else {
        VLOG("[ICM45686] SKIP: INT1 pin test (no pin given)\n");
    }

    // --- Restore defaults (begin() re-applies the stored ranges/ODRs) ---
    imu.setAccelRange(ICM45686::AccelRange::G16);
    imu.setGyroRange(ICM45686::GyroRange::DPS2000);
    imu.setAccelODR(ICM45686::ODR::Hz1600);
    imu.setGyroODR(ICM45686::ODR::Hz1600);
    imu.begin();

    printf("===== ICM45686: %d passed, %d failed =====\n", check.pass, check.fail);
    return check.fail == 0;
}
