#include "MS5611_test.h"
#include <stdio.h>
#include <math.h>

namespace {

bool g_verbose = false;
#define VLOG(...) do { if (g_verbose) printf(__VA_ARGS__); } while (0)

struct Checker {
    int pass = 0, fail = 0;
    bool operator()(const char* name, bool ok) {
        if (!ok || g_verbose) printf("[MS5611] %s: %s\n", ok ? "PASS" : "FAIL", name);
        ok ? pass++ : fail++;
        return ok;
    }
};

} // namespace

bool MS5611_test(MS5611& baro, bool verbose, uint32_t samples) {
    g_verbose = verbose;
    Checker check;
    VLOG("\n===== MS5611 test =====\n");

    // --- Communication / PROM ---
    if (!check("begin() (reset + PROM CRC + first read)", baro.begin())) {
        printf("[MS5611] PROM:");
        for (int i = 0; i < 8; i++) printf(" %u", baro.getPROM(i));
        printf("  (all 0 or 65535 means no SPI response), aborting\n");
        return false;
    }
    VLOG("  PROM C1..C6 = %u %u %u %u %u %u\n", baro.getPROM(1), baro.getPROM(2), baro.getPROM(3),
           baro.getPROM(4), baro.getPROM(5), baro.getPROM(6));
    check("PROM CRC valid", baro.promValid());

    // --- Every OSR gives sane values ---
    const MS5611::OSR osrs[] = {MS5611::OSR::OSR256, MS5611::OSR::OSR512, MS5611::OSR::OSR1024,
                                MS5611::OSR::OSR2048, MS5611::OSR::OSR4096};
    const int osrVal[] = {256, 512, 1024, 2048, 4096};
    bool osrOk = true;
    for (int i = 0; i < 5; i++) {
        baro.setOversampling(osrs[i]);
        uint64_t t0 = time_us_64();
        bool ok = baro.read();
        uint32_t dt = (uint32_t)(time_us_64() - t0);
        VLOG("  OSR %4d: P = %8.2f mbar  T = %6.2f C  (%lu us)\n", osrVal[i], baro.getPressure(),
               baro.getTemperature(), (unsigned long)dt);
        osrOk &= ok && baro.getPressure() > 300.0f && baro.getPressure() < 1100.0f &&
                 baro.getTemperature() > -20.0f && baro.getTemperature() < 70.0f;
    }
    check("pressure (300..1100 mbar) and temperature (-20..70 C) plausible at every OSR", osrOk);

    // --- Noise at OSR 4096 ---
    baro.setOversampling(MS5611::OSR::OSR4096);
    double sum = 0, sq = 0;
    uint32_t n = 0;
    for (uint32_t i = 0; i < samples; i++) {
        if (!baro.read()) continue;
        double p = baro.getPressure();
        sum += p; sq += p * p; n++;
    }
    check("all blocking reads succeeded", n == samples);
    if (n > 1) {
        double mean = sum / n;
        double sd = sqrt(fmax(sq / n - mean * mean, 0.0));
        VLOG("  OSR 4096: mean %.3f mbar, std dev %.4f mbar (~%.2f m)\n", mean, sd, sd * 8.3);
        check("pressure noise < 0.1 mbar at OSR 4096", sd < 0.1);
    }

    // --- Altitude reference ---
    baro.calibrateAltitude(0.0f);
    baro.read();
    VLOG("  sea-level ref = %.2f mbar, altitude now = %.2f m\n", baro.getSeaLevelPressure(), baro.getAltitude());
    check("altitude ~0 m right after calibrateAltitude(0)", fabsf(baro.getAltitude()) < 2.0f);

    // --- Non-blocking state machine ---
    int updates = 0;
    absolute_time_t end = make_timeout_time_ms(1000);
    while (!time_reached(end)) {
        if (baro.update(10)) updates++;
        sleep_us(200);
    }
    VLOG("  update(10): %d results in 1 s at OSR 4096 (expected ~95)\n", updates);
    check("non-blocking update() rate (>= 70 Hz)", updates >= 70);
    check("non-blocking result plausible", baro.getPressure() > 300.0f && baro.getPressure() < 1100.0f);

    baro.setSeaLevelPressure(MS5611::SEA_LEVEL_MBAR);

    printf("===== MS5611: %d passed, %d failed =====\n", check.pass, check.fail);
    return check.fail == 0;
}
