#include "TMP117_test.h"
#include <stdio.h>
#include <math.h>

namespace {

bool g_verbose = false;
#define VLOG(...) do { if (g_verbose) printf(__VA_ARGS__); } while (0)

struct Checker {
    int pass = 0, fail = 0;
    bool operator()(const char* name, bool ok) {
        if (!ok || g_verbose) printf("[TMP117] %s: %s\n", ok ? "PASS" : "FAIL", name);
        ok ? pass++ : fail++;
        return ok;
    }
};

bool waitReady(TMP117& tmp, uint32_t timeoutMs) {
    absolute_time_t deadline = make_timeout_time_ms(timeoutMs);
    while (!time_reached(deadline)) {
        if (tmp.dataReady()) return true;
        sleep_ms(5);
    }
    return false;
}

} // namespace

bool TMP117_test(TMP117& tmp, bool verbose, uint32_t samples) {
    g_verbose = verbose;
    Checker check;
    VLOG("\n===== TMP117 test =====\n");

    // --- Communication ---
    if (!check("begin()", tmp.begin())) {
        printf("[TMP117] Device ID = 0x%03X (expected 0x117) at address 0x%02X, aborting\n",
               tmp.deviceId(), tmp.getAddress());
        return false;
    }
    VLOG("  device ID 0x%03X, revision %u\n", tmp.deviceId(), tmp.revision());
    check("device ID == 0x117", tmp.deviceId() == TMP117::DEVICE_ID_VALUE);

    // --- Configuration readback ---
    tmp.setAveraging(TMP117::Averaging::AVG8);
    tmp.setConversionCycle(TMP117::ConversionCycle::C0);
    tmp.setMode(TMP117::Mode::Continuous);
    check("averaging readback", tmp.getAveraging() == TMP117::Averaging::AVG8);
    check("conversion cycle readback", tmp.getConversionCycle() == TMP117::ConversionCycle::C0);
    check("mode readback", tmp.getMode() == TMP117::Mode::Continuous);

    // --- Continuous readings (AVG8 + C0 = 125 ms cycle) ---
    tmp.readConfig(); // clear stale flag
    check("data ready within 300 ms", waitReady(tmp, 300));
    float sum = 0;
    uint32_t n = 0;
    for (uint32_t i = 0; i < samples; i++) {
        if (!waitReady(tmp, 300)) continue;
        float t = tmp.readTemperature();
        VLOG("  T = %.4f C\n", t);
        sum += t; n++;
    }
    check("all continuous reads completed", n == samples);
    float tAvg = n ? sum / n : NAN;
    check("temperature plausible (0 .. 60 C)", tAvg > 0.0f && tAvg < 60.0f);

    // --- One-shot ---
    float t1 = tmp.readTemperatureOneShot();
    VLOG("  one-shot T = %.4f C\n", t1);
    check("one-shot conversion", !isnan(t1) && fabsf(t1 - tAvg) < 1.0f);
    // After a one-shot the device idles in shutdown (the MOD bits may still read back as one-shot),
    // so no new result should appear.
    tmp.readConfig(); // clear data-ready
    sleep_ms(300);    // > 2 conversion cycles at AVG8
    check("no further conversions after one-shot", !tmp.dataReady());

    // --- Offset register ---
    check("setOffset(+5 C)", tmp.setOffset(5.0f));
    check("offset readback", fabsf(tmp.getOffset() - 5.0f) < TMP117::RESOLUTION);
    float t2 = tmp.readTemperatureOneShot();
    VLOG("  with +5 C offset: T = %.4f C (delta %.3f)\n", t2, t2 - t1);
    check("offset applied to result (delta 5 +- 0.5 C)", fabsf((t2 - t1) - 5.0f) < 0.5f);
    tmp.setOffset(0.0f);

    // --- Alert limits ---
    tmp.setHighLimit(42.5f);
    tmp.setLowLimit(-10.25f);
    check("high limit readback", fabsf(tmp.getHighLimit() - 42.5f) < TMP117::RESOLUTION);
    check("low limit readback", fabsf(tmp.getLowLimit() + 10.25f) < TMP117::RESOLUTION);

    // Force a high alert: limit below current temperature
    tmp.setHighLimit(tAvg - 10.0f);
    tmp.setLowLimit(tAvg - 20.0f);
    // Reading the config register clears the alert flags, so don't poll dataReady() here:
    // wait out the conversion and read the flags once.
    tmp.readConfig();
    tmp.triggerOneShot();
    sleep_ms(200);
    bool hi = false, lo = false;
    tmp.alertStatus(hi, lo);
    check("high alert flag sets when T > high limit", hi && !lo);

    // --- Restore power-on defaults (does not touch EEPROM) ---
    tmp.reset();

    printf("===== TMP117: %d passed, %d failed =====\n", check.pass, check.fail);
    return check.fail == 0;
}
