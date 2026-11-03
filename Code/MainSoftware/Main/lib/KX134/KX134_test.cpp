#include "KX134_test.h"
#include <stdio.h>
#include <math.h>

namespace {

bool g_verbose = false;
#define VLOG(...) do { if (g_verbose) printf(__VA_ARGS__); } while (0)

struct Checker {
    int pass = 0, fail = 0;
    bool operator()(const char* name, bool ok) {
        if (!ok || g_verbose) printf("[KX134] %s: %s\n", ok ? "PASS" : "FAIL", name);
        ok ? pass++ : fail++;
        return ok;
    }
};

bool waitFor(KX134& acc, uint32_t timeoutMs) {
    absolute_time_t deadline = make_timeout_time_ms(timeoutMs);
    while (!time_reached(deadline)) {
        if (acc.dataReady()) return true;
        sleep_us(100);
    }
    return false;
}

float averageMagnitude(KX134& acc, int n) {
    float sum = 0;
    for (int i = 0; i < n; i++) {
        waitFor(acc, 20);
        KX134::Data d;
        acc.read(d);
        acc.clearInterrupts();
        sum += sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
    }
    return sum / n;
}

} // namespace

bool KX134_test(KX134& acc, bool verbose, uint32_t samples) {
    g_verbose = verbose;
    Checker check;
    VLOG("\n===== KX134 test =====\n");

    // --- Communication ---
    if (!check("begin()", acc.begin())) {
        printf("[KX134] WHO_AM_I = 0x%02X (expected 0x%02X), COTR = 0x%02X, aborting\n",
               acc.whoAmI(), KX134::WHO_AM_I_VALUE, acc.readRegister(KX134::REG_COTR));
        return false;
    }
    check("WHO_AM_I == 0x46", acc.whoAmI() == KX134::WHO_AM_I_VALUE);
    check("device enabled after begin()", acc.isEnabled() && (acc.readRegister(KX134::REG_CNTL1) & 0x80));

    // --- Configuration readback ---
    acc.setODR(KX134::ODR::Hz400);
    check("ODCNTL ODR readback", (acc.readRegister(KX134::REG_ODCNTL) & 0x0F) == (uint8_t)KX134::ODR::Hz400);
    acc.setRange(KX134::Range::G16);
    check("CNTL1 GSEL readback", ((acc.readRegister(KX134::REG_CNTL1) >> 3) & 0x03) == (uint8_t)KX134::Range::G16);
    check("still enabled after settings change", acc.readRegister(KX134::REG_CNTL1) & 0x80);
    check("enableDataReadyEngine()", acc.enableDataReadyEngine(true));

    // --- Data ready + readings ---
    check("data ready within 20 ms", waitFor(acc, 20));
    float magSum = 0;
    for (uint32_t i = 0; i < samples; i++) {
        waitFor(acc, 20);
        KX134::Data d;
        if (!acc.read(d)) { check("read()", false); break; }
        acc.clearInterrupts();
        float mag = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
        magSum += mag;
        VLOG("  a = [%7.3f %7.3f %7.3f] g  |a| = %.3f\n", d.x, d.y, d.z, mag);
        sleep_ms(20);
    }
    float magAvg = magSum / samples;
    check("|accel| ~ 1 g while still (0.8 .. 1.2)", magAvg > 0.8f && magAvg < 1.2f);

    // --- Every range should read ~1 g ---
    const KX134::Range ranges[] = {KX134::Range::G8, KX134::Range::G16, KX134::Range::G32, KX134::Range::G64};
    const char* names[] = {"+-8 g", "+-16 g", "+-32 g", "+-64 g"};
    bool allOk = true;
    for (int i = 0; i < 4; i++) {
        acc.setRange(ranges[i]);
        sleep_ms(10);
        float m = averageMagnitude(acc, 8);
        VLOG("  %-7s |a| = %.3f g\n", names[i], m);
        allOk &= (m > 0.8f && m < 1.2f);
    }
    check("all ranges read ~1 g", allOk);

    // --- Interrupt pin ---
    if (acc.intPin() != KX134::NO_PIN) {
        acc.configureInterrupt1(true, true, true); // enabled, active high, latched
        acc.enableDataReadyInterrupt(true);
        acc.clearInterrupts();
        bool went = false;
        absolute_time_t deadline = make_timeout_time_ms(20);
        while (!time_reached(deadline)) { if (acc.interruptPinLevel()) { went = true; break; } }
        check("INT1 pin asserts on data ready", went);
        acc.enableDataReadyInterrupt(false);
        acc.clearInterrupts();
        sleep_us(100);
        check("INT1 pin releases after INT_REL read", !acc.interruptPinLevel());
    } else {
        VLOG("[KX134] SKIP: INT1 pin test (no pin given)\n");
    }

    // --- Sample buffer ---
    acc.setRange(KX134::Range::G16);
    acc.setODR(KX134::ODR::Hz1600);
    check("enableBuffer()", acc.enableBuffer(40, KX134::BufferMode::Stream));
    acc.clearBuffer();
    sleep_ms(30); // ~48 samples at 1600 Hz
    uint16_t count = acc.bufferSampleCount();
    VLOG("  buffer holds %u samples after 30 ms\n", count);
    check("buffer is filling", count >= 20);
    KX134::Data buf[86];
    uint16_t got = acc.readBuffer(buf, 86);
    bool bufOk = got > 0;
    for (uint16_t i = 0; i < got; i++) {
        float m = sqrtf(buf[i].x * buf[i].x + buf[i].y * buf[i].y + buf[i].z * buf[i].z);
        if (m < 0.7f || m > 1.3f) bufOk = false;
    }
    check("buffered samples read ~1 g", bufOk);
    acc.disableBuffer();

    // --- Restore defaults (begin() re-applies the stored range/ODR) ---
    acc.setRange(KX134::Range::G64);
    acc.setODR(KX134::ODR::Hz1600);
    acc.begin();

    printf("===== KX134: %d passed, %d failed =====\n", check.pass, check.fail);
    return check.fail == 0;
}
