#include "INA219_test.h"
#include <stdio.h>
#include <math.h>

namespace {

bool g_verbose = false;
#define VLOG(...) do { if (g_verbose) printf(__VA_ARGS__); } while (0)

struct Checker {
    int pass = 0, fail = 0;
    bool operator()(const char* name, bool ok) {
        if (!ok || g_verbose) printf("[INA219] %s: %s\n", ok ? "PASS" : "FAIL", name);
        ok ? pass++ : fail++;
        return ok;
    }
};

} // namespace

bool INA219_test(INA219& ina, bool verbose, uint32_t samples) {
    g_verbose = verbose;
    Checker check;
    VLOG("\n===== INA219 test =====\n");

    // --- Communication ---
    if (!check("begin()", ina.begin())) {
        printf("[INA219] no ACK at address 0x%02X, aborting\n", ina.getAddress());
        return false;
    }

    // --- Configuration / calibration readback ---
    uint16_t cfg = 0, cal = 0;
    ina.readRegister(INA219::REG_CONFIG, cfg);
    ina.readRegister(INA219::REG_CALIBRATION, cal);
    VLOG("  config = 0x%04X, calibration = %u (expected %u), current LSB = %.3f mA, shunt = %.4f ohm\n",
           cfg, cal, ina.getCalibrationValue(), ina.getCurrentLSB() * 1000.0f, ina.getShuntOhms());
    check("config: 32 V range, 12-bit ADCs, shunt+bus continuous", (cfg & 0x27FF) == 0x2000 + (0x3 << 7) + (0x3 << 3) + 0x7);
    check("calibration register readback", cal == ina.getCalibrationValue() && cal != 0);

    // --- Readings ---
    float vSum = 0, iSum = 0, iShSum = 0, pSum = 0;
    for (uint32_t i = 0; i < samples; i++) {
        sleep_ms(2); // > 1.06 ms (12-bit shunt + bus)
        float vbus = ina.readBusVoltage();
        float vsh = ina.readShuntVoltage_mV();
        float cur = ina.readCurrent_mA();
        float curSh = ina.readCurrentFromShunt_mA();
        float pw = ina.readPower_mW();
        VLOG("  Vbus = %6.3f V  Vshunt = %8.3f mV  I = %9.2f mA (shunt calc %9.2f)  P = %9.2f mW\n",
               vbus, vsh, cur, curSh, pw);
        vSum += vbus; iSum += cur; iShSum += curSh; pSum += pw;
    }
    float v = vSum / samples, cur = iSum / samples, curSh = iShSum / samples, pw = pSum / samples;
    check("readings are valid numbers", !isnan(v) && !isnan(cur) && !isnan(pw));
    check("bus voltage in range (0 .. 26 V)", v >= 0.0f && v <= 26.0f);
    if (v < 0.5f) printf("[INA219] NOTE: bus voltage ~0 V - is the monitored rail powered?\n");
    check("overflow flag clear", !ina.overflow());

    // Current register vs. shunt-voltage calculation (2 % + 4 LSB tolerance)
    float tolI = fabsf(curSh) * 0.02f + 4.0f * ina.getCurrentLSB() * 1000.0f + 0.02f / ina.getShuntOhms();
    check("current register matches shunt voltage / R", fabsf(cur - curSh) <= tolI);

    // Power register vs. V * I (5 % + 2 power LSB tolerance)
    float pCalc = v * cur;
    float tolP = fabsf(pCalc) * 0.05f + 2.0f * ina.getPowerLSB() * 1000.0f;
    VLOG("  P register %.2f mW vs V*I %.2f mW\n", pw, pCalc);
    check("power register matches V * I", fabsf(pw - pCalc) <= tolP);

    // --- Triggered mode + ADC averaging timing (128 samples ~68 ms per channel) ---
    ina.setBusADC(INA219::ADC::Samples128);
    ina.setShuntADC(INA219::ADC::Bits12);
    ina.setMode(INA219::Mode::ShuntBusTriggered);
    sleep_ms(150);
    ina.readPowerRaw(); // clears CNVR
    ina.trigger();
    uint64_t t0 = time_us_64();
    bool ready = false;
    while (time_us_64() - t0 < 200000) {
        if (ina.conversionReady()) { ready = true; break; }
        sleep_us(500);
    }
    uint32_t dt = (uint32_t)(time_us_64() - t0);
    VLOG("  triggered conversion (bus x128, shunt 12-bit) took %lu us (expected ~69 ms)\n", (unsigned long)dt);
    check("triggered conversion completes", ready);
    check("conversion time reflects averaging setting (40 .. 100 ms)", ready && dt > 40000 && dt < 100000);

    // --- Restore defaults ---
    ina.begin();

    printf("===== INA219: %d passed, %d failed =====\n", check.pass, check.fail);
    return check.fail == 0;
}
