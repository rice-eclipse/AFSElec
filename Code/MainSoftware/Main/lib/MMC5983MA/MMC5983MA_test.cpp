#include "MMC5983MA_test.h"
#include <stdio.h>
#include <math.h>
#include <initializer_list>

namespace {

bool g_verbose = false;
#define VLOG(...) do { if (g_verbose) printf(__VA_ARGS__); } while (0)

struct Checker {
    int pass = 0, fail = 0;
    bool operator()(const char* name, bool ok) {
        if (!ok || g_verbose) printf("[MMC5983MA] %s: %s\n", ok ? "PASS" : "FAIL", name);
        ok ? pass++ : fail++;
        return ok;
    }
};

float magnitude(const MMC5983MA::Data& d) {
    return sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
}

/**
 * Run when begin() fails. Tells "chip never drives MISO" (hardware: power, CS, solder, footprint)
 * apart from "chip answers but the SPI speed/mode/read framing is wrong". Returns true if a working
 * combination was found (and leaves the driver set to it).
 */
bool diagnoseBus(MMC5983MA& mag) {
    const uint32_t savedHz = mag.getSpiFrequency();
    const uint8_t savedMode = mag.getSpiMode();
    const uint miso = mag.misoPin();
    printf("[MMC5983MA] --- SPI diagnostics ---\n");

    // 1. Is anything driving MISO? With a pull-up on MISO, an undriven line reads 0xFF;
    //    with the default pull-down it reads 0x00.
    const bool wasUp = gpio_is_pulled_up(miso), wasDown = gpio_is_pulled_down(miso);
    mag.setSpiFrequency(1'000'000);
    mag.setSpiMode(0);
    gpio_set_pulls(miso, true, false);
    sleep_us(50);
    uint8_t idUp = mag.readRegister(MMC5983MA::REG_PRODUCT_ID);
    gpio_set_pulls(miso, false, true);
    sleep_us(50);
    uint8_t idDown = mag.readRegister(MMC5983MA::REG_PRODUCT_ID);
    gpio_set_pulls(miso, wasUp, wasDown);
    printf("  product ID at 1 MHz mode 0: 0x%02X with MISO pull-up, 0x%02X with pull-down\n", idUp, idDown);
    if (idUp == 0xFF && idDown == 0x00) {
        printf("  -> MISO is never driven: the MMC5983MA is not responding at all.\n"
               "     Check 3.3 V on VDD (pin 2) and VDDIO (pin 13), that the chip-select line toggles\n"
               "     at pin 4, the solder joints, and that the LGA-16 footprint's pin 1 / pad numbering\n"
               "     matches the MMC5983MA datasheet.\n");
    }

    // 2. Sweep SPI mode x read framing x speed
    const uint32_t speeds[] = {500'000, 1'000'000, 4'000'000, 8'000'000};
    const uint8_t savedFlags = mag.getReadFlags();
    bool found = false;
    uint32_t goodHz = 0;
    uint8_t goodMode = 0, goodFlags = 0;
    for (uint8_t flags : {MMC5983MA::READ_FLAGS_SPARKFUN, MMC5983MA::READ_FLAGS_PX4}) {
        for (uint8_t mode : {0, 3}) {
            for (uint32_t hz : speeds) {
                mag.setSpiFrequency(hz);
                mag.setSpiMode(mode);
                mag.setReadFlags(flags);
                uint8_t id = mag.readRegister(MMC5983MA::REG_PRODUCT_ID);
                uint8_t st = mag.readRegister(MMC5983MA::REG_STATUS);
                printf("  read 0x%02X|reg, mode %u, %7lu Hz: product ID 0x%02X, status 0x%02X%s\n", flags, mode,
                       (unsigned long)hz, id, st, id == MMC5983MA::PRODUCT_ID_VALUE ? "  <-- OK" : "");
                if (id == MMC5983MA::PRODUCT_ID_VALUE && !found) {
                    found = true; goodHz = hz; goodMode = mode; goodFlags = flags;
                }
            }
        }
    }

    if (found) {
        printf("  -> works with read 0x%02X|reg, mode %u, %lu Hz.\n", goodFlags, goodMode, (unsigned long)goodHz);
        mag.setSpiFrequency(goodHz);
        mag.setSpiMode(goodMode);
        mag.setReadFlags(goodFlags);
    } else {
        printf("  -> no combination answered. If MISO shows activity on a scope/logic analyser while\n"
               "     MAG_CS is low, compare the waveform against the datasheet's SPI timing diagram.\n");
        mag.setSpiFrequency(savedHz);
        mag.setSpiMode(savedMode);
        mag.setReadFlags(savedFlags);
    }
    return found;
}

} // namespace

bool MMC5983MA_test(MMC5983MA& mag, bool verbose, uint32_t samples) {
    g_verbose = verbose;
    Checker check;
    VLOG("\n===== MMC5983MA test =====\n");

    // --- Communication ---
    if (!check("begin()", mag.begin())) {
        printf("[MMC5983MA] Product ID = 0x%02X (expected 0x%02X)\n", mag.productId(), MMC5983MA::PRODUCT_ID_VALUE);
        if (!diagnoseBus(mag) || !check("begin() with SPI settings found by diagnostics", mag.begin())) {
            printf("[MMC5983MA] aborting\n");
            printf("===== MMC5983MA: %d passed, %d failed =====\n", check.pass, check.fail);
            return false;
        }
    }
    check("Product ID == 0x30", mag.productId() == MMC5983MA::PRODUCT_ID_VALUE);
    VLOG("  SPI settings in use: read 0x%02X|reg, mode %u, %lu Hz\n", mag.getReadFlags(), mag.getSpiMode(),
         (unsigned long)mag.getSpiFrequency());

    // --- Link integrity: hardware SPI vs. bit-banged transfers (no other bus traffic) ---
    {
        auto idErrors = [&](int n) {
            int e = 0;
            for (int i = 0; i < n; i++) if (mag.productId() != MMC5983MA::PRODUCT_ID_VALUE) e++;
            return e;
        };
        mag.setBitBang(false);
        int eHw = idErrors(1000);
        mag.setBitBang(true);
        if (eHw) mag.recover();
        int eBb = idErrors(1000);
        if (g_verbose || eHw || eBb)
            printf("  product-ID errors in 1000 reads: %d with hardware SPI, %d bit-banged (driver default)\n", eHw, eBb);
        check("bit-banged link error-free (1000 product-ID reads)", eBb == 0);
        if (eBb) mag.recover();
    }

    // --- Temperature ---
    float t = mag.readTemperature();
    VLOG("  die temperature = %.1f C\n", t);
    check("temperature plausible (-10 .. 70 C)", t > -10.0f && t < 70.0f);

    // --- SET/RESET actually flips the sensor: after SET = +H + offset, after RESET = -H + offset ---
    {
        MMC5983MA::RawData sr{0, 0, 0}, rr{0, 0, 0};
        bool okSR = mag.measureSetReset(sr, rr);
        if (!okSR) printf("  measureSetReset() failed: a measurement did not complete (communication error?)\n");
        const float k = 1.0f / MMC5983MA::COUNTS_PER_GAUSS_18BIT;
        float hx = ((float)sr.x - (float)rr.x) / 2.0f * k;
        float hy = ((float)sr.y - (float)rr.y) / 2.0f * k;
        float hz = ((float)sr.z - (float)rr.z) / 2.0f * k;
        float hm = sqrtf(hx * hx + hy * hy + hz * hz);
        bool flipOk = okSR && hm > 0.1f && hm < 8.0f; // sensor range is +-8 G
        if (okSR && (!flipOk || g_verbose)) {
            printf("  raw after SET   = %6lu %6lu %6lu\n", (unsigned long)sr.x, (unsigned long)sr.y, (unsigned long)sr.z);
            printf("  raw after RESET = %6lu %6lu %6lu\n", (unsigned long)rr.x, (unsigned long)rr.y, (unsigned long)rr.z);
            printf("  field (SET-RESET)/2 = [%.4f %.4f %.4f] G  |H| = %.4f G\n", hx, hy, hz, hm);
        }
        check("SET/RESET flips the sensor (|(SET-RESET)/2| = 0.1 .. 8 G)", flipOk);
        if (okSR && hm < 0.02f)
            printf("[MMC5983MA] NOTE: SET and RESET readings are identical - the SET/RESET pulses are not\n"
                   "            reaching the sensor (CAP pin / C20, or the Control 0 writes are not landing).\n");
        if (okSR && hm > 0.7f)
            printf("[MMC5983MA] NOTE: |H| = %.2f G is stronger than Earth's field (0.25..0.65 G): something\n"
                   "            magnetic nearby (inductors, battery, connectors, magnets, laptop) adds a\n"
                   "            hard-iron offset. Use calibrateHardIron() while rotating the board.\n", hm);
    }

    // --- Offset calibration (SET/RESET) ---
    check("calibrateOffset()", mag.calibrateOffset());
    float ox, oy, oz;
    mag.getOffset(ox, oy, oz);
    VLOG("  bridge offset = [%.4f %.4f %.4f] G\n", ox, oy, oz);
    check("offset plausible (< 1 G per axis)", fabsf(ox) < 1 && fabsf(oy) < 1 && fabsf(oz) < 1);

    // --- One-shot readings ---
    float sum[3] = {0, 0, 0}, sq[3] = {0, 0, 0}, magSum = 0;
    uint32_t ok = 0;
    for (uint32_t i = 0; i < samples; i++) {
        MMC5983MA::Data d;
        if (!mag.readSingle(d)) continue;
        ok++;
        float m = magnitude(d);
        magSum += m;
        sum[0] += d.x; sum[1] += d.y; sum[2] += d.z;
        sq[0] += d.x * d.x; sq[1] += d.y * d.y; sq[2] += d.z * d.z;
        VLOG("  B = [%8.4f %8.4f %8.4f] G  |B| = %.4f G  heading = %5.1f deg\n",
               d.x, d.y, d.z, m, MMC5983MA::heading(d));
    }
    check("all one-shot reads completed", ok == samples);
    if (ok > 0) {
        float magAvg = magSum / ok;
        // Earth's field is 0.25..0.65 G, but board hard-iron (before calibrateHardIron()) can add more
        check("|B| present and within sensor range (0.1 .. 8 G)", magAvg > 0.1f && magAvg < 8.0f);
        float noise = 0;
        for (int a = 0; a < 3; a++) {
            float mean = sum[a] / ok;
            noise = fmaxf(noise, sqrtf(fmaxf(sq[a] / ok - mean * mean, 0)));
        }
        VLOG("  worst-axis noise (std dev) = %.2f mG\n", noise * 1000.0f);
        check("noise < 5 mG at BW 100 Hz", noise < 0.005f);
    }

    // --- Each bandwidth works ---
    const MMC5983MA::Bandwidth bws[] = {MMC5983MA::Bandwidth::Hz100, MMC5983MA::Bandwidth::Hz200,
                                        MMC5983MA::Bandwidth::Hz400, MMC5983MA::Bandwidth::Hz800};
    bool bwOk = true;
    for (auto bw : bws) {
        mag.setBandwidth(bw);
        MMC5983MA::Data d;
        bwOk &= mag.readSingle(d);
    }
    check("one-shot read at every bandwidth", bwOk);
    mag.setBandwidth(MMC5983MA::Bandwidth::Hz100);

    // --- Interrupt pin (one-shot measurement done) ---
    if (mag.intPin() != MMC5983MA::NO_PIN) {
        const uint pin = mag.intPin();
        // The GPIO block latches edges in its raw interrupt register even with IRQs disabled,
        // so a pulse of any length is caught, not just a level we happen to poll.
        auto rawEvents = [pin]() { return (io_bank0_hw->intr[pin / 8] >> (4 * (pin % 8))) & 0xFu; };
        constexpr uint32_t EDGE_RISE = 0x8u; // bit 3 of the per-GPIO nibble

        mag.enableInterrupt(true);
        mag.clearInterrupt();
        sleep_us(50);
        gpio_acknowledge_irq(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
        mag.startMeasurement();
        bool level = false;
        absolute_time_t deadline = make_timeout_time_ms(20);
        while (!time_reached(deadline)) { if (mag.interruptPinLevel()) { level = true; break; } }
        bool rose = (rawEvents() & EDGE_RISE) != 0;
        uint8_t st = mag.getStatus();

        // Continuous mode, as in SparkFun's interrupt example (auto set/reset on, read the data, then
        // clear the interrupt). Repeated with each GPIO pull setting: if the chip's "high" is weak or it
        // only releases the pin, the RP2350's default pull-down would hide every edge.
        const bool up0 = gpio_is_pulled_up(pin), down0 = gpio_is_pulled_down(pin);
        struct PullCase { const char* name; bool up, down; int edges, samples, highs; };
        PullCase cases[] = {{"pull-down", false, true, 0, 0, 0}, {"no pull", false, false, 0, 0, 0},
                            {"pull-up", true, false, 0, 0, 0}};
        mag.enableAutoSetReset(true);
        for (PullCase& c : cases) {
            gpio_set_pulls(pin, c.up, c.down);
            mag.clearInterrupt();
            sleep_us(200);
            gpio_acknowledge_irq(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
            mag.startContinuous(MMC5983MA::ContinuousRate::Hz100);
            absolute_time_t end = make_timeout_time_ms(300);
            while (!time_reached(end)) {
                if (rawEvents() & EDGE_RISE) { c.edges++; gpio_acknowledge_irq(pin, GPIO_IRQ_EDGE_RISE); }
                if (gpio_get(pin)) c.highs++;
                if (mag.dataReady()) {
                    MMC5983MA::Data d;
                    mag.read(d);          // reading the data clears Meas_M_Done
                    mag.clearInterrupt(); // writing 1 to Meas_M_Done clears/re-arms the interrupt
                    c.samples++;
                }
                sleep_us(200);
            }
            mag.stopContinuous();
        }
        gpio_set_pulls(pin, up0, down0);
        mag.enableAutoSetReset(false);

        int contEdges = 0;
        const char* workingPull = nullptr;
        for (const PullCase& c : cases) {
            if (c.edges > 0 && !workingPull) { workingPull = c.name; contEdges = c.edges; }
        }
        const bool intOk = level || rose || contEdges > 0;
        if (!intOk || g_verbose) {
            printf("  INT one-shot: level high = %d, rising edge = %d, status = 0x%02X\n", (int)level, (int)rose, st);
            for (const PullCase& c : cases)
                printf("  INT continuous (%-9s): %3d rising edges, %3d samples, pin high in %d polls\n",
                       c.name, c.edges, c.samples, c.highs);
        }
        check("INT pin signals measurement done (one-shot or continuous)", intOk);
        if (workingPull && !cases[0].edges)
            printf("[MMC5983MA] NOTE: INT edges only appear with %s on GPIO %u - the chip's INT high is weak or\n"
                   "            open-drain-like; configure that pull (or add an external pull-up).\n", workingPull, pin);
        if (!level && !rose && contEdges > 0)
            printf("[MMC5983MA] NOTE: INT only fires in continuous mode on this part - use a rising-edge IRQ there.\n");
        else if (rose && !level)
            printf("[MMC5983MA] NOTE: INT is a short pulse, not a level - use a rising-edge GPIO IRQ, not polling.\n");

        if (!rose && contEdges == 0) {
            // Does the chip drive the pin at all? With interrupts disabled the INT output is hi-Z, so a
            // pull-up reads 1; enabled and idle, the chip drives it low, so it reads 0.
            const bool up = gpio_is_pulled_up(pin), down = gpio_is_pulled_down(pin);
            mag.clearInterrupt();
            mag.enableInterrupt(false);
            gpio_set_pulls(pin, true, false);
            sleep_us(200);
            int lvlDisabled = gpio_get(pin);
            mag.enableInterrupt(true);
            sleep_us(200);
            int lvlEnabled = gpio_get(pin);
            gpio_set_pulls(pin, up, down);
            printf("  with GPIO pull-up: INT reads %d with interrupt disabled, %d enabled\n", lvlDisabled, lvlEnabled);
            if (lvlDisabled == 1 && lvlEnabled == 0)
                printf("  -> the chip drives INT (low when idle) but never raised it after the measurement.\n");
            else if (lvlDisabled == 1 && lvlEnabled == 1)
                printf("  -> INT never driven low either: pin not connected (check U3 pad 15, TP7, trace to GPIO %u).\n", pin);
            else
                printf("  -> GPIO %u is held low by something else.\n", pin);
        }
        mag.clearInterrupt();
        mag.enableInterrupt(false);
    } else {
        VLOG("[MMC5983MA] SKIP: INT pin test (no pin given)\n");
    }

    // --- Continuous mode rate (auto set/reset off, then on) ---
    auto measureRate = [&](bool autoSR) {
        mag.stopContinuous();
        mag.enableAutoSetReset(autoSR);
        mag.clearInterrupt();
        mag.startContinuous(MMC5983MA::ContinuousRate::Hz100);
        int events = 0;
        absolute_time_t end = make_timeout_time_ms(500);
        while (!time_reached(end)) {
            if (mag.dataReady()) {
                MMC5983MA::Data d;
                mag.read(d);
                mag.clearInterrupt();
                events++;
            }
            sleep_us(200);
        }
        mag.stopContinuous();
        return events;
    };
    int evOff = measureRate(false);
    int evOn = measureRate(true);
    if (g_verbose || evOff < 35 || evOff > 65)
        printf("  continuous 100 Hz: %d samples/500 ms with auto set/reset off, %d with it on (expected ~50)\n",
               evOff, evOn);
    check("continuous rate ~100 Hz with auto set/reset off (35 .. 65 samples)", evOff >= 35 && evOff <= 65);
    mag.enableAutoSetReset(false);

    // --- Shared-bus robustness: does other-device SPI traffic (CS high) knock the chip off the bus? ---
    {
        spi_inst_t* spi = mag.spiInstance();
        uint8_t junk[512];
        uint32_t lfsr = 0xACE1u;
        for (uint8_t& b2 : junk) { lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u); b2 = (uint8_t)lfsr; }

        auto trafficTest = [&]() {
            int lost = 0;
            const int rounds = 40;
            for (int r = 0; r < rounds; r++) {
                for (int burst = 0; burst < 4; burst++) {   // mimic other drivers: mode 0, re-applied each transaction
                    spi_set_baudrate(spi, 8'000'000);
                    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
                    spi_write_blocking(spi, junk, sizeof(junk));
                }
                if (mag.productId() != MMC5983MA::PRODUCT_ID_VALUE) lost++;
            }
            return lost;
        };

        // Without the guard: shows whether this board's bus actually upsets the chip (informational)
        mag.setI2CGuard(false);
        int lostOff = trafficTest();
        mag.setI2CGuard(true);
        if (lostOff) mag.recover();

        // With the guard (driver default): must not lose the chip
        int lostOn = trafficTest();
        if (g_verbose || lostOn || lostOff)
            printf("  other-device traffic, product ID wrong in N of 40 checks: %d without I2C guard, %d with it\n",
                   lostOff, lostOn);
        check("still answers after 80 kB of other-device bus traffic (I2C guard on)", lostOn == 0);
        if (lostOn) check("recover() restores communication", mag.recover());
    }

    // --- Restore defaults ---
    mag.begin();

    printf("===== MMC5983MA: %d passed, %d failed =====\n", check.pass, check.fail);
    return check.fail == 0;
}
