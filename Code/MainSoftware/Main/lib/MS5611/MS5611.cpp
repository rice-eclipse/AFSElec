#include "MS5611.h"
#include <math.h>

MS5611::MS5611(spi_inst_t* spi, uint csPin, uint sckPin, uint mosiPin, uint misoPin, uint32_t spiHz)
    : spi_(spi), cs_(csPin), sck_(sckPin), mosi_(mosiPin), miso_(misoPin), spiHz_(spiHz) {}

// ============================================================================
// Bus helpers
// ============================================================================

void MS5611::select() {
    spi_set_baudrate(spi_, spiHz_);
    spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_put(cs_, 0);
}

void MS5611::deselect() {
    gpio_put(cs_, 1);
}

bool MS5611::command(uint8_t cmd) {
    select();
    int n = spi_write_blocking(spi_, &cmd, 1);
    deselect();
    return n == 1;
}

// ============================================================================
// Setup
// ============================================================================

bool MS5611::begin(bool initBus) {
    gpio_init(cs_);
    gpio_set_dir(cs_, GPIO_OUT);
    gpio_put(cs_, 1);

    if (initBus) {
        if (!(spi_get_const_hw(spi_)->cr1 & SPI_SSPCR1_SSE_BITS)) {
            spi_init(spi_, spiHz_);
        }
        gpio_set_function(sck_, GPIO_FUNC_SPI);
        gpio_set_function(mosi_, GPIO_FUNC_SPI);
        gpio_set_function(miso_, GPIO_FUNC_SPI);
    }

    sleep_ms(10);
    if (!reset()) return false;
    if (!readPROM()) return false;
    return read();
}

bool MS5611::reset() {
    bool ok = command(CMD_RESET);
    sleep_ms(3); // PROM reload time 2.8 ms
    state_ = State::Idle;
    haveTemp_ = false;
    return ok;
}

bool MS5611::readPROM() {
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t cmd = CMD_PROM_READ + (i << 1);
        uint8_t b[2];
        select();
        spi_write_blocking(spi_, &cmd, 1);
        spi_read_blocking(spi_, 0x00, b, 2);
        deselect();
        prom_[i] = (uint16_t)((b[0] << 8) | b[1]);
    }

    bool allZero = true, allOnes = true;
    for (uint8_t i = 1; i < 7; i++) {
        if (prom_[i] != 0x0000) allZero = false;
        if (prom_[i] != 0xFFFF) allOnes = false;
    }
    promValid_ = !allZero && !allOnes && (crc4(prom_) == (prom_[7] & 0x000F));
    return promValid_;
}

// CRC4 from TE application note AN520
uint8_t MS5611::crc4(const uint16_t promIn[8]) {
    uint16_t p[8];
    for (int i = 0; i < 8; i++) p[i] = promIn[i];
    p[7] &= 0xFF00; // CRC byte replaced by 0

    uint16_t rem = 0;
    for (int cnt = 0; cnt < 16; cnt++) {
        if (cnt & 1) rem ^= (uint16_t)(p[cnt >> 1] & 0x00FF);
        else         rem ^= (uint16_t)(p[cnt >> 1] >> 8);
        for (int bit = 8; bit > 0; bit--) {
            if (rem & 0x8000) rem = (uint16_t)((rem << 1) ^ 0x3000);
            else              rem = (uint16_t)(rem << 1);
        }
    }
    return (uint8_t)((rem >> 12) & 0x000F);
}

uint32_t MS5611::conversionTimeUs() const {
    // Datasheet max conversion times, plus margin
    switch (osr_) {
        case OSR::OSR256:  return 650;
        case OSR::OSR512:  return 1250;
        case OSR::OSR1024: return 2400;
        case OSR::OSR2048: return 4700;
        case OSR::OSR4096:
        default:           return 9300;
    }
}

// ============================================================================
// Conversions
// ============================================================================

bool MS5611::startConversion(bool pressure) {
    uint8_t cmd = (pressure ? CMD_CONV_D1 : CMD_CONV_D2) + (uint8_t)osr_;
    convTimeUs_ = conversionTimeUs();
    convStart_ = get_absolute_time();
    return command(cmd); // CS may be released during conversion
}

bool MS5611::conversionDone() const {
    return absolute_time_diff_us(convStart_, get_absolute_time()) >= (int64_t)convTimeUs_;
}

uint32_t MS5611::readADC() {
    uint8_t cmd = CMD_ADC_READ;
    uint8_t b[3];
    select();
    spi_write_blocking(spi_, &cmd, 1);
    spi_read_blocking(spi_, 0x00, b, 3);
    deselect();
    return ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2]; // 0 = read too early
}

bool MS5611::read() {
    if (!promValid_) return false;

    startConversion(false);
    sleep_us(convTimeUs_);
    uint32_t d2 = readADC();

    startConversion(true);
    sleep_us(convTimeUs_);
    uint32_t d1 = readADC();

    state_ = State::Idle;
    if (d1 == 0 || d2 == 0) return false;
    compute(d1, d2);
    haveTemp_ = true;
    return true;
}

bool MS5611::update(uint8_t temperatureEvery) {
    if (!promValid_) return false;
    if (temperatureEvery == 0) temperatureEvery = 1;

    switch (state_) {
        case State::Idle:
            if (!haveTemp_ || pressCount_ >= temperatureEvery) {
                pressCount_ = 0;
                startConversion(false);
                state_ = State::ConvTemp;
            } else {
                startConversion(true);
                state_ = State::ConvPress;
            }
            return false;

        case State::ConvTemp:
            if (!conversionDone()) return false;
            {
                uint32_t d2 = readADC();
                if (d2 != 0) { D2_ = d2; haveTemp_ = true; }
            }
            startConversion(true);
            state_ = State::ConvPress;
            return false;

        case State::ConvPress:
            if (!conversionDone()) return false;
            {
                uint32_t d1 = readADC();
                state_ = State::Idle;
                pressCount_++;
                if (d1 == 0 || !haveTemp_) return false;
                compute(d1, D2_);
            }
            return true;
    }
    return false;
}

void MS5611::compute(uint32_t D1, uint32_t D2) {
    D1_ = D1;
    D2_ = D2;

    const int64_t C1 = prom_[1], C2 = prom_[2], C3 = prom_[3];
    const int64_t C4 = prom_[4], C5 = prom_[5], C6 = prom_[6];

    int64_t dT = (int64_t)D2 - C5 * 256;
    int64_t TEMP = 2000 + (dT * C6) / 8388608;          // 2^23
    int64_t OFF  = C2 * 65536 + (C4 * dT) / 128;        // 2^16, 2^7
    int64_t SENS = C1 * 32768 + (C3 * dT) / 256;        // 2^15, 2^8

    if (secondOrder_) {
        int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
        if (TEMP < 2000) {
            int64_t t = TEMP - 2000;
            T2 = (dT * dT) / 2147483648LL;              // 2^31
            OFF2 = 5 * t * t / 2;
            SENS2 = 5 * t * t / 4;
            if (TEMP < -1500) {
                int64_t t2 = TEMP + 1500;
                OFF2 += 7 * t2 * t2;
                SENS2 += 11 * t2 * t2 / 2;
            }
        }
        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    int64_t P = (((int64_t)D1 * SENS) / 2097152 - OFF) / 32768; // 2^21, 2^15

    temperature_ = (int32_t)TEMP;
    pressure_ = (int32_t)P;
}

// ============================================================================
// Altitude
// ============================================================================

float MS5611::pressureToAltitude(float mbar, float seaLevelMbar) {
    return 44330.0f * (1.0f - powf(mbar / seaLevelMbar, 0.1902949f));
}

float MS5611::getAltitude() const {
    return pressureToAltitude(getPressure(), seaLevel_);
}

void MS5611::calibrateAltitude(float altitudeM) {
    seaLevel_ = getPressure() / powf(1.0f - altitudeM / 44330.0f, 5.255f);
}
