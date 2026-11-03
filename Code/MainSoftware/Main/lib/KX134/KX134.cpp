#include "KX134.h"

static constexpr uint8_t SPI_READ = 0x80;

// CNTL1 bits
static constexpr uint8_t CNTL1_PC1   = 0x80;
static constexpr uint8_t CNTL1_RES   = 0x40;
static constexpr uint8_t CNTL1_DRDYE = 0x20;
static constexpr uint8_t CNTL1_GSEL  = 0x18;

KX134::KX134(spi_inst_t* spi, uint csPin, uint sckPin, uint mosiPin, uint misoPin,
             uint intPin, uint32_t spiHz)
    : spi_(spi), cs_(csPin), sck_(sckPin), mosi_(mosiPin), miso_(misoPin),
      intPin_(intPin), spiHz_(spiHz) {}

// ============================================================================
// Bus helpers
// ============================================================================

void KX134::select() {
    spi_set_baudrate(spi_, spiHz_);
    spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_put(cs_, 0);
}

void KX134::deselect() {
    gpio_put(cs_, 1);
}

bool KX134::readRegisters(uint8_t reg, uint8_t* buf, size_t len) {
    uint8_t cmd = reg | SPI_READ;
    select();
    spi_write_blocking(spi_, &cmd, 1);
    int n = spi_read_blocking(spi_, 0x00, buf, len);
    deselect();
    return n == (int)len;
}

uint8_t KX134::readRegister(uint8_t reg) {
    uint8_t v = 0;
    readRegisters(reg, &v, 1);
    return v;
}

bool KX134::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), value};
    select();
    int n = spi_write_blocking(spi_, tx, 2);
    deselect();
    return n == 2;
}

bool KX134::modifyRegister(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t v;
    if (!readRegisters(reg, &v, 1)) return false;
    v = (v & ~mask) | (value & mask);
    return writeRegister(reg, v);
}

template <typename F>
bool KX134::inStandby(F fn) {
    bool wasOn = isEnabled();
    if (wasOn && !enable(false)) return false;
    bool ok = fn();
    if (wasOn) ok &= enable(true);
    return ok;
}

// ============================================================================
// Setup
// ============================================================================

bool KX134::begin(bool initBus) {
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

    if (intPin_ != NO_PIN) {
        gpio_init(intPin_);
        gpio_set_dir(intPin_, GPIO_IN);
    }

    sleep_ms(50); // power-on time

    if (!reset()) return false;
    if (whoAmI() != WHO_AM_I_VALUE) return false;

    // Default config: standby, high-performance, ±64 g, 1600 Hz
    cntl1_ = readRegister(REG_CNTL1) & ~CNTL1_PC1;
    bool ok = writeRegister(REG_CNTL1, cntl1_);
    ok &= setHighPerformance(true);
    ok &= setRange(range_);
    ok &= setODR(odr_);
    ok &= enable(true);
    return ok;
}

bool KX134::reset() {
    // Kionix TN027 power-on / software reset procedure
    writeRegister(REG_CNTL1, 0x00); // PC1 = 0 required before SRST
    writeRegister(REG_INTERNAL_7F, 0x00);
    writeRegister(REG_CNTL2, 0x00);
    writeRegister(REG_CNTL2, 0x80); // SRST
    sleep_ms(3);

    // Wait for SRST to self-clear (max ~10 ms)
    for (int i = 0; i < 10; i++) {
        uint8_t c2 = readRegister(REG_CNTL2);
        if (!(c2 & 0x80)) break;
        sleep_ms(1);
    }

    cntl1_ = 0;
    if (readRegister(REG_COTR) != COTR_VALUE) return false;
    return true;
}

uint8_t KX134::whoAmI() {
    return readRegister(REG_WHO_AM_I);
}

bool KX134::isConnected() {
    return whoAmI() == WHO_AM_I_VALUE;
}

bool KX134::enable(bool on) {
    uint8_t v = on ? (cntl1_ | CNTL1_PC1) : (cntl1_ & ~CNTL1_PC1);
    if (!writeRegister(REG_CNTL1, v)) return false;
    cntl1_ = v;
    if (on) sleep_ms(2); // start-up; first samples may be invalid for ~1/ODR
    return true;
}

// ============================================================================
// Configuration
// ============================================================================

bool KX134::setRange(Range range) {
    return inStandby([&] {
        uint8_t v = (cntl1_ & ~CNTL1_GSEL) | ((uint8_t)range << 3);
        if (!writeRegister(REG_CNTL1, v)) return false;
        cntl1_ = v;
        range_ = range;
        return true;
    });
}

bool KX134::setODR(ODR odr) {
    return inStandby([&] {
        if (!modifyRegister(REG_ODCNTL, 0x0F, (uint8_t)odr)) return false;
        odr_ = odr;
        return true;
    });
}

bool KX134::setHighPerformance(bool hp) {
    return inStandby([&] {
        uint8_t v = hp ? (cntl1_ | CNTL1_RES) : (cntl1_ & ~CNTL1_RES);
        if (!writeRegister(REG_CNTL1, v)) return false;
        cntl1_ = v;
        return true;
    });
}

bool KX134::setFilter(bool bypassIIR, bool cornerOdrOver2) {
    return inStandby([&] {
        uint8_t v = (bypassIIR ? 0x80 : 0) | (cornerOdrOver2 ? 0x40 : 0);
        return modifyRegister(REG_ODCNTL, 0xC0, v);
    });
}

bool KX134::enableDataReadyEngine(bool enable) {
    return inStandby([&] {
        uint8_t v = enable ? (cntl1_ | CNTL1_DRDYE) : (cntl1_ & ~CNTL1_DRDYE);
        if (!writeRegister(REG_CNTL1, v)) return false;
        cntl1_ = v;
        return true;
    });
}

float KX134::scale() const {
    static constexpr float fs[] = {8.0f, 16.0f, 32.0f, 64.0f};
    return fs[(uint8_t)range_] / 32768.0f;
}

// ============================================================================
// Interrupts
// ============================================================================

bool KX134::configureInterrupt1(bool enablePin, bool activeHigh, bool latched) {
    return inStandby([&] {
        // INC1: bit5 IEN1, bit4 IEA1 (1 = active high), bit3 IEL1 (0 = latched, 1 = pulsed)
        uint8_t v = (enablePin ? 0x20 : 0) | (activeHigh ? 0x10 : 0) | (latched ? 0 : 0x08);
        return modifyRegister(REG_INC1, 0x38, v);
    });
}

bool KX134::setInterrupt1Sources(uint8_t mask) {
    return inStandby([&] { return writeRegister(REG_INC4, mask); });
}

bool KX134::enableDataReadyInterrupt(bool enable) {
    return inStandby([&] {
        uint8_t v = enable ? (cntl1_ | CNTL1_DRDYE) : (cntl1_ & ~CNTL1_DRDYE);
        if (!writeRegister(REG_CNTL1, v)) return false;
        cntl1_ = v;
        return modifyRegister(REG_INC4, INT_DATA_READY, enable ? INT_DATA_READY : 0);
    });
}

uint8_t KX134::getInterruptStatus() {
    return readRegister(REG_INS2);
}

void KX134::clearInterrupts() {
    readRegister(REG_INT_REL);
}

bool KX134::dataReady() {
    return (getInterruptStatus() & INT_DATA_READY) != 0;
}

bool KX134::interruptPinLevel() const {
    return intPin_ != NO_PIN && gpio_get(intPin_);
}

// ============================================================================
// Readings
// ============================================================================

bool KX134::readRaw(RawData& raw) {
    uint8_t b[6];
    if (!readRegisters(REG_XOUT_L, b, sizeof(b))) return false;
    raw.x = (int16_t)((b[1] << 8) | b[0]);
    raw.y = (int16_t)((b[3] << 8) | b[2]);
    raw.z = (int16_t)((b[5] << 8) | b[4]);
    return true;
}

bool KX134::read(Data& d) {
    return read(d.x, d.y, d.z);
}

bool KX134::read(float& x, float& y, float& z) {
    RawData r;
    if (!readRaw(r)) return false;
    const float s = scale();
    x = r.x * s - offset_[0];
    y = r.y * s - offset_[1];
    z = r.z * s - offset_[2];
    return true;
}

// ============================================================================
// Buffer
// ============================================================================

bool KX134::enableBuffer(uint8_t watermark, BufferMode mode, bool fullInterrupt) {
    if (watermark > 86) watermark = 86;
    bool ok = writeRegister(REG_BUF_CNTL1, watermark);
    // BUF_CNTL2: bit7 BUFE, bit6 BRES (1 = 16-bit), bit5 BFIE, bits1:0 BM
    uint8_t v = 0x80 | 0x40 | (fullInterrupt ? 0x20 : 0) | ((uint8_t)mode & 0x03);
    ok &= writeRegister(REG_BUF_CNTL2, v);
    return ok;
}

bool KX134::disableBuffer() {
    return modifyRegister(REG_BUF_CNTL2, 0x80, 0);
}

uint16_t KX134::bufferSampleCount() {
    uint8_t b[2];
    if (!readRegisters(REG_BUF_STATUS_1, b, 2)) return 0;
    uint16_t bytes = (uint16_t)(((b[1] & 0x03) << 8) | b[0]);
    return bytes / 6;
}

uint16_t KX134::readBuffer(Data* out, uint16_t maxSamples) {
    uint16_t n = bufferSampleCount();
    if (n > maxSamples) n = maxSamples;
    const float s = scale();
    for (uint16_t i = 0; i < n; i++) {
        uint8_t b[6];
        if (!readRegisters(REG_BUF_READ, b, 6)) return i;
        out[i].x = (int16_t)((b[1] << 8) | b[0]) * s - offset_[0];
        out[i].y = (int16_t)((b[3] << 8) | b[2]) * s - offset_[1];
        out[i].z = (int16_t)((b[5] << 8) | b[4]) * s - offset_[2];
    }
    return n;
}

bool KX134::clearBuffer() {
    return writeRegister(REG_BUF_CLEAR, 0x00);
}

// ============================================================================
// Calibration
// ============================================================================

void KX134::setOffset(float x, float y, float z) {
    offset_[0] = x; offset_[1] = y; offset_[2] = z;
}
