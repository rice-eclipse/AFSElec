#include "MMC5983MA.h"
#include <math.h>
#include "hardware/clocks.h"


// Internal Control 0 bits
static constexpr uint8_t CTRL0_TM_M      = 0x01;
static constexpr uint8_t CTRL0_TM_T      = 0x02;
static constexpr uint8_t CTRL0_INT_EN    = 0x04;
static constexpr uint8_t CTRL0_SET       = 0x08;
static constexpr uint8_t CTRL0_RESET     = 0x10;
static constexpr uint8_t CTRL0_AUTO_SR   = 0x20;
// Internal Control 1 bits
static constexpr uint8_t CTRL1_BW_MASK   = 0x03;
static constexpr uint8_t CTRL1_X_INHIBIT = 0x04;
static constexpr uint8_t CTRL1_YZ_INHIBIT= 0x18;
static constexpr uint8_t CTRL1_SW_RST    = 0x80;
// Internal Control 2 bits
static constexpr uint8_t CTRL2_CM_FREQ   = 0x07;
static constexpr uint8_t CTRL2_CMM_EN    = 0x08;
static constexpr uint8_t CTRL2_PRD_SET   = 0x70;
static constexpr uint8_t CTRL2_EN_PRD_SET= 0x80;

MMC5983MA::MMC5983MA(spi_inst_t* spi, uint csPin, uint sckPin, uint mosiPin, uint misoPin,
                     uint intPin, uint32_t spiHz)
    : spi_(spi), cs_(csPin), sck_(sckPin), mosi_(mosiPin), miso_(misoPin),
      intPin_(intPin), spiHz_(spiHz) {}

// ============================================================================
// Bus helpers
// ============================================================================

// The chip's I2C interface is live whenever CS is high (it has no I2C-disable bit) and listens on
// SCK (=SCL) / MOSI (=SDA). An I2C START is "SDA falls while SCL is high". The RP2350's SPI block
// changes MOSI on the same SCK falling edge, so any skew can look like a START to the chip, which
// can then "address" itself over I2C and stop answering SPI. Two defences:
//   * I2C guard: an I2C STOP right before every transaction ends any phantom I2C transaction.
//   * Bit-banged transfers (default): MOSI only changes a quarter period AFTER SCK has fallen,
//     so this driver's own traffic can never form a START.

void MMC5983MA::bbDelay() const {
    busy_wait_at_least_cycles(bbQuarterCycles_);
}

void MMC5983MA::bbTransfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    // SPI mode 0 timing: SCK idles low; data set while SCK is low, sampled on the rising edge.
    for (size_t i = 0; i < len; i++) {
        uint8_t out = tx ? tx[i] : 0x00;
        uint8_t in = 0;
        for (int bit = 7; bit >= 0; bit--) {
            bbDelay();                          // SCK has been low for a quarter period
            gpio_put(mosi_, (out >> bit) & 1);  // ...so this change can't look like START/STOP
            bbDelay();
            in = (uint8_t)((in << 1) | (gpio_get(miso_) ? 1 : 0)); // chip drove it on the previous falling edge
            gpio_put(sck_, 1);
            bbDelay(); bbDelay();
            gpio_put(sck_, 0);
        }
        if (rx) rx[i] = in;
    }
}

void MMC5983MA::select() {
    if (bitBang_) {
        const uint32_t sys = clock_get_hz(clk_sys);
        // Cap at 4 MHz so MISO is sampled well after the chip's 50 ns output-valid time
        const uint32_t hz = (spiHz_ == 0 || spiHz_ > 4'000'000) ? 4'000'000 : spiHz_;
        bbQuarterCycles_ = sys / (4 * hz) + 1;
        gpio_put(cs_, 1);
        takePins();                       // SCK low, MOSI low
        if (i2cGuard_) stopCondition();   // ends with SCK low
        bbDelay();
        gpio_put(cs_, 0);
        bbDelay();
        return;
    }
    spi_set_baudrate(spi_, spiHz_);
    if (i2cGuard_) sendI2CStop();
    if (spiMode3_) spi_set_format(spi_, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    else           spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_put(cs_, 0);
}

void MMC5983MA::deselect() {
    if (bitBang_) {
        bbDelay();
        gpio_put(cs_, 1);  // SCK is low here
        releasePins();
        return;
    }
    gpio_put(cs_, 1);
}

bool MMC5983MA::readRegisters(uint8_t reg, uint8_t* buf, size_t len) {
    uint8_t cmd = (reg & 0x3F) | readFlags_;
    select();
    if (bitBang_) {
        bbTransfer(&cmd, nullptr, 1);
        bbTransfer(nullptr, buf, len);
        deselect();
        return true;
    }
    spi_write_blocking(spi_, &cmd, 1);
    int n = spi_read_blocking(spi_, 0x00, buf, len);
    deselect();
    return n == (int)len;
}

uint8_t MMC5983MA::readRegister(uint8_t reg) {
    uint8_t v = 0;
    readRegisters(reg, &v, 1);
    return v;
}

bool MMC5983MA::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {(uint8_t)(reg & 0x3F), value};
    select();
    if (bitBang_) {
        bbTransfer(tx, nullptr, 2);
        deselect();
        return true;
    }
    int n = spi_write_blocking(spi_, tx, 2);
    deselect();
    return n == 2;
}

bool MMC5983MA::writeCtrl0(uint8_t oneShotBits) {
    // Self-clearing action bits are OR'd with the persistent (shadowed) bits
    return writeRegister(REG_CTRL0, ctrl0_ | oneShotBits);
}

bool MMC5983MA::waitStatus(uint8_t mask, uint32_t timeoutMs) {
    absolute_time_t deadline = make_timeout_time_ms(timeoutMs);
    while (!(readRegister(REG_STATUS) & mask)) {
        if (time_reached(deadline)) return false;
        sleep_us(100);
    }
    return true;
}

// ============================================================================
// Setup
// ============================================================================

bool MMC5983MA::begin(bool initBus) {
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

    sleep_ms(10); // power-up time

    // While CS is high the chip's I2C interface listens on SCK/MOSI, so traffic for other devices
    // on the shared bus can leave it mid-"I2C transaction" and deaf to SPI. Free it, then reset.
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; attempt++) {
        recoverInterface();
        reset();
        ok = idStable();
    }
    if (!ok) {
        if (!autoDetectBus()) return false;
        reset();
        if (!idStable()) return false;
    }

    // Clear any residual magnetization
    performSet();
    return true;
}

bool MMC5983MA::idStable(int reads) {
    for (int i = 0; i < reads; i++) {
        if (productId() != PRODUCT_ID_VALUE) return false;
    }
    return true;
}

// The I2C START condition is "SDA (MOSI) falls while SCL (SCK) is high". Everything below is ordered
// so MOSI only ever changes while SCK is LOW - except the deliberate low->high STOP edge.

void MMC5983MA::takePins() {
    // SPI mode 0 idles SCK low; take both lines over at their idle levels (SCK low, MOSI low).
    spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_put(sck_, 0);
    gpio_put(mosi_, 0);
    gpio_set_dir(sck_, GPIO_OUT);
    gpio_set_dir(mosi_, GPIO_OUT);
    gpio_set_function(sck_, GPIO_FUNC_SIO);
    gpio_set_function(mosi_, GPIO_FUNC_SIO);
}

void MMC5983MA::releasePins() {
    // SCK is low here. Give MOSI back first (it may change level - fine while SCK is low),
    // then SCK (the SPI block idles it low in mode 0, so it doesn't move).
    gpio_set_function(mosi_, GPIO_FUNC_SPI);
    gpio_set_function(sck_, GPIO_FUNC_SPI);
}

void MMC5983MA::stopCondition() {
    // Pins already taken, SCK low. MOSI low -> SCK high -> MOSI high (= STOP) -> SCK low. ~4 us.
    gpio_put(mosi_, 0);
    busy_wait_us_32(1);
    gpio_put(sck_, 1);
    busy_wait_us_32(1);
    gpio_put(mosi_, 1);   // STOP: ends any I2C transaction the chip thinks is in progress
    busy_wait_us_32(1);
    gpio_put(sck_, 0);
    busy_wait_us_32(1);
}

void MMC5983MA::sendI2CStop() {
    gpio_put(cs_, 1);
    takePins();
    stopCondition();
    releasePins();
}

void MMC5983MA::recoverInterface() {
    // 9 I2C clocks with SDA released (so a slave stuck driving SDA low can finish its byte),
    // then a STOP. Harmless to the other devices: their CS is high, and a STOP is the I2C idle state.
    const bool upMosi = gpio_is_pulled_up(mosi_), downMosi = gpio_is_pulled_down(mosi_);
    gpio_put(cs_, 1);
    takePins();

    // Release SDA: input with pull-up, like an open-drain I2C master (changes only while SCK is low)
    gpio_set_pulls(mosi_, true, false);
    gpio_set_dir(mosi_, GPIO_IN);
    sleep_us(10);
    for (int i = 0; i < 9; i++) {
        gpio_put(sck_, 1); sleep_us(5);
        gpio_put(sck_, 0); sleep_us(5);
    }

    // STOP: SDA driven low while SCL low, raise SCL, then raise SDA; finish with SCL low
    gpio_put(mosi_, 0);
    gpio_set_dir(mosi_, GPIO_OUT);
    sleep_us(5);
    gpio_put(sck_, 1); sleep_us(5);
    gpio_put(mosi_, 1); sleep_us(5);
    gpio_put(sck_, 0); sleep_us(5);

    gpio_set_pulls(mosi_, upMosi, downMosi);
    releasePins();
}

bool MMC5983MA::recover() {
    recoverInterface();
    if (idStable()) return true;

    // Still deaf: reset and restore the configuration from the shadow registers
    const uint8_t c0 = ctrl0_, c1 = ctrl1_, c2 = ctrl2_, c3 = ctrl3_;
    const Bandwidth bw = bw_;
    reset();
    if (!idStable()) return false;
    bool ok = writeRegister(REG_CTRL1, c1) && writeRegister(REG_CTRL3, c3) && writeRegister(REG_CTRL0, c0);
    ok = ok && writeRegister(REG_CTRL2, c2);
    ctrl0_ = c0; ctrl1_ = c1; ctrl2_ = c2; ctrl3_ = c3; bw_ = bw;
    performSet();
    return ok;
}

bool MMC5983MA::reset() {
    bool ok = writeRegister(REG_CTRL1, CTRL1_SW_RST);
    sleep_ms(20); // reset/power-on time ~10 ms
    ctrl0_ = ctrl1_ = ctrl2_ = ctrl3_ = 0;
    bw_ = Bandwidth::Hz100;
    return ok;
}

bool MMC5983MA::autoDetectBus() {
    // Mode 0 first (safer on a shared bus, see setSpiMode), then the datasheet's mode 3.
    const bool modes[] = {false, true};
    const uint32_t hzList[] = {spiHz_, 1'000'000};
    const bool savedMode3 = spiMode3_;
    const uint32_t savedHz = spiHz_;
    for (uint32_t hz : hzList) {
        for (bool m3 : modes) {
            spiHz_ = hz;
            spiMode3_ = m3;
            recoverInterface();
            if (idStable()) return true;
        }
    }
    spiMode3_ = savedMode3;
    spiHz_ = savedHz;
    return false;
}

uint8_t MMC5983MA::productId() {
    return readRegister(REG_PRODUCT_ID);
}

bool MMC5983MA::isConnected() {
    return productId() == PRODUCT_ID_VALUE;
}

// ============================================================================
// Configuration
// ============================================================================

bool MMC5983MA::setBandwidth(Bandwidth bw) {
    uint8_t v = (ctrl1_ & ~CTRL1_BW_MASK) | (uint8_t)bw;
    if (!writeRegister(REG_CTRL1, v)) return false;
    ctrl1_ = v;
    bw_ = bw;
    return true;
}

uint32_t MMC5983MA::measurementTimeUs() const {
    switch (bw_) {
        case Bandwidth::Hz100: return 8000;
        case Bandwidth::Hz200: return 4000;
        case Bandwidth::Hz400: return 2000;
        case Bandwidth::Hz800:
        default:               return 500;
    }
}

bool MMC5983MA::setChannelInhibit(bool x, bool yz) {
    uint8_t v = (ctrl1_ & ~(CTRL1_X_INHIBIT | CTRL1_YZ_INHIBIT)) |
                (x ? CTRL1_X_INHIBIT : 0) | (yz ? CTRL1_YZ_INHIBIT : 0);
    if (!writeRegister(REG_CTRL1, v)) return false;
    ctrl1_ = v;
    return true;
}

bool MMC5983MA::enableAutoSetReset(bool enable) {
    uint8_t v = enable ? (ctrl0_ | CTRL0_AUTO_SR) : (ctrl0_ & ~CTRL0_AUTO_SR);
    if (!writeRegister(REG_CTRL0, v)) return false;
    ctrl0_ = v;
    return true;
}

bool MMC5983MA::setPeriodicSet(bool enable, PeriodicSet period) {
    uint8_t v = (ctrl2_ & ~(CTRL2_PRD_SET | CTRL2_EN_PRD_SET)) |
                ((uint8_t)period << 4) | (enable ? CTRL2_EN_PRD_SET : 0);
    if (!writeRegister(REG_CTRL2, v)) return false;
    ctrl2_ = v;
    return true;
}

bool MMC5983MA::startContinuous(ContinuousRate rate) {
    if (rate == ContinuousRate::Off) return stopContinuous();
    uint8_t v = (ctrl2_ & ~(CTRL2_CM_FREQ | CTRL2_CMM_EN)) | (uint8_t)rate | CTRL2_CMM_EN;
    if (!writeRegister(REG_CTRL2, v)) return false;
    ctrl2_ = v;
    return true;
}

bool MMC5983MA::stopContinuous() {
    uint8_t v = ctrl2_ & ~(CTRL2_CM_FREQ | CTRL2_CMM_EN);
    if (!writeRegister(REG_CTRL2, v)) return false;
    ctrl2_ = v;
    return true;
}

// ============================================================================
// Set / Reset
// ============================================================================

bool MMC5983MA::performSet() {
    bool ok = writeCtrl0(CTRL0_SET);
    sleep_ms(1);
    return ok;
}

bool MMC5983MA::performReset() {
    bool ok = writeCtrl0(CTRL0_RESET);
    sleep_ms(1);
    return ok;
}

bool MMC5983MA::measureSetReset(RawData& afterSet, RawData& afterReset) {
    if (isContinuous()) return false;

    // Disable auto set/reset during the procedure
    const uint8_t savedCtrl0 = ctrl0_;
    ctrl0_ &= ~CTRL0_AUTO_SR;
    writeRegister(REG_CTRL0, ctrl0_);

    bool ok = performSet() && readSingleRaw(afterSet);
    ok = ok && performReset() && readSingleRaw(afterReset);

    ctrl0_ = savedCtrl0;
    writeRegister(REG_CTRL0, ctrl0_);
    performSet();
    return ok;
}

bool MMC5983MA::calibrateHardIron(uint32_t durationMs, float* fieldStrength) {
    if (isContinuous()) return false;
    // Measure without the old correction
    const float savedHard[3] = {hard_[0], hard_[1], hard_[2]};
    const float savedSoft[3] = {soft_[0], soft_[1], soft_[2]};
    for (int i = 0; i < 3; i++) { hard_[i] = 0; soft_[i] = 1; }

    float mn[3] = {1e9f, 1e9f, 1e9f}, mx[3] = {-1e9f, -1e9f, -1e9f};
    uint32_t n = 0;
    absolute_time_t end = make_timeout_time_ms(durationMs);
    while (!time_reached(end)) {
        Data d;
        if (!readSingle(d)) continue;
        const float v[3] = {d.x, d.y, d.z};
        for (int i = 0; i < 3; i++) { mn[i] = fminf(mn[i], v[i]); mx[i] = fmaxf(mx[i], v[i]); }
        n++;
    }

    float r[3];
    for (int i = 0; i < 3; i++) r[i] = (mx[i] - mn[i]) / 2.0f;
    if (n < 50 || r[0] < 0.1f || r[1] < 0.1f || r[2] < 0.1f) { // Earth's field is >= 0.25 G
        for (int i = 0; i < 3; i++) { hard_[i] = savedHard[i]; soft_[i] = savedSoft[i]; }
        return false;
    }
    const float avg = (r[0] + r[1] + r[2]) / 3.0f;
    for (int i = 0; i < 3; i++) {
        hard_[i] = (mx[i] + mn[i]) / 2.0f;
        soft_[i] = avg / r[i];
    }
    if (fieldStrength) *fieldStrength = avg;
    return true;
}

bool MMC5983MA::calibrateOffset() {
    RawData s, r;
    if (!measureSetReset(s, r)) return false;
    // After SET: +H + offset, after RESET: -H + offset (datasheet "Offset" section)
    offset_[0] = (((float)s.x + (float)r.x) / 2.0f - ZERO_FIELD_18BIT) / COUNTS_PER_GAUSS_18BIT;
    offset_[1] = (((float)s.y + (float)r.y) / 2.0f - ZERO_FIELD_18BIT) / COUNTS_PER_GAUSS_18BIT;
    offset_[2] = (((float)s.z + (float)r.z) / 2.0f - ZERO_FIELD_18BIT) / COUNTS_PER_GAUSS_18BIT;
    return true;
}

// ============================================================================
// Interrupts
// ============================================================================

bool MMC5983MA::enableInterrupt(bool enable) {
    uint8_t v = enable ? (ctrl0_ | CTRL0_INT_EN) : (ctrl0_ & ~CTRL0_INT_EN);
    if (!writeRegister(REG_CTRL0, v)) return false;
    ctrl0_ = v;
    return true;
}

bool MMC5983MA::clearInterrupt() {
    // Write 1 to clear Meas_M_Done / Meas_T_Done
    return writeRegister(REG_STATUS, STATUS_MEAS_M_DONE | STATUS_MEAS_T_DONE);
}

uint8_t MMC5983MA::getStatus() {
    return readRegister(REG_STATUS);
}

bool MMC5983MA::dataReady() {
    return (getStatus() & STATUS_MEAS_M_DONE) != 0;
}

bool MMC5983MA::interruptPinLevel() const {
    return intPin_ != NO_PIN && gpio_get(intPin_);
}

// ============================================================================
// Readings
// ============================================================================

bool MMC5983MA::startMeasurement() {
    clearInterrupt();
    return writeCtrl0(CTRL0_TM_M);
}

bool MMC5983MA::readRaw(RawData& raw) {
    uint8_t b[7];
    if (!readRegisters(REG_XOUT0, b, sizeof(b))) return false;
    raw.x = ((uint32_t)b[0] << 10) | ((uint32_t)b[1] << 2) | ((b[6] >> 6) & 0x03);
    raw.y = ((uint32_t)b[2] << 10) | ((uint32_t)b[3] << 2) | ((b[6] >> 4) & 0x03);
    raw.z = ((uint32_t)b[4] << 10) | ((uint32_t)b[5] << 2) | ((b[6] >> 2) & 0x03);
    return true;
}

void MMC5983MA::convert(const RawData& r, Data& d) const {
    const float k = 1.0f / COUNTS_PER_GAUSS_18BIT;
    d.x = (((float)r.x - ZERO_FIELD_18BIT) * k - offset_[0] - hard_[0]) * soft_[0];
    d.y = (((float)r.y - ZERO_FIELD_18BIT) * k - offset_[1] - hard_[1]) * soft_[1];
    d.z = (((float)r.z - ZERO_FIELD_18BIT) * k - offset_[2] - hard_[2]) * soft_[2];
}

bool MMC5983MA::read(Data& data) {
    RawData r;
    if (!readRaw(r)) return false;
    convert(r, data);
    return true;
}

bool MMC5983MA::readSingleRaw(RawData& raw, uint32_t timeoutMs) {
    if (!startMeasurement()) return false;
    sleep_us(measurementTimeUs());
    if (!waitStatus(STATUS_MEAS_M_DONE, timeoutMs)) return false;
    bool ok = readRaw(raw);
    clearInterrupt();
    return ok;
}

bool MMC5983MA::readSingle(Data& data, uint32_t timeoutMs) {
    RawData r;
    if (!readSingleRaw(r, timeoutMs)) return false;
    convert(r, data);
    return true;
}

float MMC5983MA::readTemperature(uint32_t timeoutMs) {
    if (isContinuous()) return NAN;
    clearInterrupt();
    if (!writeCtrl0(CTRL0_TM_T)) return NAN;
    if (!waitStatus(STATUS_MEAS_T_DONE, timeoutMs)) return NAN;
    uint8_t t = readRegister(REG_TOUT);
    clearInterrupt();
    return t * 0.8f - 75.0f;
}

float MMC5983MA::heading(const Data& d) {
    float h = atan2f(d.y, d.x) * 57.29578f;
    if (h < 0) h += 360.0f;
    return h;
}
