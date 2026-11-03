#include "ICM45686.h"
#include <math.h>

static constexpr uint8_t SPI_READ = 0x80;

ICM45686::ICM45686(spi_inst_t* spi, uint csPin, uint sckPin, uint mosiPin, uint misoPin,
                   uint intPin, uint32_t spiHz)
    : spi_(spi), cs_(csPin), sck_(sckPin), mosi_(mosiPin), miso_(misoPin),
      intPin_(intPin), spiHz_(spiHz) {}

// ============================================================================
// Bus helpers
// ============================================================================

void ICM45686::select() {
    // The bus may be shared with devices using other speeds/modes, so re-apply ours every time.
    spi_set_baudrate(spi_, spiHz_);
    spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_put(cs_, 0);
}

void ICM45686::deselect() {
    gpio_put(cs_, 1);
}

bool ICM45686::readRegisters(uint8_t reg, uint8_t* buf, size_t len) {
    uint8_t cmd = reg | SPI_READ;
    select();
    spi_write_blocking(spi_, &cmd, 1);
    int n = spi_read_blocking(spi_, 0x00, buf, len);
    deselect();
    return n == (int)len;
}

uint8_t ICM45686::readRegister(uint8_t reg) {
    uint8_t v = 0;
    readRegisters(reg, &v, 1);
    return v;
}

bool ICM45686::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), value};
    select();
    int n = spi_write_blocking(spi_, tx, 2);
    deselect();
    return n == 2;
}

bool ICM45686::modifyRegister(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t v;
    if (!readRegisters(reg, &v, 1)) return false;
    v = (v & ~mask) | (value & mask);
    return writeRegister(reg, v);
}

// Indirect register access (IPREG banks), per TDK inv_imu_transport.c
bool ICM45686::readIndirect(uint16_t addr, uint8_t& value) {
    uint8_t tx[3] = {(uint8_t)(REG_IREG_ADDR_15_8 & 0x7F), (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF)};
    sleep_us(4);
    select();
    int n = spi_write_blocking(spi_, tx, 3);
    deselect();
    if (n != 3) return false;
    sleep_us(4);
    return readRegisters(REG_IREG_DATA, &value, 1);
}

bool ICM45686::writeIndirect(uint16_t addr, uint8_t value) {
    // Burst: IREG_ADDR_15_8, IREG_ADDR_7_0, IREG_DATA
    uint8_t tx[4] = {(uint8_t)(REG_IREG_ADDR_15_8 & 0x7F), (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF), value};
    sleep_us(4);
    select();
    int n = spi_write_blocking(spi_, tx, 4);
    deselect();
    sleep_us(4);
    return n == 4;
}

bool ICM45686::modifyIndirect(uint16_t addr, uint8_t mask, uint8_t value) {
    uint8_t v;
    if (!readIndirect(addr, v)) return false;
    v = (v & ~mask) | (value & mask);
    bool ok = writeIndirect(addr, v);
    sleep_ms(1); // indirect writes need time to take effect before the next one
    return ok;
}

int16_t ICM45686::toInt16(const uint8_t* p) const {
    return bigEndian_ ? (int16_t)((p[0] << 8) | p[1]) : (int16_t)((p[1] << 8) | p[0]);
}

// ============================================================================
// Setup
// ============================================================================

bool ICM45686::begin(bool initBus) {
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

    sleep_ms(3); // power-up time

    // A dummy read puts the interface into SPI mode
    whoAmI();

    if (!reset()) return false;
    if (whoAmI() != WHO_AM_I_VALUE) return false;

    // Determine data endianness (little endian by default)
    uint8_t sreg = 0;
    if (readIndirect(IREG_SREG_CTRL, sreg)) bigEndian_ = (sreg & 0x02) != 0;

    bool ok = true;
    ok &= setAccelRange(accelRange_);
    ok &= setGyroRange(gyroRange_);
    ok &= setAccelODR(accelODR_);
    ok &= setGyroODR(gyroODR_);
    // The DRDY status bit is only set when its INT1 status-enable bit is set, so turn it on
    // so that dataReady() polling works. (INT1 then pulses on each new sample.)
    ok &= enableDataReadyInterrupt(true);
    ok &= setAccelMode(AccelMode::LowNoise);
    ok &= setGyroMode(GyroMode::LowNoise);
    sleep_ms(30); // gyro start-up time
    return ok;
}

bool ICM45686::reset() {
    // Slew rate must be set for reliable SPI readout; it and INTF_CONFIG1_OVRD are restored after reset
    setSpiSlewRate(2);
    uint8_t intfOvrd = readRegister(REG_INTF_CONFIG1_OVRD);
    uint8_t drive0 = readRegister(REG_DRIVE_CONFIG0);

    if (!writeRegister(REG_MISC2, 0x02)) return false; // SOFT_RST
    sleep_ms(2);

    writeRegister(REG_DRIVE_CONFIG0, drive0);
    writeRegister(REG_INTF_CONFIG1_OVRD, intfOvrd);

    uint8_t status = readRegister(REG_INT1_STATUS0); // clears RESET_DONE
    return (status & INT_RESET_DONE) != 0;
}

uint8_t ICM45686::whoAmI() {
    return readRegister(REG_WHO_AM_I);
}

bool ICM45686::isConnected() {
    return whoAmI() == WHO_AM_I_VALUE;
}

// ============================================================================
// Configuration
// ============================================================================

bool ICM45686::setAccelRange(AccelRange range) {
    if (!modifyRegister(REG_ACCEL_CONFIG0, 0x70, (uint8_t)range << 4)) return false;
    accelRange_ = range;
    return true;
}

bool ICM45686::setGyroRange(GyroRange range) {
    if (!modifyRegister(REG_GYRO_CONFIG0, 0xF0, (uint8_t)range << 4)) return false;
    gyroRange_ = range;
    return true;
}

bool ICM45686::setAccelODR(ODR odr) {
    if (!modifyRegister(REG_ACCEL_CONFIG0, 0x0F, (uint8_t)odr)) return false;
    accelODR_ = odr;
    return true;
}

bool ICM45686::setGyroODR(ODR odr) {
    if (!modifyRegister(REG_GYRO_CONFIG0, 0x0F, (uint8_t)odr)) return false;
    gyroODR_ = odr;
    return true;
}

bool ICM45686::setAccelMode(AccelMode mode) {
    bool ok = modifyRegister(REG_PWR_MGMT0, 0x03, (uint8_t)mode);
    sleep_us(200); // no register writes for 200 us after a power mode change
    return ok;
}

bool ICM45686::setGyroMode(GyroMode mode) {
    bool ok = modifyRegister(REG_PWR_MGMT0, 0x0C, (uint8_t)mode << 2);
    sleep_us(200);
    return ok;
}

bool ICM45686::setAccelLPF(LPF bw) {
    return modifyIndirect(IREG_ACCEL_UI_LPF, 0x07, (uint8_t)bw);
}

bool ICM45686::setGyroLPF(LPF bw) {
    return modifyIndirect(IREG_GYRO_UI_LPF, 0x07, (uint8_t)bw);
}

bool ICM45686::setAntiAliasFilter(bool accelEnable, bool gyroEnable) {
    // src_ctrl: 0 = interpolator off / FIR off, 1 = interpolator off / FIR on
    bool ok = modifyIndirect(IREG_GYRO_SRC_CTRL, 0x60, (gyroEnable ? 1 : 0) << 5);
    ok &= modifyIndirect(IREG_ACCEL_SRC_CTRL, 0x03, accelEnable ? 1 : 0);
    return ok;
}

bool ICM45686::setSpiSlewRate(uint8_t slew) {
    if (slew > 6) slew = 6;
    bool ok = modifyRegister(REG_DRIVE_CONFIG0, 0x0E, slew << 1);
    sleep_us(2);
    return ok;
}

float ICM45686::accelScale() const {
    static constexpr float fs[] = {32.0f, 16.0f, 8.0f, 4.0f, 2.0f};
    return fs[(uint8_t)accelRange_] / 32768.0f;
}

float ICM45686::gyroScale() const {
    static constexpr float fs[] = {4000.0f, 2000.0f, 1000.0f, 500.0f, 250.0f,
                                   125.0f, 62.5f, 31.25f, 15.625f};
    return fs[(uint8_t)gyroRange_] / 32768.0f;
}

// ============================================================================
// Interrupts
// ============================================================================

bool ICM45686::configureInterrupt1(bool activeHigh, bool latched, bool pushPull) {
    uint8_t v = (activeHigh ? 0x01 : 0) | (latched ? 0x02 : 0) | (pushPull ? 0 : 0x04);
    return modifyRegister(REG_INT1_CONFIG2, 0x07, v);
}

bool ICM45686::setInterrupt1Sources(uint8_t mask) {
    return writeRegister(REG_INT1_CONFIG0, mask);
}

bool ICM45686::enableDataReadyInterrupt(bool enable) {
    return modifyRegister(REG_INT1_CONFIG0, INT_DRDY, enable ? INT_DRDY : 0);
}

uint8_t ICM45686::getInterruptStatus() {
    return readRegister(REG_INT1_STATUS0);
}

bool ICM45686::dataReady() {
    return (getInterruptStatus() & INT_DRDY) != 0;
}

bool ICM45686::interruptPinLevel() const {
    return intPin_ != NO_PIN && gpio_get(intPin_);
}

// ============================================================================
// Readings
// ============================================================================

bool ICM45686::readRaw(RawData& raw) {
    uint8_t b[14];
    if (!readRegisters(REG_ACCEL_DATA_X1, b, sizeof(b))) return false;
    raw.ax = toInt16(&b[0]);
    raw.ay = toInt16(&b[2]);
    raw.az = toInt16(&b[4]);
    raw.gx = toInt16(&b[6]);
    raw.gy = toInt16(&b[8]);
    raw.gz = toInt16(&b[10]);
    raw.temp = toInt16(&b[12]);
    return true;
}

bool ICM45686::read(Data& d) {
    RawData r;
    if (!readRaw(r)) return false;
    const float as = accelScale(), gs = gyroScale();
    d.ax = r.ax * as - accelOffset_[0];
    d.ay = r.ay * as - accelOffset_[1];
    d.az = r.az * as - accelOffset_[2];
    d.gx = r.gx * gs - gyroOffset_[0];
    d.gy = r.gy * gs - gyroOffset_[1];
    d.gz = r.gz * gs - gyroOffset_[2];
    d.temperature = r.temp / 128.0f + 25.0f;
    return true;
}

bool ICM45686::readAccel(float& x, float& y, float& z) {
    uint8_t b[6];
    if (!readRegisters(REG_ACCEL_DATA_X1, b, sizeof(b))) return false;
    const float s = accelScale();
    x = toInt16(&b[0]) * s - accelOffset_[0];
    y = toInt16(&b[2]) * s - accelOffset_[1];
    z = toInt16(&b[4]) * s - accelOffset_[2];
    return true;
}

bool ICM45686::readGyro(float& x, float& y, float& z) {
    uint8_t b[6];
    if (!readRegisters(REG_GYRO_DATA_X1, b, sizeof(b))) return false;
    const float s = gyroScale();
    x = toInt16(&b[0]) * s - gyroOffset_[0];
    y = toInt16(&b[2]) * s - gyroOffset_[1];
    z = toInt16(&b[4]) * s - gyroOffset_[2];
    return true;
}

float ICM45686::readTemperature() {
    uint8_t b[2];
    if (!readRegisters(REG_TEMP_DATA1, b, sizeof(b))) return NAN;
    return toInt16(b) / 128.0f + 25.0f;
}

// ============================================================================
// Calibration
// ============================================================================

void ICM45686::setAccelOffset(float x, float y, float z) {
    accelOffset_[0] = x; accelOffset_[1] = y; accelOffset_[2] = z;
}

void ICM45686::setGyroOffset(float x, float y, float z) {
    gyroOffset_[0] = x; gyroOffset_[1] = y; gyroOffset_[2] = z;
}

bool ICM45686::calibrateGyro(uint16_t samples, uint32_t delayUs) {
    if (samples == 0) return false;
    setGyroOffset(0, 0, 0);
    double sx = 0, sy = 0, sz = 0;
    for (uint16_t i = 0; i < samples; i++) {
        float x, y, z;
        if (!readGyro(x, y, z)) return false;
        sx += x; sy += y; sz += z;
        sleep_us(delayUs);
    }
    setGyroOffset((float)(sx / samples), (float)(sy / samples), (float)(sz / samples));
    return true;
}
