#include "INA219.h"
#include <math.h>

static constexpr uint32_t I2C_TIMEOUT_US = 5000;

// Configuration register fields
static constexpr uint16_t CFG_RST   = 0x8000;
static constexpr uint16_t CFG_BRNG  = 0x2000; // bit 13
static constexpr uint16_t CFG_PG    = 0x1800; // bits 12:11
static constexpr uint16_t CFG_BADC  = 0x0780; // bits 10:7
static constexpr uint16_t CFG_SADC  = 0x0078; // bits 6:3
static constexpr uint16_t CFG_MODE  = 0x0007; // bits 2:0

INA219::INA219(i2c_inst_t* i2c, uint sdaPin, uint sclPin, uint8_t address,
               float shuntOhms, float maxCurrentA, uint32_t i2cHz)
    : i2c_(i2c), sda_(sdaPin), scl_(sclPin), addr_(address), i2cHz_(i2cHz),
      shuntOhms_(shuntOhms), maxCurrentA_(maxCurrentA) {}

// ============================================================================
// Bus helpers
// ============================================================================

bool INA219::readRegister(uint8_t reg, uint16_t& value) {
    uint8_t b[2];
    if (i2c_write_timeout_us(i2c_, addr_, &reg, 1, true, I2C_TIMEOUT_US) != 1) return false;
    if (i2c_read_timeout_us(i2c_, addr_, b, 2, false, I2C_TIMEOUT_US) != 2) return false;
    value = (uint16_t)((b[0] << 8) | b[1]);
    return true;
}

bool INA219::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t b[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return i2c_write_timeout_us(i2c_, addr_, b, 3, false, I2C_TIMEOUT_US) == 3;
}

bool INA219::modifyConfig(uint16_t mask, uint16_t value) {
    uint16_t cfg = (config_ & ~mask) | (value & mask);
    cfg &= ~CFG_RST;
    if (!writeRegister(REG_CONFIG, cfg)) return false;
    config_ = cfg;
    return true;
}

// ============================================================================
// Setup
// ============================================================================

bool INA219::begin(bool initBus) {
    if (initBus) {
        if (!i2c_get_hw(i2c_)->enable) {
            i2c_init(i2c_, i2cHz_);
        }
        gpio_set_function(sda_, GPIO_FUNC_I2C);
        gpio_set_function(scl_, GPIO_FUNC_I2C);
        gpio_pull_up(sda_);
        gpio_pull_up(scl_);
    }

    sleep_ms(1);
    if (!isConnected()) return false;
    if (!reset()) return false;

    bool ok = configure(BusRange::V32, Gain::mV320, ADC::Bits12, ADC::Bits12, Mode::ShuntBusContinuous);
    ok &= setCalibration(shuntOhms_, maxCurrentA_, true);
    return ok;
}

bool INA219::reset() {
    bool ok = writeRegister(REG_CONFIG, CFG_RST);
    sleep_ms(1);
    config_ = 0x399F;
    return ok;
}

bool INA219::isConnected() {
    uint16_t v;
    return readRegister(REG_CONFIG, v);
}

// ============================================================================
// Configuration
// ============================================================================

bool INA219::setBusRange(BusRange range) {
    return modifyConfig(CFG_BRNG, range == BusRange::V32 ? CFG_BRNG : 0);
}

bool INA219::setGain(Gain gain) {
    return modifyConfig(CFG_PG, (uint16_t)gain << 11);
}

bool INA219::setBusADC(ADC adc) {
    return modifyConfig(CFG_BADC, (uint16_t)adc << 7);
}

bool INA219::setShuntADC(ADC adc) {
    return modifyConfig(CFG_SADC, (uint16_t)adc << 3);
}

bool INA219::setMode(Mode mode) {
    return modifyConfig(CFG_MODE, (uint16_t)mode);
}

bool INA219::configure(BusRange range, Gain gain, ADC busAdc, ADC shuntAdc, Mode mode) {
    uint16_t cfg = (range == BusRange::V32 ? CFG_BRNG : 0) |
                   ((uint16_t)gain << 11) |
                   ((uint16_t)busAdc << 7) |
                   ((uint16_t)shuntAdc << 3) |
                   (uint16_t)mode;
    if (!writeRegister(REG_CONFIG, cfg)) return false;
    config_ = cfg;
    return true;
}

bool INA219::setCalibration(float shuntOhms, float maxCurrentA, bool autoGain) {
    if (shuntOhms <= 0.0f || maxCurrentA <= 0.0f) return false;
    shuntOhms_ = shuntOhms;
    maxCurrentA_ = maxCurrentA;

    // Datasheet: Current_LSB = Max_Expected_I / 2^15, Cal = trunc(0.04096 / (Current_LSB * R_shunt))
    float lsb = maxCurrentA / 32768.0f;
    float calF = 0.04096f / (lsb * shuntOhms);
    if (calF > 65534.0f) calF = 65534.0f;
    if (calF < 2.0f) calF = 2.0f;
    cal_ = (uint16_t)calF & 0xFFFE; // bit 0 is reserved
    currentLSB_ = 0.04096f / ((float)cal_ * shuntOhms); // actual LSB after truncation

    bool ok = true;
    if (autoGain) {
        float vMax = maxCurrentA * shuntOhms;
        Gain g = vMax <= 0.040f ? Gain::mV40 : vMax <= 0.080f ? Gain::mV80 :
                 vMax <= 0.160f ? Gain::mV160 : Gain::mV320;
        ok &= setGain(g);
    }
    ok &= writeRegister(REG_CALIBRATION, cal_);
    return ok;
}

bool INA219::restoreCalibration() {
    return writeRegister(REG_CALIBRATION, cal_);
}

// ============================================================================
// Readings
// ============================================================================

int16_t INA219::readShuntRaw() {
    uint16_t v = 0;
    readRegister(REG_SHUNT_VOLT, v);
    return (int16_t)v;
}

uint16_t INA219::readBusRaw() {
    uint16_t v = 0;
    readRegister(REG_BUS_VOLT, v);
    return v;
}

int16_t INA219::readCurrentRaw() {
    if (autoRecal_) restoreCalibration();
    uint16_t v = 0;
    readRegister(REG_CURRENT, v);
    return (int16_t)v;
}

uint16_t INA219::readPowerRaw() {
    if (autoRecal_) restoreCalibration();
    uint16_t v = 0;
    readRegister(REG_POWER, v);
    return v;
}

float INA219::readShuntVoltage_mV() {
    uint16_t v;
    if (!readRegister(REG_SHUNT_VOLT, v)) return NAN;
    return (int16_t)v * 0.01f; // 10 uV / LSB
}

float INA219::readBusVoltage() {
    uint16_t v;
    if (!readRegister(REG_BUS_VOLT, v)) return NAN;
    return (v >> 3) * 0.004f; // 4 mV / LSB
}

float INA219::readLoadVoltage() {
    return readBusVoltage() + readShuntVoltage_mV() / 1000.0f;
}

float INA219::readCurrent_mA() {
    if (autoRecal_ && !restoreCalibration()) return NAN;
    uint16_t v;
    if (!readRegister(REG_CURRENT, v)) return NAN;
    return (int16_t)v * currentLSB_ * 1000.0f;
}

float INA219::readPower_mW() {
    if (autoRecal_ && !restoreCalibration()) return NAN;
    uint16_t v;
    if (!readRegister(REG_POWER, v)) return NAN;
    return v * currentLSB_ * 20.0f * 1000.0f;
}

float INA219::readCurrentFromShunt_mA() {
    return readShuntVoltage_mV() / shuntOhms_;
}

bool INA219::conversionReady() {
    return (readBusRaw() & 0x0002) != 0;
}

bool INA219::overflow() {
    return (readBusRaw() & 0x0001) != 0;
}

bool INA219::trigger() {
    return writeRegister(REG_CONFIG, config_);
}
