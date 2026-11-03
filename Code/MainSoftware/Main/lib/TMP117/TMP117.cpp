#include "TMP117.h"
#include <math.h>

static constexpr uint32_t I2C_TIMEOUT_US = 5000;

// Configuration register fields
static constexpr uint16_t CFG_MOD_MASK   = 0x0C00; // bits 11:10
static constexpr uint16_t CFG_CONV_MASK  = 0x0380; // bits 9:7
static constexpr uint16_t CFG_AVG_MASK   = 0x0060; // bits 6:5
static constexpr uint16_t CFG_TNA        = 0x0010; // bit 4: therm mode
static constexpr uint16_t CFG_POL        = 0x0008; // bit 3: alert polarity (1 = active high)
static constexpr uint16_t CFG_DR_ALERT   = 0x0004; // bit 2: ALERT pin = data ready
static constexpr uint16_t CFG_SOFT_RESET = 0x0002; // bit 1
static constexpr uint16_t CFG_WRITABLE   = 0x0FFC; // bits that can be written

// EEPROM_UL bits
static constexpr uint16_t EEPROM_UNLOCK = 0x8000;
static constexpr uint16_t EEPROM_BUSY   = 0x4000;

TMP117::TMP117(i2c_inst_t* i2c, uint sdaPin, uint sclPin, uint8_t address, uint32_t i2cHz, uint alertPin)
    : i2c_(i2c), sda_(sdaPin), scl_(sclPin), alert_(alertPin), addr_(address), i2cHz_(i2cHz) {}

// ============================================================================
// Bus helpers
// ============================================================================

bool TMP117::readRegister(uint8_t reg, uint16_t& value) {
    uint8_t b[2];
    if (i2c_write_timeout_us(i2c_, addr_, &reg, 1, true, I2C_TIMEOUT_US) != 1) return false;
    if (i2c_read_timeout_us(i2c_, addr_, b, 2, false, I2C_TIMEOUT_US) != 2) return false;
    value = (uint16_t)((b[0] << 8) | b[1]);
    return true;
}

bool TMP117::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t b[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return i2c_write_timeout_us(i2c_, addr_, b, 3, false, I2C_TIMEOUT_US) == 3;
}

bool TMP117::modifyConfig(uint16_t mask, uint16_t value) {
    uint16_t cfg;
    if (!readRegister(REG_CONFIG, cfg)) return false;
    cfg = (cfg & ~mask) | (value & mask);
    cfg &= CFG_WRITABLE & ~CFG_SOFT_RESET;
    return writeRegister(REG_CONFIG, cfg);
}

int16_t TMP117::degToRaw(float degC) {
    float r = degC / RESOLUTION;
    if (r > 32767.0f) r = 32767.0f;
    if (r < -32768.0f) r = -32768.0f;
    return (int16_t)lroundf(r);
}

// ============================================================================
// Setup
// ============================================================================

bool TMP117::begin(bool initBus) {
    if (initBus) {
        if (!i2c_get_hw(i2c_)->enable) {
            i2c_init(i2c_, i2cHz_);
        }
        gpio_set_function(sda_, GPIO_FUNC_I2C);
        gpio_set_function(scl_, GPIO_FUNC_I2C);
        gpio_pull_up(sda_);
        gpio_pull_up(scl_);
    }

    if (alert_ != NO_PIN) {
        gpio_init(alert_);
        gpio_set_dir(alert_, GPIO_IN);
        gpio_pull_up(alert_); // ALERT is open-drain
    }

    sleep_ms(2); // power-on reset time 1.5 ms
    if (!isConnected()) return false;
    return reset();
}

bool TMP117::reset() {
    bool ok = writeRegister(REG_CONFIG, CFG_SOFT_RESET);
    sleep_ms(2);
    return ok;
}

uint16_t TMP117::deviceId() {
    uint16_t v = 0;
    readRegister(REG_DEVICE_ID, v);
    return v & 0x0FFF;
}

uint8_t TMP117::revision() {
    uint16_t v = 0;
    readRegister(REG_DEVICE_ID, v);
    return (uint8_t)(v >> 12);
}

bool TMP117::isConnected() {
    return deviceId() == DEVICE_ID_VALUE;
}

// ============================================================================
// Configuration
// ============================================================================

bool TMP117::setMode(Mode mode) {
    return modifyConfig(CFG_MOD_MASK, (uint16_t)mode << 10);
}

TMP117::Mode TMP117::getMode() {
    uint8_t m = (readConfig() & CFG_MOD_MASK) >> 10;
    return m == 1 ? Mode::Shutdown : m == 3 ? Mode::OneShot : Mode::Continuous;
}

bool TMP117::setAveraging(Averaging avg) {
    return modifyConfig(CFG_AVG_MASK, (uint16_t)avg << 5);
}

TMP117::Averaging TMP117::getAveraging() {
    return (Averaging)((readConfig() & CFG_AVG_MASK) >> 5);
}

bool TMP117::setConversionCycle(ConversionCycle cycle) {
    return modifyConfig(CFG_CONV_MASK, (uint16_t)cycle << 7);
}

TMP117::ConversionCycle TMP117::getConversionCycle() {
    return (ConversionCycle)((readConfig() & CFG_CONV_MASK) >> 7);
}

bool TMP117::triggerOneShot() {
    return setMode(Mode::OneShot);
}

bool TMP117::setOffset(float degC) {
    return writeRegister(REG_TEMP_OFFSET, (uint16_t)degToRaw(degC));
}

float TMP117::getOffset() {
    uint16_t v;
    if (!readRegister(REG_TEMP_OFFSET, v)) return NAN;
    return (int16_t)v * RESOLUTION;
}

// ============================================================================
// Alerts
// ============================================================================

bool TMP117::setHighLimit(float degC) {
    return writeRegister(REG_THIGH_LIMIT, (uint16_t)degToRaw(degC));
}

bool TMP117::setLowLimit(float degC) {
    return writeRegister(REG_TLOW_LIMIT, (uint16_t)degToRaw(degC));
}

float TMP117::getHighLimit() {
    uint16_t v;
    if (!readRegister(REG_THIGH_LIMIT, v)) return NAN;
    return (int16_t)v * RESOLUTION;
}

float TMP117::getLowLimit() {
    uint16_t v;
    if (!readRegister(REG_TLOW_LIMIT, v)) return NAN;
    return (int16_t)v * RESOLUTION;
}

bool TMP117::setAlertMode(AlertMode mode) {
    return modifyConfig(CFG_TNA, mode == AlertMode::Therm ? CFG_TNA : 0);
}

bool TMP117::setAlertPolarity(bool activeHigh) {
    return modifyConfig(CFG_POL, activeHigh ? CFG_POL : 0);
}

bool TMP117::setAlertPinDataReady(bool dataReady) {
    return modifyConfig(CFG_DR_ALERT, dataReady ? CFG_DR_ALERT : 0);
}

bool TMP117::interruptPinLevel() const {
    return alert_ != NO_PIN && gpio_get(alert_);
}

// ============================================================================
// Readings
// ============================================================================

uint16_t TMP117::readConfig() {
    uint16_t v = 0;
    readRegister(REG_CONFIG, v);
    return v;
}

bool TMP117::dataReady() {
    return (readConfig() & CFG_DATA_READY) != 0;
}

bool TMP117::alertStatus(bool& high, bool& low) {
    uint16_t v;
    if (!readRegister(REG_CONFIG, v)) return false;
    high = (v & CFG_HIGH_ALERT) != 0;
    low = (v & CFG_LOW_ALERT) != 0;
    return true;
}

int16_t TMP117::readTemperatureRaw() {
    uint16_t v = 0x8000; // -256 C marks "no data"
    readRegister(REG_TEMP_RESULT, v);
    return (int16_t)v;
}

float TMP117::readTemperature() {
    uint16_t v;
    if (!readRegister(REG_TEMP_RESULT, v)) return NAN;
    return (int16_t)v * RESOLUTION;
}

float TMP117::readTemperatureOneShot(uint32_t timeoutMs) {
    readConfig(); // clear stale data-ready flag
    if (!triggerOneShot()) return NAN;
    absolute_time_t deadline = make_timeout_time_ms(timeoutMs);
    while (!dataReady()) {
        if (time_reached(deadline)) return NAN;
        sleep_ms(2);
    }
    return readTemperature();
}

// ============================================================================
// EEPROM
// ============================================================================

bool TMP117::waitEEPROM(uint32_t timeoutMs) {
    absolute_time_t deadline = make_timeout_time_ms(timeoutMs);
    uint16_t v;
    do {
        sleep_ms(1);
        if (!readRegister(REG_EEPROM_UL, v)) return false;
        if (!(v & EEPROM_BUSY)) return true;
    } while (!time_reached(deadline));
    return false;
}

bool TMP117::saveToEEPROM() {
    uint16_t cfg, hi, lo, off;
    if (!readRegister(REG_CONFIG, cfg) || !readRegister(REG_THIGH_LIMIT, hi) ||
        !readRegister(REG_TLOW_LIMIT, lo) || !readRegister(REG_TEMP_OFFSET, off)) return false;
    cfg &= CFG_WRITABLE & ~CFG_SOFT_RESET;

    bool ok = writeRegister(REG_EEPROM_UL, EEPROM_UNLOCK);
    // Each write while unlocked programs the corresponding EEPROM cell (~7 ms each)
    ok = ok && writeRegister(REG_CONFIG, cfg) && waitEEPROM();
    ok = ok && writeRegister(REG_THIGH_LIMIT, hi) && waitEEPROM();
    ok = ok && writeRegister(REG_TLOW_LIMIT, lo) && waitEEPROM();
    ok = ok && writeRegister(REG_TEMP_OFFSET, off) && waitEEPROM();
    ok &= writeRegister(REG_EEPROM_UL, 0x0000); // lock
    return ok;
}

bool TMP117::readEEPROM(uint8_t index, uint16_t& value) {
    static constexpr uint8_t regs[] = {REG_EEPROM1, REG_EEPROM2, REG_EEPROM3};
    if (index < 1 || index > 3) return false;
    return readRegister(regs[index - 1], value);
}

bool TMP117::writeEEPROM(uint8_t index, uint16_t value) {
    static constexpr uint8_t regs[] = {REG_EEPROM1, REG_EEPROM2, REG_EEPROM3};
    if (index < 1 || index > 3) return false;
    bool ok = writeRegister(REG_EEPROM_UL, EEPROM_UNLOCK);
    ok = ok && writeRegister(regs[index - 1], value) && waitEEPROM();
    ok &= writeRegister(REG_EEPROM_UL, 0x0000);
    return ok;
}
