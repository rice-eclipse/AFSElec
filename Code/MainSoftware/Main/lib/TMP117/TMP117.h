/**
 * TMP117.h - TI TMP117 high-accuracy digital temperature sensor driver for the RP2040/RP2350 (Pico SDK), I2C.
 *
 * Based on the TMP117 datasheet (SNOSD82).
 *
 * Units: deg C.
 *
 * Example:
 *   TMP117 temp(i2c0, SDA, SCL);                 // address 0x48 (ADD0 -> GND)
 *   temp.begin();
 *   temp.setAveraging(TMP117::Averaging::AVG8);
 *   temp.setConversionCycle(TMP117::ConversionCycle::C1);
 *   if (temp.dataReady()) printf("%.3f C\n", temp.readTemperature());
 */
#pragma once

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

class TMP117 {
public:
    static constexpr uint NO_PIN = 0xFFFFFFFFu;
    static constexpr uint16_t DEVICE_ID_VALUE = 0x0117;
    static constexpr float RESOLUTION = 0.0078125f; // deg C per LSB

    // I2C addresses selected by the ADD0 pin
    static constexpr uint8_t ADDR_GND = 0x48;
    static constexpr uint8_t ADDR_VCC = 0x49;
    static constexpr uint8_t ADDR_SDA = 0x4A;
    static constexpr uint8_t ADDR_SCL = 0x4B;

    enum Reg : uint8_t {
        REG_TEMP_RESULT = 0x00,
        REG_CONFIG      = 0x01,
        REG_THIGH_LIMIT = 0x02,
        REG_TLOW_LIMIT  = 0x03,
        REG_EEPROM_UL   = 0x04,
        REG_EEPROM1     = 0x05,
        REG_EEPROM2     = 0x06,
        REG_TEMP_OFFSET = 0x07,
        REG_EEPROM3     = 0x08,
        REG_DEVICE_ID   = 0x0F,
    };

    // Configuration register flags
    static constexpr uint16_t CFG_HIGH_ALERT   = 1u << 15;
    static constexpr uint16_t CFG_LOW_ALERT    = 1u << 14;
    static constexpr uint16_t CFG_DATA_READY   = 1u << 13;
    static constexpr uint16_t CFG_EEPROM_BUSY  = 1u << 12;

    enum class Mode : uint8_t { Continuous = 0, Shutdown = 1, OneShot = 3 };

    /** Averaged conversions per result. */
    enum class Averaging : uint8_t { None = 0, AVG8 = 1, AVG32 = 2, AVG64 = 3 };

    /**
     * Conversion cycle time. The actual cycle is max(cycle, averaging time):
     *            None    AVG8    AVG32   AVG64
     *   C0      15.5ms  125ms   500ms   1s
     *   C1      125ms   125ms   500ms   1s
     *   C2      250ms   250ms   500ms   1s
     *   C3      500ms   500ms   500ms   1s
     *   C4      1s      1s      1s      1s
     *   C5      4s      4s      4s      4s
     *   C6      8s      8s      8s      8s
     *   C7      16s     16s     16s     16s
     */
    enum class ConversionCycle : uint8_t { C0 = 0, C1, C2, C3, C4, C5, C6, C7 };

    enum class AlertMode : uint8_t { Alert = 0, Therm = 1 };

    /**
     * @param i2c     I2C instance (i2c0 / i2c1)
     * @param sdaPin  SDA GPIO
     * @param sclPin  SCL GPIO
     * @param address 7-bit I2C address (0x48..0x4B)
     * @param i2cHz   I2C clock (max 400 kHz)
     * @param alertPin ALERT GPIO, or NO_PIN if unused
     */
    TMP117(i2c_inst_t* i2c, uint sdaPin, uint sclPin, uint8_t address = ADDR_GND,
           uint32_t i2cHz = 400'000, uint alertPin = NO_PIN);

    /** Initializes pins/bus (if initBus and not already running), checks the device ID, soft-resets. */
    bool begin(bool initBus = true);
    bool reset();
    uint16_t deviceId();     // bits 11:0 (0x117)
    uint8_t revision();      // bits 15:12
    bool isConnected();
    void setAddress(uint8_t address) { addr_ = address; }
    uint8_t getAddress() const { return addr_; }

    // ---------------- Configuration ----------------
    bool setMode(Mode mode);
    /** Note: after a one-shot conversion completes, MOD can still read back as OneShot. */
    Mode getMode();
    bool setAveraging(Averaging avg);
    Averaging getAveraging();
    bool setConversionCycle(ConversionCycle cycle);
    ConversionCycle getConversionCycle();
    /** Starts a single conversion (device returns to shutdown afterwards). */
    bool triggerOneShot();

    /** Temperature offset added to results by the device (deg C, ±256). */
    bool setOffset(float degC);
    float getOffset();

    // ---------------- Alerts ----------------
    bool setHighLimit(float degC);
    bool setLowLimit(float degC);
    float getHighLimit();
    float getLowLimit();
    bool setAlertMode(AlertMode mode);
    bool setAlertPolarity(bool activeHigh);
    /** true: ALERT pin reflects data ready, false: ALERT pin reflects alert flags. */
    bool setAlertPinDataReady(bool dataReady);
    bool interruptPinLevel() const;

    // ---------------- Readings ----------------
    /** Reads (and thereby clears) the configuration flags. */
    uint16_t readConfig();
    /** True if a new result is available (reading clears the flags). */
    bool dataReady();
    /** Also returns the high/low alert flags (reading clears them). */
    bool alertStatus(bool& high, bool& low);
    /** Latest result in deg C (NAN on bus failure). */
    float readTemperature();
    int16_t readTemperatureRaw();
    /** Triggers a one-shot conversion and waits for the result (NAN on timeout). */
    float readTemperatureOneShot(uint32_t timeoutMs = 1200);

    // ---------------- EEPROM ----------------
    /** Saves the current configuration, limits and offset registers to EEPROM (power-on defaults). */
    bool saveToEEPROM();
    bool readEEPROM(uint8_t index, uint16_t& value);       // index 1..3 (general purpose)
    bool writeEEPROM(uint8_t index, uint16_t value);

    // ---------------- Low-level ----------------
    bool readRegister(uint8_t reg, uint16_t& value);
    bool writeRegister(uint8_t reg, uint16_t value);
    bool modifyConfig(uint16_t mask, uint16_t value);

private:
    bool waitEEPROM(uint32_t timeoutMs = 20);
    static int16_t degToRaw(float degC);

    i2c_inst_t* i2c_;
    uint sda_, scl_, alert_;
    uint8_t addr_;
    uint32_t i2cHz_;
};
