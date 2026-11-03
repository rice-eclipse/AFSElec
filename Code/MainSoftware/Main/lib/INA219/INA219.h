/**
 * INA219.h - TI INA219 high-side current / power monitor driver for the RP2040/RP2350 (Pico SDK), I2C.
 *
 * Based on the INA219 datasheet (SBOS448).
 *
 * Units: volts, milliamps, milliwatts (see each method).
 *
 * Example:
 *   INA219 pm(i2c0, SDA, SCL, 0x40);
 *   pm.begin();
 *   pm.setCalibration(0.010f, 8.0f);       // 10 mOhm shunt, 8 A max expected current
 *   printf("%.3f V  %.1f mA  %.1f mW\n", pm.readBusVoltage(), pm.readCurrent_mA(), pm.readPower_mW());
 */
#pragma once

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

class INA219 {
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x40; // A0/A1 select 0x40..0x4F

    enum Reg : uint8_t {
        REG_CONFIG      = 0x00,
        REG_SHUNT_VOLT  = 0x01,
        REG_BUS_VOLT    = 0x02,
        REG_POWER       = 0x03,
        REG_CURRENT     = 0x04,
        REG_CALIBRATION = 0x05,
    };

    enum class BusRange : uint8_t { V16 = 0, V32 = 1 };

    /** Shunt PGA gain / full-scale shunt voltage. */
    enum class Gain : uint8_t { mV40 = 0, mV80 = 1, mV160 = 2, mV320 = 3 };

    /** ADC resolution / averaging (conversion time). */
    enum class ADC : uint8_t {
        Bits9 = 0x0,      // 84 us
        Bits10 = 0x1,     // 148 us
        Bits11 = 0x2,     // 276 us
        Bits12 = 0x3,     // 532 us
        Samples2 = 0x9,   // 1.06 ms
        Samples4 = 0xA,   // 2.13 ms
        Samples8 = 0xB,   // 4.26 ms
        Samples16 = 0xC,  // 8.51 ms
        Samples32 = 0xD,  // 17.02 ms
        Samples64 = 0xE,  // 34.05 ms
        Samples128 = 0xF, // 68.10 ms
    };

    enum class Mode : uint8_t {
        PowerDown = 0,
        ShuntTriggered = 1,
        BusTriggered = 2,
        ShuntBusTriggered = 3,
        ADCOff = 4,
        ShuntContinuous = 5,
        BusContinuous = 6,
        ShuntBusContinuous = 7,
    };

    /**
     * @param i2c       I2C instance (i2c0 / i2c1)
     * @param sdaPin    SDA GPIO
     * @param sclPin    SCL GPIO
     * @param address   7-bit I2C address (0x40..0x4F)
     * @param shuntOhms shunt resistor value
     * @param maxCurrentA maximum expected current (sets the current LSB)
     * @param i2cHz     I2C clock (max 400 kHz in fast mode)
     */
    INA219(i2c_inst_t* i2c, uint sdaPin, uint sclPin, uint8_t address = DEFAULT_ADDRESS,
           float shuntOhms = 0.1f, float maxCurrentA = 3.2f, uint32_t i2cHz = 400'000);

    /**
     * Initializes pins/bus (if initBus and not already running), resets the device and applies the
     * default config: 32 V range, gain picked from shunt/max current, 12-bit ADCs, continuous mode,
     * and writes the calibration register.
     */
    bool begin(bool initBus = true);
    bool reset();
    bool isConnected();
    void setAddress(uint8_t address) { addr_ = address; }
    uint8_t getAddress() const { return addr_; }

    // ---------------- Configuration ----------------
    bool setBusRange(BusRange range);
    bool setGain(Gain gain);
    bool setBusADC(ADC adc);
    bool setShuntADC(ADC adc);
    bool setMode(Mode mode);
    /** Write the whole configuration register at once. */
    bool configure(BusRange range, Gain gain, ADC busAdc, ADC shuntAdc, Mode mode);

    /**
     * Computes and writes the calibration register.
     * @param autoGain pick the smallest PGA range that fits maxCurrentA * shuntOhms
     */
    bool setCalibration(float shuntOhms, float maxCurrentA, bool autoGain = true);
    /** Re-writes the stored calibration value (the INA219 loses it on reset/brown-out). */
    bool restoreCalibration();
    /** If enabled (default), calibration is re-written before each current/power read. */
    void setAutoRecalibrate(bool enable) { autoRecal_ = enable; }

    float getShuntOhms() const { return shuntOhms_; }
    float getCurrentLSB() const { return currentLSB_; } // A per LSB
    float getPowerLSB() const { return currentLSB_ * 20.0f; } // W per LSB
    uint16_t getCalibrationValue() const { return cal_; }

    // ---------------- Readings ----------------
    float readShuntVoltage_mV();
    float readBusVoltage();        // V (IN- to GND)
    float readLoadVoltage();       // V, bus + shunt (supply side)
    float readCurrent_mA();        // from the current register (needs calibration)
    float readPower_mW();          // from the power register (needs calibration)
    /** Current computed directly from the shunt voltage and shunt value (no calibration needed). */
    float readCurrentFromShunt_mA();

    int16_t readShuntRaw();        // 10 uV / LSB
    uint16_t readBusRaw();         // raw bus register (bits 15:3 value, bit1 CNVR, bit0 OVF)
    int16_t readCurrentRaw();
    uint16_t readPowerRaw();

    /** Conversion ready flag (CNVR). Cleared when the power register is read. */
    bool conversionReady();
    /** Math overflow flag (OVF): current/power out of range. */
    bool overflow();
    /** Triggers a conversion in a triggered mode (rewrites config). */
    bool trigger();

    // ---------------- Low-level ----------------
    bool readRegister(uint8_t reg, uint16_t& value);
    bool writeRegister(uint8_t reg, uint16_t value);

private:
    bool modifyConfig(uint16_t mask, uint16_t value);

    i2c_inst_t* i2c_;
    uint sda_, scl_;
    uint8_t addr_;
    uint32_t i2cHz_;

    float shuntOhms_;
    float maxCurrentA_;
    float currentLSB_ = 0;
    uint16_t cal_ = 0;
    bool autoRecal_ = true;
    uint16_t config_ = 0x399F; // power-on default
};
