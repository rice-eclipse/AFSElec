/**
 * MMC5983MA.h - MEMSIC MMC5983MA 3-axis magnetometer driver for the RP2040/RP2350 (Pico SDK), SPI.
 *
 * Based on the MMC5983MA datasheet and the register conventions of the SparkFun MMC5983MA
 * Arduino library. Internal Control registers 0-3 are write-only, so the driver keeps shadow
 * copies of them.
 *
 * Units: field in Gauss (1 G = 100 uT), temperature in deg C.
 *
 * Example:
 *   MMC5983MA mag(spi0, MAG_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, MAG_INT);
 *   mag.begin();   // auto-detects SPI mode / read framing if the defaults don't answer
 *   MMC5983MA::Data m;
 *   mag.readSingle(m);                                   // one-shot measurement
 *   mag.calibrateOffset();                               // SET/RESET bridge-offset removal
 *   mag.setBandwidth(MMC5983MA::Bandwidth::Hz800);
 *   mag.startContinuous(MMC5983MA::ContinuousRate::Hz1000);
 *   mag.read(m);                                         // latest continuous sample
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

class MMC5983MA {
public:
    static constexpr uint NO_PIN = 0xFFFFFFFFu;
    static constexpr uint8_t PRODUCT_ID_VALUE = 0x30;
    static constexpr uint32_t ZERO_FIELD_18BIT = 131072;  // 2^17
    static constexpr float COUNTS_PER_GAUSS_18BIT = 16384.0f;
    static constexpr uint8_t READ_FLAGS_SPARKFUN = 0x80; // RW bit only
    static constexpr uint8_t READ_FLAGS_PX4      = 0xC0; // RW bit + bit 6 (as used by the PX4 driver)

    enum Reg : uint8_t {
        REG_XOUT0 = 0x00, REG_XOUT1 = 0x01, REG_YOUT0 = 0x02, REG_YOUT1 = 0x03,
        REG_ZOUT0 = 0x04, REG_ZOUT1 = 0x05, REG_XYZOUT2 = 0x06,
        REG_TOUT = 0x07, REG_STATUS = 0x08,
        REG_CTRL0 = 0x09, REG_CTRL1 = 0x0A, REG_CTRL2 = 0x0B, REG_CTRL3 = 0x0C,
        REG_PRODUCT_ID = 0x2F,
    };

    // STATUS bits
    static constexpr uint8_t STATUS_MEAS_M_DONE = 0x01;
    static constexpr uint8_t STATUS_MEAS_T_DONE = 0x02;
    static constexpr uint8_t STATUS_OTP_READ_DONE = 0x10;

    /** Decimation filter bandwidth; also sets the measurement time (8 / 4 / 2 / 0.5 ms). */
    enum class Bandwidth : uint8_t { Hz100 = 0, Hz200 = 1, Hz400 = 2, Hz800 = 3 };

    /** Continuous-mode rate. 200 Hz needs BW >= 200 Hz, 1000 Hz needs BW = 800 Hz. */
    enum class ContinuousRate : uint8_t { Off = 0, Hz1 = 1, Hz10 = 2, Hz20 = 3, Hz50 = 4, Hz100 = 5, Hz200 = 6, Hz1000 = 7 };

    /** In continuous mode with periodic SET enabled: perform SET once every N measurements. */
    enum class PeriodicSet : uint8_t { Every1 = 0, Every25 = 1, Every75 = 2, Every100 = 3, Every250 = 4, Every500 = 5, Every1000 = 6, Every2000 = 7 };

    struct RawData { uint32_t x, y, z; }; // 18-bit unsigned, 131072 = zero field
    struct Data { float x, y, z; };      // Gauss

    /**
     * @param spi     SPI instance (spi0 / spi1)
     * @param csPin   chip select GPIO
     * @param sckPin  SPI clock GPIO
     * @param mosiPin SPI TX GPIO
     * @param misoPin SPI RX GPIO
     * @param intPin  INT GPIO, or NO_PIN if unused
     * @param spiHz   SPI clock (max 10 MHz)
     */
    MMC5983MA(spi_inst_t* spi, uint csPin, uint sckPin, uint mosiPin, uint misoPin,
              uint intPin = NO_PIN, uint32_t spiHz = 8'000'000);

    /** Initializes pins/bus, soft-resets, checks the product ID. Device is left in one-shot mode, BW 100 Hz. */
    bool begin(bool initBus = true);
    bool reset();
    uint8_t productId();
    bool isConnected();

    // ---------------- Configuration ----------------
    bool setBandwidth(Bandwidth bw);
    Bandwidth getBandwidth() const { return bw_; }
    /** Measurement duration for the current bandwidth in microseconds. */
    uint32_t measurementTimeUs() const;
    /** Inhibit (disable) channels to speed up measurements. Y and Z can only be inhibited together. */
    bool setChannelInhibit(bool x, bool yz);
    /**
     * Automatic SET/RESET before each measurement (recommended). Note: in continuous mode this halves the
     * actual output rate (measured: 100 Hz setting -> ~54 Hz), so pick the next rate up if you need it.
     */
    bool enableAutoSetReset(bool enable = true);
    /** Periodic SET in continuous mode (requires auto set/reset). */
    bool setPeriodicSet(bool enable, PeriodicSet period = PeriodicSet::Every100);
    /** Start continuous measurement mode at the given rate. */
    bool startContinuous(ContinuousRate rate);
    bool stopContinuous();
    bool isContinuous() const { return (ctrl2_ & 0x08) != 0; }
    void setSpiFrequency(uint32_t hz) { spiHz_ = hz; }
    uint32_t getSpiFrequency() const { return spiHz_; }
    /**
     * SPI mode 0 (default) or 3. The datasheet describes mode 3, but mode 0 works and keeps SCK idling
     * LOW, so traffic for other devices can't easily form I2C START conditions (SDA falling while SCL
     * high) that the chip's always-on I2C interface would react to. Use mode 0 on a shared bus.
     */
    void setSpiMode(uint8_t mode) { spiMode3_ = (mode == 3); }
    uint8_t getSpiMode() const { return spiMode3_ ? 3 : 0; }
    uint misoPin() const { return miso_; }
    spi_inst_t* spiInstance() const { return spi_; }
    /**
     * Bits OR'd into the register address for SPI reads. READ_FLAGS_SPARKFUN (0x80, default) or
     * READ_FLAGS_PX4 (0xC0). The datasheet makes bit 6 a don't-care, so either works.
     */
    void setReadFlags(uint8_t flags) { readFlags_ = flags; }
    uint8_t getReadFlags() const { return readFlags_; }
    /** Tries SPI mode 0 and 3 (at the set speed, then 1 MHz); keeps the first that reads the product ID 3 times in a row. */
    bool autoDetectBus();
    /**
     * I2C guard (default on): send an I2C STOP on SCK/MOSI (CS high) before every transaction. The
     * MMC5983MA has no way to disable its I2C interface, which listens whenever CS is high; the STOP
     * ends any phantom I2C transaction that other devices' traffic may have started. Costs ~3-4 us per
     * transaction. Only turn off if the chip has a bus to itself.
     */
    void setI2CGuard(bool enable) { i2cGuard_ = enable; }
    bool getI2CGuard() const { return i2cGuard_; }
    /**
     * Bit-banged transfers (default on). The RP2350 SPI block changes MOSI on the same SCK edge, which the
     * chip's always-on I2C interface can mistake for an I2C START. Bit-banging changes MOSI only well
     * after SCK has fallen, so the driver's own traffic can't do that. Runs at up to the set SPI
     * frequency (in practice a few MHz); a 7-byte field read takes ~15-30 us. false = hardware SPI.
     */
    void setBitBang(bool enable) { bitBang_ = enable; }
    bool getBitBang() const { return bitBang_; }
    /** Sends just the I2C STOP (CS high). Called automatically before each transaction when the guard is on. */
    void sendI2CStop();
    /**
     * While CS is high the chip's I2C interface listens on SCK/MOSI, so other devices' SPI traffic
     * can leave it stuck mid-"I2C transaction" and ignoring SPI. This bit-bangs 9 clocks + an I2C STOP
     * on SCK/MOSI (CS high) to free it. Safe to call at any time; begin() calls it.
     */
    void recoverInterface();
    /** recoverInterface(); if the chip still doesn't answer, reset it and restore the current settings. */
    bool recover();

    // ---------------- Set / Reset ----------------
    bool performSet();
    bool performReset();
    /**
     * Measures the bridge offset using SET and RESET measurements (offset = (set + reset) / 2)
     * and stores it so it is subtracted from all subsequent readings. Use in one-shot mode.
     */
    bool calibrateOffset();
    /** One measurement right after SET and one right after RESET (one-shot mode). (set - reset) / 2 = field. */
    bool measureSetReset(RawData& afterSet, RawData& afterReset);
    void setOffset(float x, float y, float z) { offset_[0] = x; offset_[1] = y; offset_[2] = z; }
    void getOffset(float& x, float& y, float& z) const { x = offset_[0]; y = offset_[1]; z = offset_[2]; }
    /** Hard-iron (and optional soft-iron scale) correction applied after offset removal. */
    void setHardIron(float x, float y, float z) { hard_[0] = x; hard_[1] = y; hard_[2] = z; }
    void setSoftIronScale(float x, float y, float z) { soft_[0] = x; soft_[1] = y; soft_[2] = z; }
    void getHardIron(float& x, float& y, float& z) const { x = hard_[0]; y = hard_[1]; z = hard_[2]; }
    void getSoftIronScale(float& x, float& y, float& z) const { x = soft_[0]; y = soft_[1]; z = soft_[2]; }
    /**
     * Hard/soft-iron calibration (min/max method). Slowly rotate the board through every orientation
     * (figure-eights, all axes) for `durationMs`. Sets the hard-iron offset to the centre of the measured
     * range and a per-axis soft-iron scale so all axes match. Run calibrateOffset() first, in one-shot mode.
     * @param fieldStrength if given, receives the calibrated field magnitude (should be ~0.25..0.65 G)
     * @return false if an axis didn't see enough variation (board not rotated enough)
     */
    bool calibrateHardIron(uint32_t durationMs = 20000, float* fieldStrength = nullptr);

    // ---------------- Interrupts ----------------
    /** INT pin pulses when a measurement completes. */
    bool enableInterrupt(bool enable = true);
    /** Clears/re-arms the measurement-done interrupt (writes 1 to Meas_M_Done / Meas_T_Done). */
    bool clearInterrupt();
    uint8_t getStatus();
    /**
     * Meas_M_Done. Note: it stays set until the data is read (or a new measurement starts), so in
     * continuous mode call read() before polling again, then clearInterrupt() to re-arm INT.
     */
    bool dataReady();
    bool interruptPinLevel() const;
    uint intPin() const { return intPin_; }

    // ---------------- Readings ----------------
    /** Trigger a measurement (one-shot mode). */
    bool startMeasurement();
    /** Trigger + wait + read (one-shot mode). */
    bool readSingle(Data& data, uint32_t timeoutMs = 20);
    bool readSingleRaw(RawData& raw, uint32_t timeoutMs = 20);
    /** Reads the current output registers (continuous mode or after dataReady()). */
    bool readRaw(RawData& raw);
    bool read(Data& data);
    /** Triggers a temperature measurement and returns deg C (NAN on failure). One-shot mode only. */
    float readTemperature(uint32_t timeoutMs = 20);
    /** Heading in degrees (0-360) from X/Y, for a level sensor. */
    static float heading(const Data& d);

    // ---------------- Low-level ----------------
    bool readRegisters(uint8_t reg, uint8_t* buf, size_t len);
    uint8_t readRegister(uint8_t reg);
    bool writeRegister(uint8_t reg, uint8_t value);

private:
    void select();
    void deselect();
    bool waitStatus(uint8_t mask, uint32_t timeoutMs);
    bool idStable(int reads = 3);
    void takePins();
    void releasePins();
    void stopCondition();
    void bbDelay() const;
    void bbTransfer(const uint8_t* tx, uint8_t* rx, size_t len);
    bool writeCtrl0(uint8_t oneShotBits);
    void convert(const RawData& r, Data& d) const;

    spi_inst_t* spi_;
    uint cs_, sck_, mosi_, miso_, intPin_;
    uint32_t spiHz_;
    bool spiMode3_ = false; // mode 0: SCK idles low (see setSpiMode)
    bool i2cGuard_ = true;
    bool bitBang_ = true;
    uint32_t bbQuarterCycles_ = 10;
    uint8_t readFlags_ = READ_FLAGS_SPARKFUN;

    // Shadow copies of write-only control registers (persistent bits only)
    uint8_t ctrl0_ = 0, ctrl1_ = 0, ctrl2_ = 0, ctrl3_ = 0;
    Bandwidth bw_ = Bandwidth::Hz100;

    float offset_[3] = {0, 0, 0};
    float hard_[3] = {0, 0, 0};
    float soft_[3] = {1, 1, 1};
};
