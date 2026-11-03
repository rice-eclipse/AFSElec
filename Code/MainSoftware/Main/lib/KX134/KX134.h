/**
 * KX134.h - Kionix KX134-1211 ±64 g 3-axis accelerometer driver for the RP2040/RP2350 (Pico SDK), SPI.
 *
 * Based on the KX134-1211 datasheet / Technical Reference Manual and the register map used by
 * the SparkFun KX13X Arduino library.
 *
 * Units: acceleration in g.
 *
 * Example:
 *   KX134 acc(spi0, ACC_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, ACC_INT);
 *   if (!acc.begin()) { printf("KX134 not found\n"); }
 *   acc.setRange(KX134::Range::G64);       // settings may be changed at any time;
 *   acc.setODR(KX134::ODR::Hz1600);        // the driver handles standby/re-enable
 *   KX134::Data d;
 *   if (acc.read(d)) printf("%f %f %f\n", d.x, d.y, d.z);
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

class KX134 {
public:
    static constexpr uint NO_PIN = 0xFFFFFFFFu;
    static constexpr uint8_t WHO_AM_I_VALUE = 0x46;
    static constexpr uint8_t COTR_VALUE = 0x55;

    enum Reg : uint8_t {
        REG_MAN_ID = 0x00, REG_PART_ID = 0x01,
        REG_XADP_L = 0x02,
        REG_XOUT_L = 0x08, // X/Y/Z out: 0x08..0x0D (little endian)
        REG_COTR = 0x12, REG_WHO_AM_I = 0x13,
        REG_TSCP = 0x14, REG_TSPP = 0x15,
        REG_INS1 = 0x16, REG_INS2 = 0x17, REG_INS3 = 0x18,
        REG_STATUS = 0x19, REG_INT_REL = 0x1A,
        REG_CNTL1 = 0x1B, REG_CNTL2 = 0x1C, REG_CNTL3 = 0x1D, REG_CNTL4 = 0x1E,
        REG_CNTL5 = 0x1F, REG_CNTL6 = 0x20,
        REG_ODCNTL = 0x21,
        REG_INC1 = 0x22, REG_INC2 = 0x23, REG_INC3 = 0x24, REG_INC4 = 0x25,
        REG_INC5 = 0x26, REG_INC6 = 0x27,
        REG_FFTH = 0x32, REG_FFC = 0x33, REG_FFCNTL = 0x34,
        REG_LP_CNTL1 = 0x3A, REG_LP_CNTL2 = 0x3B,
        REG_WUFTH = 0x49, REG_BTSWUFTH = 0x4A, REG_BTSTH = 0x4B, REG_BTSC = 0x4C, REG_WUFC = 0x4D,
        REG_SELF_TEST = 0x5D,
        REG_BUF_CNTL1 = 0x5E, REG_BUF_CNTL2 = 0x5F,
        REG_BUF_STATUS_1 = 0x60, REG_BUF_STATUS_2 = 0x61,
        REG_BUF_CLEAR = 0x62, REG_BUF_READ = 0x63,
        REG_INTERNAL_7F = 0x7F,
    };

    // Interrupt source bits (INC4 routing to INT1 / INS2 status)
    static constexpr uint8_t INT_TILT       = 1u << 0;
    static constexpr uint8_t INT_WAKEUP     = 1u << 1;
    static constexpr uint8_t INT_TAP        = 1u << 2;
    static constexpr uint8_t INT_BACK_SLEEP = 1u << 3;
    static constexpr uint8_t INT_DATA_READY = 1u << 4;
    static constexpr uint8_t INT_WATERMARK  = 1u << 5;
    static constexpr uint8_t INT_BUFFER_FULL= 1u << 6;
    static constexpr uint8_t INT_FREEFALL   = 1u << 7;

    enum class Range : uint8_t { G8 = 0, G16 = 1, G32 = 2, G64 = 3 };

    enum class ODR : uint8_t {
        Hz0_781 = 0, Hz1_563 = 1, Hz3_125 = 2, Hz6_25 = 3, Hz12_5 = 4, Hz25 = 5, Hz50 = 6,
        Hz100 = 7, Hz200 = 8, Hz400 = 9, Hz800 = 10, Hz1600 = 11, Hz3200 = 12,
        Hz6400 = 13, Hz12800 = 14, Hz25600 = 15
    };

    enum class BufferMode : uint8_t { FIFO = 0, Stream = 1, Trigger = 2 };

    struct RawData { int16_t x, y, z; };
    struct Data { float x, y, z; }; // g

    /**
     * @param spi     SPI instance (spi0 / spi1)
     * @param csPin   chip select GPIO
     * @param sckPin  SPI clock GPIO
     * @param mosiPin SPI TX GPIO
     * @param misoPin SPI RX GPIO
     * @param intPin  INT1 GPIO, or NO_PIN if unused
     * @param spiHz   SPI clock (max 10 MHz)
     */
    KX134(spi_inst_t* spi, uint csPin, uint sckPin, uint mosiPin, uint misoPin,
          uint intPin = NO_PIN, uint32_t spiHz = 8'000'000);

    /**
     * Initializes pins (and the SPI bus if initBus and it isn't already running), soft-resets the
     * device, checks WHO_AM_I/COTR and enables it with the default config:
     * ±64 g, 1600 Hz, high-performance mode.
     */
    bool begin(bool initBus = true);
    bool reset();
    uint8_t whoAmI();
    bool isConnected();

    // ---------------- Operating mode ----------------
    /** PC1: true = measuring, false = standby (required to change most settings). */
    bool enable(bool on = true);
    bool isEnabled() const { return (cntl1_ & 0x80) != 0; }

    // ---------------- Configuration (auto standby/re-enable) ----------------
    bool setRange(Range range);
    bool setODR(ODR odr);
    /** true = high-performance (16-bit, full ODR range), false = low-power. */
    bool setHighPerformance(bool hp);
    /** IIR low-pass filter: bypass it, or set corner at ODR/9 (false) or ODR/2 (true). */
    bool setFilter(bool bypassIIR, bool cornerOdrOver2 = false);
    /** Enable the data-ready engine (needed for data-ready interrupt / dataReady()). */
    bool enableDataReadyEngine(bool enable = true);

    Range getRange() const { return range_; }
    ODR getODR() const { return odr_; }
    float scale() const; // g per LSB
    void setSpiFrequency(uint32_t hz) { spiHz_ = hz; }

    // ---------------- Interrupts ----------------
    /** Physical INT1 pin config: enable, active high/low, latched or pulsed. */
    bool configureInterrupt1(bool enablePin = true, bool activeHigh = true, bool latched = true);
    /** Bitmask of INT_* sources routed to INT1 (INC4). */
    bool setInterrupt1Sources(uint8_t mask);
    /** Convenience: data-ready engine + routing to INT1. */
    bool enableDataReadyInterrupt(bool enable = true);
    /** INS2 status bits (INT_* masks). */
    uint8_t getInterruptStatus();
    /** Releases latched interrupts (reads INT_REL). */
    void clearInterrupts();
    bool dataReady();
    bool interruptPinLevel() const;
    uint intPin() const { return intPin_; }

    // ---------------- Readings ----------------
    bool readRaw(RawData& raw);
    bool read(Data& data);
    bool read(float& x, float& y, float& z);

    // ---------------- Sample buffer (FIFO) ----------------
    /**
     * @param watermark sample threshold for the watermark interrupt
     * @param mode FIFO (stop when full), Stream (overwrite oldest), Trigger
     * @param fullInterrupt enable buffer-full interrupt engine
     * Buffer is configured for 16-bit samples (86 samples max).
     */
    bool enableBuffer(uint8_t watermark = 64, BufferMode mode = BufferMode::Stream, bool fullInterrupt = false);
    bool disableBuffer();
    /** Number of complete XYZ samples in the buffer. */
    uint16_t bufferSampleCount();
    /** Reads up to maxSamples samples from the buffer, returns the number read. */
    uint16_t readBuffer(Data* out, uint16_t maxSamples);
    bool clearBuffer();

    // ---------------- Software calibration ----------------
    void setOffset(float x, float y, float z); // g, subtracted from readings

    // ---------------- Low-level access ----------------
    bool readRegisters(uint8_t reg, uint8_t* buf, size_t len);
    uint8_t readRegister(uint8_t reg);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool modifyRegister(uint8_t reg, uint8_t mask, uint8_t value);

private:
    void select();
    void deselect();
    /** Runs `fn` with the device in standby, then restores the previous PC1 state. */
    template <typename F> bool inStandby(F fn);

    spi_inst_t* spi_;
    uint cs_, sck_, mosi_, miso_, intPin_;
    uint32_t spiHz_;

    uint8_t cntl1_ = 0;
    Range range_ = Range::G64;
    ODR odr_ = ODR::Hz1600;
    float offset_[3] = {0, 0, 0};
};
