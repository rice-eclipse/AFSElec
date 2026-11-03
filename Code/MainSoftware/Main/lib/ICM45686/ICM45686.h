/**
 * ICM45686.h - TDK InvenSense ICM-45686 6-axis IMU driver for the RP2040/RP2350 (Pico SDK), SPI.
 *
 * Register map and init sequence are based on the TDK ICM-45686 datasheet and
 * TDK's reference driver (hal_tdk/icm456xx, inv_imu_regmap_le.h / inv_imu_driver.c).
 *
 * Units: acceleration in g, angular rate in deg/s, temperature in deg C.
 *
 * Example:
 *   ICM45686 imu(spi0, IMU_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, IMU_INT1);
 *   if (!imu.begin()) { printf("IMU not found\n"); }
 *   imu.setAccelRange(ICM45686::AccelRange::G32);
 *   imu.setGyroRange(ICM45686::GyroRange::DPS4000);
 *   imu.setAccelODR(ICM45686::ODR::Hz1600);
 *   imu.setGyroODR(ICM45686::ODR::Hz1600);
 *   ICM45686::Data d;
 *   if (imu.read(d)) printf("%f %f %f\n", d.ax, d.ay, d.az);
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

class ICM45686 {
public:
    static constexpr uint NO_PIN = 0xFFFFFFFFu;
    static constexpr uint8_t WHO_AM_I_VALUE = 0xE9;

    // ---------------- Register addresses (bank 0, direct access) ----------------
    enum Reg : uint8_t {
        REG_ACCEL_DATA_X1 = 0x00,  // 0x00..0x05 accel X/Y/Z, 0x06..0x0B gyro, 0x0C..0x0D temp
        REG_GYRO_DATA_X1  = 0x06,
        REG_TEMP_DATA1    = 0x0C,
        REG_PWR_MGMT0     = 0x10,
        REG_FIFO_COUNT_0  = 0x12,
        REG_FIFO_DATA     = 0x14,
        REG_INT1_CONFIG0  = 0x16,
        REG_INT1_CONFIG1  = 0x17,
        REG_INT1_CONFIG2  = 0x18,
        REG_INT1_STATUS0  = 0x19,
        REG_INT1_STATUS1  = 0x1A,
        REG_ACCEL_CONFIG0 = 0x1B,
        REG_GYRO_CONFIG0  = 0x1C,
        REG_INTF_CONFIG1_OVRD = 0x2D,
        REG_DRIVE_CONFIG0 = 0x32,
        REG_WHO_AM_I      = 0x72,
        REG_IREG_ADDR_15_8 = 0x7C,
        REG_IREG_ADDR_7_0  = 0x7D,
        REG_IREG_DATA      = 0x7E,
        REG_MISC2          = 0x7F,
    };

    // Indirect (IPREG) registers, accessed through IREG_ADDR/IREG_DATA
    static constexpr uint16_t IREG_SREG_CTRL        = 0xA267; // bit1: data endianness (1 = big endian)
    static constexpr uint16_t IREG_GYRO_SRC_CTRL    = 0xA4A6; // IPREG_SYS1_REG_166, bits 6:5
    static constexpr uint16_t IREG_GYRO_UI_LPF      = 0xA4AC; // IPREG_SYS1_REG_172, bits 2:0
    static constexpr uint16_t IREG_ACCEL_SRC_CTRL   = 0xA57B; // IPREG_SYS2_REG_123, bits 1:0
    static constexpr uint16_t IREG_ACCEL_UI_LPF     = 0xA583; // IPREG_SYS2_REG_131, bits 2:0

    // INT1_CONFIG0 / INT1_STATUS0 bits
    static constexpr uint8_t INT_FIFO_FULL  = 1u << 0;
    static constexpr uint8_t INT_FIFO_THS   = 1u << 1;
    static constexpr uint8_t INT_DRDY       = 1u << 2;
    static constexpr uint8_t INT_AUX1_DRDY  = 1u << 3;
    static constexpr uint8_t INT_AP_FSYNC   = 1u << 4;
    static constexpr uint8_t INT_RESET_DONE = 1u << 7;

    // ---------------- Settings ----------------
    enum class AccelRange : uint8_t { G32 = 0, G16 = 1, G8 = 2, G4 = 3, G2 = 4 };

    enum class GyroRange : uint8_t {
        DPS4000 = 0, DPS2000 = 1, DPS1000 = 2, DPS500 = 3, DPS250 = 4,
        DPS125 = 5, DPS62_5 = 6, DPS31_25 = 7, DPS15_625 = 8
    };

    // Output data rate. 6400..800 Hz are low-noise only, 6.25..1.5625 Hz are low-power only.
    enum class ODR : uint8_t {
        Hz6400 = 3, Hz3200 = 4, Hz1600 = 5, Hz800 = 6, Hz400 = 7, Hz200 = 8, Hz100 = 9,
        Hz50 = 10, Hz25 = 11, Hz12_5 = 12, Hz6_25 = 13, Hz3_125 = 14, Hz1_5625 = 15
    };

    enum class AccelMode : uint8_t { Off = 0, LowPower = 2, LowNoise = 3 };
    enum class GyroMode  : uint8_t { Off = 0, Standby = 1, LowPower = 2, LowNoise = 3 };

    // UI path low-pass filter bandwidth (low-noise mode), as a fraction of ODR
    enum class LPF : uint8_t { Off = 0, ODR_4 = 1, ODR_8 = 2, ODR_16 = 3, ODR_32 = 4, ODR_64 = 5, ODR_128 = 6 };

    struct RawData {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t temp;
    };

    struct Data {
        float ax, ay, az;  // g
        float gx, gy, gz;  // deg/s
        float temperature; // deg C
    };

    /**
     * @param spi     SPI instance (spi0 / spi1)
     * @param csPin   chip select GPIO
     * @param sckPin  SPI clock GPIO
     * @param mosiPin SPI TX GPIO
     * @param misoPin SPI RX GPIO
     * @param intPin  INT1 GPIO, or NO_PIN if unused
     * @param spiHz   SPI clock (max 24 MHz)
     */
    ICM45686(spi_inst_t* spi, uint csPin, uint sckPin, uint mosiPin, uint misoPin,
             uint intPin = NO_PIN, uint32_t spiHz = 10'000'000);

    /**
     * Initializes pins (and the SPI bus if initBus and it isn't already running), soft-resets the
     * device, checks WHO_AM_I and applies the default config:
     * accel ±16 g / gyro ±2000 dps, 1.6 kHz, both in low-noise mode.
     */
    bool begin(bool initBus = true);
    bool reset();
    uint8_t whoAmI();
    bool isConnected();

    // ---------------- Configuration ----------------
    bool setAccelRange(AccelRange range);
    bool setGyroRange(GyroRange range);
    bool setAccelODR(ODR odr);
    bool setGyroODR(ODR odr);
    bool setAccelMode(AccelMode mode);
    bool setGyroMode(GyroMode mode);
    bool setAccelLPF(LPF bw);
    bool setGyroLPF(LPF bw);
    /** Enable/disable the FIR anti-alias filters (off at power-up). */
    bool setAntiAliasFilter(bool accelEnable, bool gyroEnable);
    /** SPI pad slew rate, 0 (38 ns) .. 6 (0.5 ns). Default after begin(): 2 (10 ns). */
    bool setSpiSlewRate(uint8_t slew);

    AccelRange getAccelRange() const { return accelRange_; }
    GyroRange  getGyroRange()  const { return gyroRange_; }
    ODR getAccelODR() const { return accelODR_; }
    ODR getGyroODR()  const { return gyroODR_; }
    float accelScale() const;  // g per LSB
    float gyroScale()  const;  // dps per LSB
    void setSpiFrequency(uint32_t hz) { spiHz_ = hz; }

    // ---------------- Interrupts ----------------
    /** Electrical configuration of INT1. */
    bool configureInterrupt1(bool activeHigh = true, bool latched = false, bool pushPull = true);
    /** Bitmask of INT_* sources routed to INT1 (written to INT1_CONFIG0). */
    bool setInterrupt1Sources(uint8_t mask);
    /**
     * Enables the data-ready status on INT1 (on by default after begin()). The DRDY status bit is
     * only set while this is enabled, so disabling it also stops dataReady() from working.
     */
    bool enableDataReadyInterrupt(bool enable = true);
    /** Reads (and thereby clears) INT1_STATUS0. */
    uint8_t getInterruptStatus();
    /** True if a new sample is available (reads/clears INT1_STATUS0). Requires enableDataReadyInterrupt(true). */
    bool dataReady();
    /** Raw level of the INT1 GPIO (false if no pin was given). */
    bool interruptPinLevel() const;
    uint intPin() const { return intPin_; }

    // ---------------- Readings ----------------
    bool readRaw(RawData& raw);
    bool read(Data& data);                 // accel + gyro + temp in one burst
    bool readAccel(float& x, float& y, float& z);
    bool readGyro(float& x, float& y, float& z);
    float readTemperature();               // NAN on failure

    // ---------------- Software calibration ----------------
    void setAccelOffset(float x, float y, float z);   // g, subtracted from readings
    void setGyroOffset(float x, float y, float z);    // dps, subtracted from readings
    /** Averages `samples` gyro readings (device must be still) and stores them as the gyro offset. */
    bool calibrateGyro(uint16_t samples = 500, uint32_t delayUs = 1000);

    // ---------------- Low-level access ----------------
    bool readRegisters(uint8_t reg, uint8_t* buf, size_t len);
    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    bool readIndirect(uint16_t addr, uint8_t& value);
    bool writeIndirect(uint16_t addr, uint8_t value);

private:
    void select();
    void deselect();
    bool modifyRegister(uint8_t reg, uint8_t mask, uint8_t value);
    bool modifyIndirect(uint16_t addr, uint8_t mask, uint8_t value);
    int16_t toInt16(const uint8_t* p) const;

    spi_inst_t* spi_;
    uint cs_, sck_, mosi_, miso_, intPin_;
    uint32_t spiHz_;
    bool bigEndian_ = false;

    AccelRange accelRange_ = AccelRange::G16;
    GyroRange gyroRange_ = GyroRange::DPS2000;
    ODR accelODR_ = ODR::Hz1600;
    ODR gyroODR_ = ODR::Hz1600;

    float accelOffset_[3] = {0, 0, 0};
    float gyroOffset_[3] = {0, 0, 0};
};
