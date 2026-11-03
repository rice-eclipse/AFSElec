/**
 * MS5611.h - TE MS5611-01BA03 barometric pressure sensor driver for the RP2040/RP2350 (Pico SDK), SPI.
 *
 * Implements the datasheet command set, PROM CRC4 check (AN520) and first + second order
 * temperature compensation.
 *
 * Units: pressure in mbar (= hPa), temperature in deg C, altitude in m.
 *
 * Blocking use:
 *   MS5611 baro(spi0, BAR_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN);
 *   baro.begin();
 *   baro.setOversampling(MS5611::OSR::OSR4096);
 *   if (baro.read()) printf("%.2f mbar %.2f C\n", baro.getPressure(), baro.getTemperature());
 *
 * Non-blocking use (call update() often, e.g. every loop iteration):
 *   if (baro.update()) { float alt = baro.getAltitude(); }   // true when a new P+T pair is ready
 */
#pragma once

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

class MS5611 {
public:
    enum Cmd : uint8_t {
        CMD_RESET   = 0x1E,
        CMD_CONV_D1 = 0x40, // + OSR offset
        CMD_CONV_D2 = 0x50, // + OSR offset
        CMD_ADC_READ = 0x00,
        CMD_PROM_READ = 0xA0, // + 2 * index (0..7)
    };

    enum class OSR : uint8_t { OSR256 = 0, OSR512 = 2, OSR1024 = 4, OSR2048 = 6, OSR4096 = 8 };

    static constexpr float SEA_LEVEL_MBAR = 1013.25f;

    /**
     * @param spi     SPI instance (spi0 / spi1)
     * @param csPin   chip select GPIO
     * @param sckPin  SPI clock GPIO
     * @param mosiPin SPI TX GPIO
     * @param misoPin SPI RX GPIO
     * @param spiHz   SPI clock (max 20 MHz)
     */
    MS5611(spi_inst_t* spi, uint csPin, uint sckPin, uint mosiPin, uint misoPin,
           uint32_t spiHz = 10'000'000);

    /** Initializes pins/bus, resets, reads and CRC-checks the calibration PROM. */
    bool begin(bool initBus = true);
    bool reset();
    /** Reads the 8 PROM words and verifies the CRC. */
    bool readPROM();
    bool promValid() const { return promValid_; }
    uint16_t getPROM(uint8_t index) const { return index < 8 ? prom_[index] : 0; }

    // ---------------- Configuration ----------------
    void setOversampling(OSR osr) { osr_ = osr; }
    OSR getOversampling() const { return osr_; }
    /** Conversion time for the current OSR in microseconds. */
    uint32_t conversionTimeUs() const;
    /** Reference pressure for getAltitude(). */
    void setSeaLevelPressure(float mbar) { seaLevel_ = mbar; }
    float getSeaLevelPressure() const { return seaLevel_; }
    /** Sets sea-level pressure so that the current pressure corresponds to `altitudeM`. */
    void calibrateAltitude(float altitudeM);
    /** Enable/disable second order (low temperature) compensation. Default on. */
    void setSecondOrderCompensation(bool on) { secondOrder_ = on; }
    void setSpiFrequency(uint32_t hz) { spiHz_ = hz; }

    // ---------------- Blocking reads ----------------
    /** Converts D2 (temperature) then D1 (pressure), and updates the compensated values. */
    bool read();

    // ---------------- Non-blocking reads ----------------
    /**
     * State machine: starts conversions and collects results without blocking.
     * Returns true each time a new compensated pressure/temperature pair is available.
     * @param temperatureEvery measure temperature once every N pressure samples (>= 1).
     */
    bool update(uint8_t temperatureEvery = 1);

    // ---------------- Low-level ----------------
    bool startConversion(bool pressure);  // D1 if pressure, else D2
    bool conversionDone() const;          // enough time has passed since startConversion()
    uint32_t readADC();                   // 24-bit result of the last conversion (0 if not ready)
    /** Computes compensated values from raw D1 (pressure) / D2 (temperature). */
    void compute(uint32_t D1, uint32_t D2);

    // ---------------- Results ----------------
    float getPressure() const { return pressure_ / 100.0f; }        // mbar
    float getTemperature() const { return temperature_ / 100.0f; }  // deg C
    int32_t getPressureRaw() const { return pressure_; }            // 0.01 mbar
    int32_t getTemperatureRaw() const { return temperature_; }      // 0.01 deg C
    float getAltitude() const;                                      // m, from sea-level pressure
    static float pressureToAltitude(float mbar, float seaLevelMbar = SEA_LEVEL_MBAR);

private:
    void select();
    void deselect();
    bool command(uint8_t cmd);
    static uint8_t crc4(const uint16_t prom[8]);

    spi_inst_t* spi_;
    uint cs_, sck_, mosi_, miso_;
    uint32_t spiHz_;

    OSR osr_ = OSR::OSR4096;
    uint16_t prom_[8] = {0};
    bool promValid_ = false;
    bool secondOrder_ = true;
    float seaLevel_ = SEA_LEVEL_MBAR;

    uint32_t D1_ = 0, D2_ = 0;
    int32_t pressure_ = 0;     // 0.01 mbar
    int32_t temperature_ = 0;  // 0.01 C

    // Non-blocking state
    enum class State : uint8_t { Idle, ConvTemp, ConvPress } state_ = State::Idle;
    absolute_time_t convStart_ = nil_time;
    uint32_t convTimeUs_ = 0;
    uint8_t pressCount_ = 0;
    bool haveTemp_ = false;
};
