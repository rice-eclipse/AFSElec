# ICM45686 — 6-axis IMU driver (SPI)

Driver for the TDK InvenSense **ICM-45686** 3-axis accelerometer + 3-axis gyroscope, for the
RP2040/RP2350 using the Pico SDK.

| | |
|---|---|
| Files | `ICM45686.h`, `ICM45686.cpp` (bench test: `ICM45686_test.h/.cpp`) |
| Bus | SPI mode 0, up to 24 MHz (default 10 MHz) |
| Units | acceleration in **g**, angular rate in **deg/s**, temperature in **°C** |
| WHO_AM_I | `0xE9` |
| CMake | `target_link_libraries(<target> ICM45686)` (needs `pico_stdlib`, `hardware_spi`) |
| Based on | TDK datasheet + TDK reference driver `hal_tdk/icm456xx` |

---

## Quick start

```cpp
#include "ICM45686.h"
#include "board_pins.h"

ICM45686 imu(SENSOR_SPI, IMU_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, IMU_INT1);

int main() {
    stdio_init_all();
    if (!imu.begin()) printf("IMU not found\n");

    imu.setAccelRange(ICM45686::AccelRange::G32);
    imu.setGyroRange(ICM45686::GyroRange::DPS2000);
    imu.setAccelODR(ICM45686::ODR::Hz1600);
    imu.setGyroODR(ICM45686::ODR::Hz1600);
    imu.calibrateGyro();                    // board must be still (~0.5 s)

    while (true) {
        ICM45686::Data d;
        if (imu.dataReady() && imu.read(d))
            printf("%.3f %.3f %.3f g | %.1f %.1f %.1f dps | %.1f C\n",
                   d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.temperature);
    }
}
```

---

## Types and constants

### Constants
| Name | Value | Meaning |
|---|---|---|
| `NO_PIN` | `0xFFFFFFFF` | Pass as `intPin` if INT1 isn't connected |
| `WHO_AM_I_VALUE` | `0xE9` | Expected `WHO_AM_I` register value |
| `INT_FIFO_FULL`, `INT_FIFO_THS`, `INT_DRDY`, `INT_AUX1_DRDY`, `INT_AP_FSYNC`, `INT_RESET_DONE` | bit masks | Interrupt sources / status bits in `INT1_CONFIG0` / `INT1_STATUS0` |
| `REG_*` | | Direct (bank 0) register addresses, for `readRegister`/`writeRegister` |
| `IREG_*` | | Indirect (IPREG) register addresses, for `readIndirect`/`writeIndirect` |

### `enum class AccelRange`
`G32`, `G16`, `G8`, `G4`, `G2` (full scale ±32 g … ±2 g).

### `enum class GyroRange`
`DPS4000`, `DPS2000`, `DPS1000`, `DPS500`, `DPS250`, `DPS125`, `DPS62_5`, `DPS31_25`, `DPS15_625`.

### `enum class ODR` (output data rate)
`Hz6400`, `Hz3200`, `Hz1600`, `Hz800`, `Hz400`, `Hz200`, `Hz100`, `Hz50`, `Hz25`, `Hz12_5`,
`Hz6_25`, `Hz3_125`, `Hz1_5625`.

* Low-noise mode supports **6400 – 12.5 Hz**.
* Low-power mode supports **400 – 1.5625 Hz**.
* So 6400–800 Hz are low-noise only, and 6.25–1.5625 Hz are low-power only.

### `enum class AccelMode` / `GyroMode`
* `AccelMode`: `Off`, `LowPower`, `LowNoise`
* `GyroMode`: `Off`, `Standby`, `LowPower`, `LowNoise`

### `enum class LPF` (UI-path low-pass filter, low-noise mode)
`Off`, `ODR_4`, `ODR_8`, `ODR_16`, `ODR_32`, `ODR_64`, `ODR_128`. The bandwidth is ODR divided by N.

### `struct RawData`
`int16_t ax, ay, az, gx, gy, gz, temp`: the raw signed register values.

### `struct Data`
`float ax, ay, az` (g), `gx, gy, gz` (deg/s), `temperature` (°C). Software offsets are already subtracted.

---

## Construction and setup

### `ICM45686(spi, csPin, sckPin, mosiPin, misoPin, intPin = NO_PIN, spiHz = 10'000'000)`
Stores the pins and settings. It does not touch the hardware, so it's safe to construct as a global.

| Param | Description |
|---|---|
| `spi` | `spi0` or `spi1` |
| `csPin` | Chip-select GPIO (driven by the driver, not the SPI peripheral) |
| `sckPin`, `mosiPin`, `misoPin` | SPI pins |
| `intPin` | INT1 GPIO, or `NO_PIN` |
| `spiHz` | SPI clock, max 24 MHz |

### `bool begin(bool initBus = true)`
1. Sets CS as an output and drives it high.
2. If `initBus` is true: calls `spi_init` (only if the SPI block isn't already enabled), then
   assigns the SCK/MOSI/MISO pin functions. Pass `false` if you set up the bus yourself.
3. Configures `intPin` as an input (if given).
4. Does a dummy read (to put the chip into SPI mode), then `reset()`, then checks `WHO_AM_I`.
5. Reads the data byte order from `IREG_SREG_CTRL`.
6. Applies the defaults: **±16 g, ±2000 dps, 1600 Hz ODR, data-ready status enabled, accel and gyro in low-noise mode**.
7. Waits 30 ms for the gyro to start up.

Returns `false` if the chip didn't respond, the reset failed, or a config write failed.
If you call `begin()` again, it reapplies the ranges and ODRs that are currently stored, not the factory defaults.

### `bool reset()`
Soft-resets the chip (`MISC2.SOFT_RST`). It saves and restores the SPI slew-rate and interface-override
registers around the reset, and returns `true` if the `RESET_DONE` status bit was set. You lose all
chip configuration; the stored ranges and ODRs inside the driver object are kept.

### `uint8_t whoAmI()` / `bool isConnected()`
`whoAmI()` reads the `WHO_AM_I` register. `isConnected()` returns `whoAmI() == 0xE9`.

---

## Configuration

Every setter returns `true` on success.

| Method | Description |
|---|---|
| `setAccelRange(AccelRange)` | Accel full scale. Changes `accelScale()`. |
| `setGyroRange(GyroRange)` | Gyro full scale. Changes `gyroScale()`. |
| `setAccelODR(ODR)` / `setGyroODR(ODR)` | Output data rates. Make sure the rate is valid for the current power mode (see `ODR`). |
| `setAccelMode(AccelMode)` / `setGyroMode(GyroMode)` | Power mode. Waits 200 µs afterwards, as the datasheet requires. |
| `setAccelLPF(LPF)` / `setGyroLPF(LPF)` | UI low-pass filter bandwidth. Uses indirect writes, so each call takes **~1 ms**. |
| `setAntiAliasFilter(accelEnable, gyroEnable)` | Turns the FIR anti-alias filters on or off (off at power-up). Indirect writes, ~2 ms. |
| `setSpiSlewRate(uint8_t slew)` | SPI pad slew, 0 (38 ns) … 6 (0.5 ns). `begin()`/`reset()` set this to 2 (10 ns). |
| `setSpiFrequency(uint32_t hz)` | Changes the SPI clock used from the next transaction on. |

Getters:

| Method | Returns |
|---|---|
| `getAccelRange()`, `getGyroRange()`, `getAccelODR()`, `getGyroODR()` | The last value set (cached, no bus access) |
| `float accelScale()` | g per LSB (full scale / 32768) |
| `float gyroScale()` | deg/s per LSB (full scale / 32768) |

---

## Interrupts / data ready

| Method | Description |
|---|---|
| `configureInterrupt1(activeHigh = true, latched = false, pushPull = true)` | Electrical setup of the INT1 pin. |
| `setInterrupt1Sources(uint8_t mask)` | Writes `INT1_CONFIG0` with a combination of `INT_*` bits. **This overwrites the whole register**, so include `INT_DRDY` if you still want `dataReady()` to work. |
| `enableDataReadyInterrupt(bool enable = true)` | Sets or clears only the `INT_DRDY` bit. `begin()` turns it on. |
| `uint8_t getInterruptStatus()` | Reads `INT1_STATUS0`. **Reading clears every flag in it.** |
| `bool dataReady()` | `true` if a new sample is available. It calls `getInterruptStatus()`, so it also clears the other flags. Only works while DRDY is enabled. |
| `bool interruptPinLevel()` | Raw GPIO level of INT1 (`false` if `intPin` is `NO_PIN`). |
| `uint intPin()` | The INT1 GPIO number. |

The driver doesn't attach a GPIO IRQ for you. Use `gpio_set_irq_enabled_with_callback(imu.intPin(), ...)`
if you want an interrupt handler.

---

## Readings

All read methods return `false` (or `NAN`) if the SPI transfer is short.

| Method | Description |
|---|---|
| `bool readRaw(RawData&)` | One 14-byte burst: raw accel, gyro and temperature. |
| `bool read(Data&)` | Same burst, scaled to g / dps / °C, with offsets subtracted. **Use this for flight data**, because all axes come from the same sample. |
| `bool readAccel(float& x, float& y, float& z)` | Accel only (6 bytes), in g. |
| `bool readGyro(float& x, float& y, float& z)` | Gyro only (6 bytes), in deg/s. |
| `float readTemperature()` | Die temperature in °C (`raw / 128 + 25`), or `NAN` on failure. |

---

## Software calibration

The offsets are kept in the driver (in RAM), not written to the chip, and are subtracted from every scaled reading.

| Method | Description |
|---|---|
| `setAccelOffset(x, y, z)` | Accel offset in g. |
| `setGyroOffset(x, y, z)` | Gyro offset in deg/s. |
| `bool calibrateGyro(samples = 500, delayUs = 1000)` | Clears the gyro offset, averages `samples` readings, then stores the average as the new offset. The board **must be completely still**. With the defaults this takes about 0.5 s. Returns `false` if `samples == 0` or a read fails. |

---

## Low-level access

| Method | Description |
|---|---|
| `readRegisters(reg, buf, len)` | Burst read starting at `reg`. |
| `uint8_t readRegister(reg)` | Single-byte read. Returns 0 on failure (no error signal). |
| `writeRegister(reg, value)` | Single-byte write. |
| `readIndirect(addr, value)` / `writeIndirect(addr, value)` | Access to the indirect IPREG register banks (16-bit addresses, see `IREG_*`). Adds the ~4 µs spacing the chip needs. |

---

## Notes and gotchas

* **The SPI bus can be shared.** The baud rate and SPI mode are set again at the start of every transaction.
* **Drive every CS on the bus high before calling `begin()`.** The ICM45686 also has an I2C/I3C interface, so a floating CS can make it react to traffic meant for another device.
* **Byte order** is read from the chip at `begin()`, so `toInt16` handles both little- and big-endian data.
* **Not implemented:** the hardware FIFO, APEX features (pedometer, tilt, etc.) and the hardware offset registers. Use the low-level access methods if you need them.

## Bench test

```cpp
#include "ICM45686_test.h"   // link ICM45686_test
bool ICM45686_test(ICM45686& imu, bool verbose = false, uint32_t samples = 10);
```
Run it with the board lying still. It checks:

* `WHO_AM_I`
* config and power-mode read-back
* that scaling stays consistent across range changes
* the LPF indirect writes
* data ready within 20 ms, and that the INT1 pin asserts
* gyro under 2 dps after `calibrateGyro()`
* temperature between 0 and 70 °C

It prints PASS/FAIL over stdio and calls `begin()` at the end to restore the defaults.
