# MMC5983MA — 3-axis magnetometer driver (SPI)

Driver for the MEMSIC **MMC5983MA** 18-bit magnetometer, for the RP2040/RP2350 using the Pico SDK.

| | |
|---|---|
| Files | `MMC5983MA.h`, `MMC5983MA.cpp` (bench test: `MMC5983MA_test.h/.cpp`) |
| Bus | SPI (bit-banged by default, see below), up to 10 MHz (default 8 MHz; bit-bang is capped at 4 MHz) |
| Units | magnetic field in **Gauss** (1 G = 100 µT; Earth's field ≈ 0.25–0.65 G), temperature in **°C** |
| Product ID | `0x30` |
| Resolution | 18-bit, 16384 counts/G, zero field = 131072 (2¹⁷) |
| CMake | `target_link_libraries(<target> MMC5983MA)` |
| Based on | MMC5983MA datasheet, SparkFun MMC5983MA library conventions |

---

## Quick start

```cpp
#include "MMC5983MA.h"
#include "board_pins.h"

MMC5983MA mag(SENSOR_SPI, MAG_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, MAG_INT);

int main() {
    stdio_init_all();
    if (!mag.begin()) printf("MMC5983MA not found\n");

    mag.calibrateOffset();                              // one-shot mode, before continuous
    mag.enableAutoSetReset(true);
    mag.setBandwidth(MMC5983MA::Bandwidth::Hz400);
    mag.startContinuous(MMC5983MA::ContinuousRate::Hz100);

    while (true) {
        MMC5983MA::Data m;
        if (mag.dataReady() && mag.read(m))
            printf("%.4f %.4f %.4f G  heading %.1f\n", m.x, m.y, m.z, MMC5983MA::heading(m));
        sleep_ms(5);
    }
}
```

---

## Why the bus handling is unusual

The MMC5983MA's **I2C interface is always on** and can't be disabled. Whenever its CS is high, it
listens on SCK (as SCL) and MOSI (as SDA). Traffic for other chips on the shared SPI bus can look
like an I2C START to it. The chip then gets stuck in a fake I2C transaction and stops answering SPI.

The driver defends against this in four ways, all on by default:

1. **Bit-banged transfers** (`setBitBang`). The RP2350 SPI block changes MOSI on the same SCK edge,
   so with any skew that can look like a START. Bit-banging changes MOSI only a quarter period after
   SCK has fallen. A 7-byte field read takes about 15–30 µs.
2. **I2C guard** (`setI2CGuard`). Sends an I2C STOP on SCK/MOSI before every transaction (about 3–4 µs),
   which ends any fake transaction in progress.
3. **SPI mode 0** (`setSpiMode`). SCK idles low, so idle bus states can't form a START. The datasheet
   describes mode 3, but mode 0 works.
4. **Recovery** (`recoverInterface` / `recover`). Clocks the fake I2C transaction out, and resets the
   chip if it still won't respond.

Leave these defaults alone unless the chip has a bus to itself.

---

## Types and constants

| Name | Meaning |
|---|---|
| `NO_PIN` | Pass as `intPin` if INT isn't connected |
| `PRODUCT_ID_VALUE` (`0x30`) | Expected product ID |
| `ZERO_FIELD_18BIT` (131072), `COUNTS_PER_GAUSS_18BIT` (16384) | Conversion constants |
| `READ_FLAGS_SPARKFUN` (`0x80`), `READ_FLAGS_PX4` (`0xC0`) | Read-address flag options, see `setReadFlags` |
| `STATUS_MEAS_M_DONE`, `STATUS_MEAS_T_DONE`, `STATUS_OTP_READ_DONE` | `STATUS` register bits |

* `enum class Bandwidth`: `Hz100`, `Hz200`, `Hz400`, `Hz800`. This also sets the measurement time: 8 / 4 / 2 / 0.5 ms.
* `enum class ContinuousRate`: `Off`, `Hz1`, `Hz10`, `Hz20`, `Hz50`, `Hz100`, `Hz200`, `Hz1000`.
  * 200 Hz needs a bandwidth of 200 Hz or higher.
  * 1000 Hz needs `Bandwidth::Hz800`.
* `enum class PeriodicSet`: `Every1`, `Every25`, `Every75`, `Every100`, `Every250`, `Every500`, `Every1000`, `Every2000`.
* `struct RawData { uint32_t x, y, z; }`: unsigned 18-bit values, where 131072 means zero field.
* `struct Data { float x, y, z; }`: in Gauss, with all corrections applied.

---

## Construction and setup

### `MMC5983MA(spi, csPin, sckPin, mosiPin, misoPin, intPin = NO_PIN, spiHz = 8'000'000)`
Stores the pins and settings. It doesn't access the hardware.

### `bool begin(bool initBus = true)`
1. Sets up CS (high), the SPI bus (if `initBus` is true and the bus isn't running yet) and INT (input). Waits 10 ms.
2. Up to 3 times: `recoverInterface()`, `reset()`, then checks the product ID reads `0x30` three times in a row.
3. If that still fails, tries `autoDetectBus()` (other SPI modes and speeds).
4. Runs `performSet()` to clear any leftover magnetization.

Afterwards the chip is in **one-shot mode** with a 100 Hz bandwidth. Software offsets and hard-iron and
soft-iron corrections are **not** cleared by `begin()`.

### `bool reset()`
Software reset (`CTRL1.SW_RST`), then a 20 ms wait. Clears the driver's copies of the control registers
(bandwidth goes back to 100 Hz, continuous mode off).

### `uint8_t productId()` / `bool isConnected()`
`productId()` reads the product ID register. `isConnected()` returns `productId() == 0x30`. Bit-banged
reads can't detect a missing chip on their own, so use `isConnected()` as your health check.

---

## Configuration

| Method | Description |
|---|---|
| `setBandwidth(Bandwidth)` / `getBandwidth()` | Decimation filter bandwidth. A higher bandwidth means faster measurements with more noise. |
| `uint32_t measurementTimeUs()` | Measurement time for the current bandwidth (8000 / 4000 / 2000 / 500 µs). |
| `setChannelInhibit(bool x, bool yz)` | Turns channels off to make measurements faster. Y and Z can only be turned off together. |
| `enableAutoSetReset(bool enable = true)` | Automatic SET/RESET before each measurement. Recommended, because it removes the bridge offset and temperature drift. **In continuous mode this roughly halves the real output rate** (100 Hz setting gives about 54 Hz), so pick the next rate up. |
| `setPeriodicSet(bool enable, PeriodicSet period = Every100)` | In continuous mode, does a SET once every N measurements. Requires auto set/reset. |
| `startContinuous(ContinuousRate)` | Starts continuous measurement. `Off` is the same as `stopContinuous()`. |
| `stopContinuous()` / `isContinuous()` | Goes back to one-shot mode / checks the mode. |

### Bus options

| Method | Default | Description |
|---|---|---|
| `setSpiFrequency(hz)` / `getSpiFrequency()` | 8 MHz | SPI clock. Bit-banged transfers are capped at 4 MHz. |
| `setSpiMode(0 or 3)` / `getSpiMode()` | 0 | SPI mode (applies to hardware SPI). Use 0 on a shared bus. |
| `setReadFlags(flags)` / `getReadFlags()` | `0x80` | Bits OR'd into the register address on reads. Either value works. |
| `setBitBang(bool)` / `getBitBang()` | on | Bit-banged transfers (`true`) or the hardware SPI block (`false`). |
| `setI2CGuard(bool)` / `getI2CGuard()` | on | I2C STOP before every transaction. |
| `bool autoDetectBus()` | | Tries modes 0 and 3, at the set speed and then at 1 MHz. Keeps the first combination that reads the ID 3 times. `begin()` calls it as a fallback. |
| `void sendI2CStop()` | | Sends one I2C STOP on SCK/MOSI with CS high. |
| `void recoverInterface()` | | Sends 9 clocks plus a STOP to free a stuck I2C state machine. Safe at any time. |
| `bool recover()` | | `recoverInterface()`. If the chip still doesn't answer, it resets the chip and restores the current settings. Call this if `isConnected()` starts returning `false` mid-flight. |
| `misoPin()`, `spiInstance()` | | Accessors. |

---

## SET/RESET and calibration

The sensor's magnetoresistive bridges have an offset that flips sign between the SET and RESET
states. Measuring once in each state lets the driver separate the real field from the offset.

| Method | Description |
|---|---|
| `performSet()` / `performReset()` | Sends a SET or RESET pulse (1 ms each). |
| `measureSetReset(RawData& afterSet, RawData& afterReset)` | Takes one measurement after a SET and one after a RESET. Field = (set − reset)/2, offset = (set + reset)/2. One-shot mode only; it returns `false` in continuous mode. Auto set/reset is turned off during the measurement and restored afterwards. |
| `bool calibrateOffset()` | Uses `measureSetReset` to measure the bridge offset and stores it. **Run it in one-shot mode, before `startContinuous()`.** |
| `setOffset(x, y, z)` / `getOffset(...)` | Bridge offset in G (set it manually or read it back). |
| `setHardIron(x, y, z)` / `getHardIron(...)` | Hard-iron offset in G, from magnetized parts on the board or vehicle. |
| `setSoftIronScale(x, y, z)` / `getSoftIronScale(...)` | Per-axis scale factors (default 1). |
| `bool calibrateHardIron(durationMs = 20000, float* fieldStrength = nullptr)` | Min/max calibration. **Slowly rotate the board through every orientation** (figure-eights on all axes) for `durationMs`. Sets the hard-iron offset to the middle of each axis's range, and the soft-iron scale so all axes match. Run `calibrateOffset()` first, in one-shot mode. Returns `false` (and keeps the old values) if fewer than 50 samples were taken or any axis varied by less than 0.1 G. `fieldStrength` receives the average field magnitude (expect 0.25–0.65 G). |

The conversion used for every scaled reading is:
```
field = ((raw − 131072) / 16384 − offset − hardIron) × softIronScale
```

All of these corrections are kept in RAM and lost on power-off. Save the hard-iron and soft-iron results
yourself (for example to flash) if you want to reuse them.

---

## Interrupts

| Method | Description |
|---|---|
| `enableInterrupt(bool enable = true)` | The INT pin pulses when a measurement finishes. |
| `clearInterrupt()` | Clears and re-arms the measurement-done flags (by writing 1s to `STATUS`). |
| `uint8_t getStatus()` | Reads `STATUS`. |
| `bool dataReady()` | `Meas_M_Done`. **It stays set until the data has been read.** In continuous mode: call `read()`, then `clearInterrupt()` to re-arm INT, then poll again. |
| `interruptPinLevel()`, `intPin()` | Raw INT level and pin number. |

---

## Readings

| Method | Mode | Description |
|---|---|---|
| `startMeasurement()` | one-shot | Starts a single measurement (doesn't wait). |
| `readSingle(Data&, timeoutMs = 20)` | one-shot | Starts a measurement, waits for it (`measurementTimeUs()` plus polling), reads it, clears the flag. Blocks for 0.5–8 ms. |
| `readSingleRaw(RawData&, timeoutMs = 20)` | one-shot | Same, but returns the raw value. |
| `readRaw(RawData&)` | either | Reads the current output registers (18-bit). |
| `read(Data&)` | either | Reads the current output registers and converts to G with all corrections. Use this in continuous mode, or after `dataReady()`. |
| `float readTemperature(timeoutMs = 20)` | one-shot | Starts a temperature measurement and returns °C (`raw × 0.8 − 75`, coarse 0.8 °C steps). Returns `NAN` in continuous mode or on timeout. |
| `static float heading(const Data&)` | — | Compass heading 0–360° from `atan2(y, x)`. Only valid when the sensor is level (no tilt compensation). |

---

## Low-level access

The control registers (`CTRL0`–`CTRL3`) are **write-only**. The driver keeps shadow copies of them,
so always change settings through the driver's methods. A raw `writeRegister` to a control register
gets the shadow copy out of sync.

| Method | Description |
|---|---|
| `readRegisters(reg, buf, len)` | Burst read. |
| `uint8_t readRegister(reg)` | Single-byte read. |
| `writeRegister(reg, value)` | Single-byte write. |

## Bench test

```cpp
#include "MMC5983MA_test.h"   // link MMC5983MA_test
bool MMC5983MA_test(MMC5983MA& mag, bool verbose = false, uint32_t samples = 10);
```
Keep the board away from magnets and motors. It checks:

* the product ID
* the bit-banged link: 1000 error-free ID reads
* one-shot reads at every bandwidth
* noise under 5 mG at 100 Hz bandwidth
* that SET/RESET flips the sensor, and the `calibrateOffset()` result is plausible
* that the INT pin signals measurement done
* temperature between −10 and 70 °C
* that `recover()` restores communication

If the chip doesn't respond, it runs diagnostics. These check whether MISO is ever driven, try
different pull settings, and look for a working SPI mode and speed. It calls `begin()` at the end.
