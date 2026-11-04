# KX134 — ±64 g accelerometer driver (SPI)

Driver for the Kionix **KX134-1211** high-g 3-axis accelerometer, for the RP2040/RP2350 using the Pico SDK.
On this board it's the high-g accelerometer, for when the ICM45686's ±32 g isn't enough.

| | |
|---|---|
| Files | `KX134.h`, `KX134.cpp` (bench test: `KX134_test.h/.cpp`) |
| Bus | SPI mode 0, up to 10 MHz (default 8 MHz) |
| Units | acceleration in **g** |
| WHO_AM_I / COTR | `0x46` / `0x55` |
| CMake | `target_link_libraries(<target> KX134)` (needs `pico_stdlib`, `hardware_spi`) |
| Based on | KX134-1211 datasheet / TRM, Kionix TN027, SparkFun KX13X register map |

---

## Quick start

```cpp
#include "KX134.h"
#include "board_pins.h"

KX134 acc(SENSOR_SPI, ACC_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, ACC_INT);

int main() {
    stdio_init_all();
    if (!acc.begin()) printf("KX134 not found\n");

    acc.setRange(KX134::Range::G64);
    acc.setODR(KX134::ODR::Hz1600);
    acc.enableDataReadyEngine();            // required for dataReady()

    while (true) {
        KX134::Data d;
        if (acc.dataReady() && acc.read(d))
            printf("%.2f %.2f %.2f g\n", d.x, d.y, d.z);
    }
}
```

---

## Standby and operating mode

Most KX134 registers can only be written while the chip is in **standby** (`CNTL1.PC1 = 0`). The
driver handles this for you. Every configuration and interrupt setter:

1. drops the chip into standby, if it was running,
2. writes the register,
3. turns it back on.

Re-enabling waits **2 ms**, so each setter call costs about 2 ms while the chip is running. To change
many settings at once, call `enable(false)` first, then the setters (they skip the standby/re-enable
step), then `enable(true)`.

| Method | Description |
|---|---|
| `bool enable(bool on = true)` | `true` = measuring, `false` = standby. Turning it on waits 2 ms. The first sample after that may be invalid for about 1/ODR. |
| `bool isEnabled()` | Current PC1 state (cached, no bus access). |

---

## Types and constants

| Name | Meaning |
|---|---|
| `NO_PIN` | Pass as `intPin` if INT1 isn't connected |
| `WHO_AM_I_VALUE` (`0x46`), `COTR_VALUE` (`0x55`) | Identification / self-check values |
| `INT_TILT`, `INT_WAKEUP`, `INT_TAP`, `INT_BACK_SLEEP`, `INT_DATA_READY`, `INT_WATERMARK`, `INT_BUFFER_FULL`, `INT_FREEFALL` | Interrupt bits for `setInterrupt1Sources()` / `getInterruptStatus()` |
| `REG_*` | Register addresses for low-level access |

* `enum class Range`: `G8`, `G16`, `G32`, `G64`.
* `enum class ODR`: `Hz0_781`, `Hz1_563`, `Hz3_125`, `Hz6_25`, `Hz12_5`, `Hz25`, `Hz50`, `Hz100`,
  `Hz200`, `Hz400`, `Hz800`, `Hz1600`, `Hz3200`, `Hz6400`, `Hz12800`, `Hz25600`.
* `enum class BufferMode`:
  * `FIFO` stops collecting when full.
  * `Stream` overwrites the oldest sample.
  * `Trigger` collects around a trigger event.
* `struct RawData { int16_t x, y, z; }`: raw counts.
* `struct Data { float x, y, z; }`: in g, offset already subtracted.

---

## Construction and setup

### `KX134(spi, csPin, sckPin, mosiPin, misoPin, intPin = NO_PIN, spiHz = 8'000'000)`
Stores the pins and settings. It doesn't access the hardware. The parameters are the same as for the other SPI drivers; the maximum clock is 10 MHz.

### `bool begin(bool initBus = true)`
1. Sets up CS (high), the SPI bus (if `initBus` is true and the bus isn't running yet) and INT1 (input).
2. Waits 50 ms (power-on time).
3. Calls `reset()` (checks COTR), then checks `WHO_AM_I`.
4. Applies the defaults: **high-performance mode, ±64 g, 1600 Hz, enabled**.

The data-ready engine is **not** enabled by `begin()`. Call `enableDataReadyEngine()` or
`enableDataReadyInterrupt()` before using `dataReady()`.

### `bool reset()`
Runs the TN027 software-reset sequence (standby, `SRST`, waits for it to clear) and returns `true`
if `COTR` reads `0x55` afterwards. The chip is left in standby.

### `uint8_t whoAmI()` / `bool isConnected()`
`whoAmI()` reads `WHO_AM_I`. `isConnected()` returns `whoAmI() == 0x46`.

---

## Configuration

All of these handle the standby step automatically and return `true` on success.

| Method | Description |
|---|---|
| `setRange(Range)` | Full scale ±8 / 16 / 32 / 64 g. Changes `scale()`. |
| `setODR(ODR)` | Output data rate. |
| `setHighPerformance(bool hp)` | `true` = high-performance (low noise, full ODR range). `false` = low-power. |
| `setFilter(bool bypassIIR, bool cornerOdrOver2 = false)` | IIR low-pass filter. Either bypass it, or set the corner frequency to ODR/9 (`false`, the default) or ODR/2 (`true`). |
| `enableDataReadyEngine(bool enable = true)` | Turns on the data-ready engine (`CNTL1.DRDYE`). Needed for `dataReady()` and the DRDY interrupt. |
| `setSpiFrequency(uint32_t hz)` | Changes the SPI clock used from the next transaction on. |

| Getter | Returns |
|---|---|
| `getRange()`, `getODR()` | Cached settings |
| `float scale()` | g per LSB (full scale / 32768) |

---

## Interrupts

| Method | Description |
|---|---|
| `configureInterrupt1(enablePin = true, activeHigh = true, latched = true)` | Electrical setup of the INT1 pin (`INC1`). In latched mode the pin stays asserted until `clearInterrupts()`. Pulsed mode gives a short pulse instead. |
| `setInterrupt1Sources(uint8_t mask)` | Writes `INC4` (which events drive INT1). **Overwrites the whole register.** |
| `enableDataReadyInterrupt(bool enable = true)` | Shortcut that enables the DRDY engine and routes DRDY to INT1. You still need `configureInterrupt1()` to enable the pin itself. |
| `uint8_t getInterruptStatus()` | Reads `INS2` (the `INT_*` bits). |
| `void clearInterrupts()` | Reads `INT_REL`, which releases latched interrupts and their status bits. |
| `bool dataReady()` | `INS2 & INT_DATA_READY`. Requires the DRDY engine. |
| `bool interruptPinLevel()` | Raw INT1 GPIO level. |
| `uint intPin()` | INT1 GPIO number. |

Typical interrupt-driven setup:
```cpp
acc.configureInterrupt1(true, true, false);  // pin on, active high, pulsed
acc.enableDataReadyInterrupt(true);
gpio_set_irq_enabled_with_callback(acc.intPin(), GPIO_IRQ_EDGE_RISE, true, &onAccelIrq);
```

---

## Readings

| Method | Description |
|---|---|
| `bool readRaw(RawData&)` | Raw X/Y/Z counts (6-byte burst). |
| `bool read(Data&)` | Scaled to g, offset subtracted. |
| `bool read(float& x, float& y, float& z)` | Same as above, but into three separate floats. |

---

## Sample buffer (FIFO)

The chip has an on-chip buffer that holds up to **86 samples** of 16-bit X/Y/Z data.

| Method | Description |
|---|---|
| `enableBuffer(watermark = 64, mode = BufferMode::Stream, fullInterrupt = false)` | Enables the buffer in 16-bit mode. `watermark` is capped at 86 and is the sample count that triggers the watermark interrupt. `fullInterrupt` enables the buffer-full interrupt engine. |
| `disableBuffer()` | Turns the buffer off. |
| `uint16_t bufferSampleCount()` | Number of complete XYZ samples currently stored. |
| `uint16_t readBuffer(Data* out, uint16_t maxSamples)` | Reads up to `maxSamples` samples, scaled to g with offset subtracted. Returns how many it read. |
| `clearBuffer()` | Throws away the buffer contents. |

Unlike the other setters, the buffer methods write directly, **without** the standby step.

---

## Software calibration and low-level access

| Method | Description |
|---|---|
| `setOffset(x, y, z)` | Offset in g, subtracted from every scaled reading (stored in RAM). |
| `readRegisters(reg, buf, len)` | Burst read. |
| `uint8_t readRegister(reg)` | Single-byte read (returns 0 on failure). |
| `writeRegister(reg, value)` | Single-byte write. You must be in standby for most config registers. |
| `modifyRegister(reg, mask, value)` | Read-modify-write. |

---

## Notes

* At ±64 g the resolution is about 1.95 mg/LSB. At rest, expect a magnitude of about 1 g with a few mg of noise.
* The bus can be shared: the baud rate and SPI mode are set again on every transaction.
* The tap, tilt, wake-up and free-fall engines aren't wrapped by the driver. Configure them with the raw registers (`REG_INC*`, `REG_WUFTH`, `REG_BTS*`, `REG_FF*`, …) while the chip is in standby.

## Bench test

```cpp
#include "KX134_test.h"   // link KX134_test
bool KX134_test(KX134& acc, bool verbose = false, uint32_t samples = 10);
```
Run it with the board still. It checks:

* `WHO_AM_I`
* range and ODR read-back
* that the chip is still enabled after settings change
* ≈1 g at every range
* data ready within 20 ms
* that INT1 asserts, then releases after `INT_REL`
* that the buffer fills and its samples read ≈1 g

It calls `begin()` at the end to restore the defaults.
