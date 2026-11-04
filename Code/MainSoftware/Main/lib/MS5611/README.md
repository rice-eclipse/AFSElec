# MS5611 — barometric pressure / altitude driver (SPI)

Driver for the TE Connectivity **MS5611-01BA03** barometer, for the RP2040/RP2350 using the Pico SDK.
It gives temperature-compensated pressure, temperature and barometric altitude.

| | |
|---|---|
| Files | `MS5611.h`, `MS5611.cpp` (bench test: `MS5611_test.h/.cpp`) |
| Bus | SPI mode 0, up to 20 MHz (default 10 MHz). No interrupt pin. |
| Units | pressure in **mbar** (= hPa), temperature in **°C**, altitude in **m** |
| CMake | `target_link_libraries(<target> MS5611)` |
| Based on | MS5611 datasheet, TE AN520 (PROM CRC4) |

---

## Quick start

```cpp
#include "MS5611.h"
#include "board_pins.h"

MS5611 baro(SENSOR_SPI, BAR_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN);

int main() {
    stdio_init_all();
    if (!baro.begin()) printf("MS5611 not found / bad PROM\n");

    baro.setOversampling(MS5611::OSR::OSR1024);
    baro.read();                    // fresh reading
    baro.calibrateAltitude(0.0f);   // the pad is 0 m AGL

    while (true) {
        if (baro.update())          // non-blocking; true when a new value is ready
            printf("%.2f mbar  %.2f C  %.2f m\n",
                   baro.getPressure(), baro.getTemperature(), baro.getAltitude());
        // ... do other work, never block here ...
    }
}
```

---

## How the chip works

The MS5611 has no registers. It's driven by commands:

1. Send a "convert" command for **D1** (raw pressure) or **D2** (raw temperature) at the chosen oversampling (OSR).
2. Wait for the conversion to finish (0.6–9.3 ms, depending on OSR).
3. Send "ADC read" to get the 24-bit result.
4. Combine D1, D2 and the six factory calibration constants (C1–C6, stored in PROM) to get
   compensated pressure and temperature.

A full pressure and temperature measurement therefore needs **two conversions**. The driver gives you
a blocking way (`read()`) and a non-blocking way (`update()`) to do this.

---

## Types and constants

* `enum class OSR`: `OSR256`, `OSR512`, `OSR1024`, `OSR2048`, `OSR4096`. Higher means less noise and
  slower conversion. The default is `OSR4096`.

  | OSR | Conversion time used by driver | Max P+T rate (`update(1)`) | Approx. resolution |
  |---|---|---|---|
  | 256 | 650 µs | ~770 Hz | 0.065 mbar |
  | 512 | 1250 µs | ~400 Hz | 0.042 mbar |
  | 1024 | 2400 µs | ~208 Hz | 0.027 mbar |
  | 2048 | 4700 µs | ~106 Hz | 0.018 mbar |
  | 4096 | 9300 µs | ~54 Hz | 0.012 mbar (~10 cm) |

* `SEA_LEVEL_MBAR` = 1013.25: the standard atmosphere reference.
* `enum Cmd`: raw command bytes (`CMD_RESET`, `CMD_CONV_D1`, `CMD_CONV_D2`, `CMD_ADC_READ`, `CMD_PROM_READ`).

---

## Construction and setup

### `MS5611(spi, csPin, sckPin, mosiPin, misoPin, spiHz = 10'000'000)`
Stores the pins and settings. It doesn't access the hardware. There is no interrupt pin.

### `bool begin(bool initBus = true)`
1. Sets up CS (high) and the SPI bus (if `initBus` is true and the bus isn't running yet). Waits 10 ms.
2. `reset()`, then `readPROM()`, which checks the CRC.
3. Takes one blocking `read()`, so `getPressure()` is valid right after `begin()` returns.

Returns `false` if the PROM is all zeros, all ones, or fails its CRC. That usually means wiring or CS problems.

### `bool reset()`
Sends the reset command and waits 3 ms for the PROM to reload. Also resets the non-blocking state machine.

### `bool readPROM()` / `bool promValid()` / `uint16_t getPROM(uint8_t index)`
`readPROM()` reads the 8 PROM words and checks them with CRC4 (AN520). The PROM layout:

* index 0: factory data
* index 1–6: calibration constants C1–C6
* index 7: contains the CRC

`getPROM()` returns 0 for an index above 7.

---

## Configuration

| Method | Description |
|---|---|
| `setOversampling(OSR)` / `getOversampling()` | Oversampling for both D1 and D2. Takes effect from the next conversion. |
| `uint32_t conversionTimeUs()` | Wait time for the current OSR (datasheet maximum plus a margin). |
| `setSeaLevelPressure(float mbar)` / `getSeaLevelPressure()` | Reference pressure for `getAltitude()`. The default is 1013.25. |
| `calibrateAltitude(float altitudeM)` | Sets the reference pressure so that **the most recent** pressure reading corresponds to `altitudeM`. Take a fresh reading first. For altitude above the launch pad (AGL), call `calibrateAltitude(0)` on the pad. |
| `setSecondOrderCompensation(bool)` | Datasheet second-order (low-temperature) correction. On by default; it only has an effect below 20 °C. |
| `setSpiFrequency(uint32_t hz)` | Changes the SPI clock used from the next transaction on. |

---

## Reading data

### Blocking: `bool read()`
Converts D2 (temperature), waits, converts D1 (pressure), waits, then computes the result. Blocks for
**2 × `conversionTimeUs()`** (up to about 18.6 ms at OSR 4096). Returns `false` if the PROM is invalid
or an ADC read returned 0 (read too early).

### Non-blocking: `bool update(uint8_t temperatureEvery = 1)`
A state machine. Call it **as often as you can** (every loop iteration). Each call returns almost
immediately:

* If no conversion is running, it starts one and returns `false`.
* If a conversion isn't finished yet, it returns `false`.
* When a temperature conversion finishes, it reads it, starts the pressure conversion, and returns `false`.
* When a pressure conversion finishes, it reads it, computes the result, and returns **`true`**.

`temperatureEvery = N` measures temperature only once every N pressure samples, and reuses the last
temperature in between. Temperature changes slowly, so N = 5–10 nearly doubles the pressure rate.

The state machine depends on wall-clock time (`get_absolute_time`), not on how often you call it. If
you call it rarely, it simply returns `true` less often.

Calling `read()` or `reset()` cancels any `update()` cycle in progress. Don't mix them in the same loop.

### Low-level building blocks
| Method | Description |
|---|---|
| `bool startConversion(bool pressure)` | Starts a D1 conversion (`true`) or D2 conversion (`false`) at the current OSR. |
| `bool conversionDone()` | `true` once `conversionTimeUs()` has passed since `startConversion()`. |
| `uint32_t readADC()` | 24-bit result of the last conversion. **Returns 0 if it was read too early.** |
| `void compute(uint32_t D1, uint32_t D2)` | Runs the datasheet compensation on raw values and updates the results. Useful for replaying logged raw data. |

---

## Results

These getters return the most recent computed values. They don't access the bus.

| Method | Returns |
|---|---|
| `float getPressure()` | mbar (hPa) |
| `float getTemperature()` | °C |
| `int32_t getPressureRaw()` | Compensated pressure as an integer, in 0.01 mbar (e.g. 101325 = 1013.25 mbar) |
| `int32_t getTemperatureRaw()` | Compensated temperature as an integer, in 0.01 °C |
| `float getAltitude()` | m, relative to the sea-level reference pressure |
| `static float pressureToAltitude(float mbar, float seaLevelMbar = 1013.25)` | International barometric formula: `44330 × (1 − (P/P0)^0.190295)` |

---

## Notes

* **Altitude accuracy.** The barometric formula assumes a standard atmosphere. For apogee
  detection, relative altitude (after `calibrateAltitude(0)`) is what matters, and it's good to a few
  tens of cm at OSR 4096.
* **Airflow and light.** Ram-air pressure, venting of the electronics bay, and direct sunlight on
  the sensor all affect readings. Shield the sensor and vent the bay properly.
* **Bus sharing.** CS can be released during a conversion, so other devices can use the bus while
  the MS5611 converts. That's what makes `update()` practical.

## Bench test

```cpp
#include "MS5611_test.h"   // link MS5611_test
bool MS5611_test(MS5611& baro, bool verbose = false, uint32_t samples = 10);
```
It checks:

* `begin()` and the PROM CRC
* that blocking reads succeed
* pressure noise under 0.1 mbar at OSR 4096
* the non-blocking `update()`: a plausible result, at 70 Hz or faster
* altitude ≈0 m right after `calibrateAltitude(0)`

It leaves the device at OSR 4096.
