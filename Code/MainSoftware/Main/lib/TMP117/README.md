# TMP117 — precision temperature sensor driver (I2C)

Driver for the TI **TMP117** digital temperature sensor (±0.1 °C accuracy, 0.0078125 °C resolution),
for the RP2040/RP2350 using the Pico SDK.

| | |
|---|---|
| Files | `TMP117.h`, `TMP117.cpp` (bench test: `TMP117_test.h/.cpp`) |
| Bus | I2C, up to 400 kHz (default 400 kHz), 5 ms timeout on every transfer |
| Address | `0x48` (ADD0→GND, used on this board), `0x49` (VCC), `0x4A` (SDA), `0x4B` (SCL) |
| Units | **°C** |
| Device ID | `0x117` |
| CMake | `target_link_libraries(<target> TMP117)` (needs `pico_stdlib`, `hardware_i2c`) |
| Based on | TMP117 datasheet (SNOSD82) |

---

## Quick start

```cpp
#include "TMP117.h"
#include "board_pins.h"

TMP117 temp(SENSOR_I2C, SDA, SCL, TMP117::ADDR_GND);

int main() {
    stdio_init_all();
    if (!temp.begin()) printf("TMP117 not found\n");

    temp.setAveraging(TMP117::Averaging::AVG8);
    temp.setConversionCycle(TMP117::ConversionCycle::C1);   // new value every 125 ms

    while (true) {
        if (temp.dataReady()) printf("%.3f C\n", temp.readTemperature());
        sleep_ms(10);
    }
}
```

---

## Types and constants

| Name | Value | Meaning |
|---|---|---|
| `NO_PIN` | `0xFFFFFFFF` | No ALERT pin connected |
| `DEVICE_ID_VALUE` | `0x0117` | Expected device ID |
| `RESOLUTION` | `0.0078125` | °C per LSB (1/128) |
| `ADDR_GND/VCC/SDA/SCL` | `0x48`–`0x4B` | I2C address, set by the ADD0 pin |
| `CFG_HIGH_ALERT`, `CFG_LOW_ALERT`, `CFG_DATA_READY`, `CFG_EEPROM_BUSY` | bits | Flags in the configuration register (see `readConfig()`) |

* `enum class Mode`:
  * `Continuous`: converts repeatedly.
  * `Shutdown`: low power, no conversions.
  * `OneShot`: one conversion, then back to shutdown.
* `enum class Averaging`: `None`, `AVG8`, `AVG32`, `AVG64`. How many conversions are averaged into each result.
* `enum class AlertMode`:
  * `Alert`: flags latch until they're read.
  * `Therm`: hysteresis behaviour, like a thermostat.
* `enum class ConversionCycle` `C0`…`C7`. The real cycle time is the longer of the cycle setting and the averaging time:

  |      | None | AVG8 | AVG32 | AVG64 |
  |------|------|------|-------|-------|
  | C0   | 15.5 ms | 125 ms | 500 ms | 1 s |
  | C1   | 125 ms | 125 ms | 500 ms | 1 s |
  | C2   | 250 ms | 250 ms | 500 ms | 1 s |
  | C3   | 500 ms | 500 ms | 500 ms | 1 s |
  | C4   | 1 s | 1 s | 1 s | 1 s |
  | C5   | 4 s | 4 s | 4 s | 4 s |
  | C6   | 8 s | 8 s | 8 s | 8 s |
  | C7   | 16 s | 16 s | 16 s | 16 s |

  The power-on default (unless it's been changed in EEPROM) is **continuous, AVG8, C4 (1 s)**.

---

## Construction and setup

### `TMP117(i2c, sdaPin, sclPin, address = ADDR_GND, i2cHz = 400'000, alertPin = NO_PIN)`
Stores the pins and settings. It doesn't access the hardware. Note that **`alertPin` comes after `i2cHz`**.

### `bool begin(bool initBus = true)`
1. If `initBus` is true: calls `i2c_init` (only if the I2C block isn't already enabled), assigns the
   SDA/SCL pin functions and enables the internal pull-ups.
2. Sets up the ALERT pin as an input with a pull-up (ALERT is open-drain).
3. Waits 2 ms, checks the device ID, then soft-resets.

After the reset, the configuration comes from the chip's EEPROM (the power-on defaults).

### Other setup methods
| Method | Description |
|---|---|
| `bool reset()` | Soft reset (reloads the configuration from EEPROM). Waits 2 ms. |
| `uint16_t deviceId()` | Device ID, bits 11:0 (`0x117`). |
| `uint8_t revision()` | Silicon revision, bits 15:12. |
| `bool isConnected()` | `deviceId() == 0x117`. |
| `setAddress(uint8_t)` / `getAddress()` | Changes or reads the I2C address the driver uses. |

---

## Configuration

| Method | Description |
|---|---|
| `setMode(Mode)` / `Mode getMode()` | Conversion mode. After a one-shot conversion finishes, `getMode()` may still report `OneShot`. |
| `setAveraging(Averaging)` / `getAveraging()` | Number of conversions averaged per result. |
| `setConversionCycle(ConversionCycle)` / `getConversionCycle()` | Cycle time (see the table above). |
| `bool triggerOneShot()` | Starts one conversion. The chip goes back to shutdown afterwards. |
| `setOffset(float degC)` / `float getOffset()` | Offset that the chip itself adds to every result (±256 °C, 1/128 °C steps). Use it for a one-point calibration. |

---

## Alerts

The chip compares each result with high and low limits and sets alert flags. The flags can optionally drive the ALERT pin.

| Method | Description |
|---|---|
| `setHighLimit(°C)` / `getHighLimit()` | High-alert threshold. |
| `setLowLimit(°C)` / `getLowLimit()` | Low-alert threshold. |
| `setAlertMode(AlertMode)` | `Alert` mode: a flag is set when a limit is crossed, and stays set until it's read. `Therm` mode: the flag is set above the high limit and cleared below the low limit (hysteresis). |
| `setAlertPolarity(bool activeHigh)` | ALERT pin polarity. |
| `setAlertPinDataReady(bool)` | `true`: the ALERT pin signals data ready. `false`: it signals the alert flags. |
| `bool interruptPinLevel()` | Raw ALERT GPIO level (`false` if there's no pin). |

---

## Readings

| Method | Description |
|---|---|
| `float readTemperature()` | Most recent result in °C, or `NAN` on bus failure. Doesn't start a conversion. |
| `int16_t readTemperatureRaw()` | Raw result (×1/128 °C). Returns `0x8000` (−256 °C) on failure. |
| `float readTemperatureOneShot(timeoutMs = 1200)` | Starts a one-shot conversion and waits for it to finish (up to 1 s with AVG64). Returns `NAN` on timeout. Use it for low-power, occasional readings. |
| `bool dataReady()` | `true` if a new result has arrived since the configuration register was last read. |
| `bool alertStatus(bool& high, bool& low)` | Returns the high and low alert flags. |
| `uint16_t readConfig()` | Raw configuration register. |

### Important: reading the configuration register clears the flags
In the TMP117, **any read of the configuration register clears the data-ready and alert flags**.
The following all read it:
`dataReady()`, `alertStatus()`, `readConfig()`, `getMode()`, `getAveraging()`, `getConversionCycle()`,
and **every setter** (they read-modify-write it). So:

* Calling `getAveraging()` (or any setter) between two `dataReady()` polls can make you miss a sample.
* If you check both data-ready and alerts, read the register once with `readConfig()` and test the
  `CFG_*` bits yourself.

---

## EEPROM

The TMP117 has EEPROM for its power-on configuration, plus three general-purpose 16-bit words.
EEPROM has limited write endurance, so **don't write it in a loop**. Each write takes about 7 ms.

| Method | Description |
|---|---|
| `bool saveToEEPROM()` | Saves the current config, limits and offset as the power-on defaults (unlocks, writes each one, waits, locks again). |
| `bool readEEPROM(index, value)` | Reads general-purpose word 1–3. |
| `bool writeEEPROM(index, value)` | Writes general-purpose word 1–3. You could use these for a board serial number or calibration data. |

---

## Low-level access

| Method | Description |
|---|---|
| `readRegister(reg, uint16_t& value)` | 16-bit big-endian register read. |
| `writeRegister(reg, uint16_t value)` | 16-bit register write. |
| `modifyConfig(mask, value)` | Read-modify-write of the configuration register. Only writable bits are written, and the reset bit is never set. |

## Notes

* For measuring air temperature, keep the TMP117 away from heat sources such as regulators and the MCU. It
  measures its own die temperature, which follows the PCB temperature closely.
* The 5 ms I2C timeout means a missing or unplugged sensor makes calls return `NAN` or `false` quickly, without hanging the bus.

## Bench test

```cpp
#include "TMP117_test.h"   // link TMP117_test
bool TMP117_test(TMP117& tmp, bool verbose = false, uint32_t samples = 5);
```
It checks:

* the device ID
* mode, averaging and cycle read-back
* temperature between 0 and 60 °C
* data ready within 300 ms
* one-shot mode, including that conversions stop afterwards
* that a +5 °C offset shows up in the result
* limit read-back, and that the high-alert flag sets

It doesn't write EEPROM, and it soft-resets the device at the end.
