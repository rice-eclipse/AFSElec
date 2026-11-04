# INA219 — current / voltage / power monitor driver (I2C)

Driver for the TI **INA219** high-side current-shunt and power monitor, for the RP2040/RP2350 using
the Pico SDK. It measures the voltage across a shunt resistor (to get current) and the bus voltage, and
calculates power on the chip.

| | |
|---|---|
| Files | `INA219.h`, `INA219.cpp` (bench test: `INA219_test.h/.cpp`) |
| Bus | I2C, up to 400 kHz (default 400 kHz), 5 ms timeout on every transfer |
| Address | `0x40` default (A0/A1 pins select `0x40`–`0x4F`) |
| Units | volts (bus), millivolts (shunt), milliamps, milliwatts |
| CMake | `target_link_libraries(<target> INA219)` (needs `pico_stdlib`, `hardware_i2c`) |
| Based on | INA219 datasheet (SBOS448) |

---

## Quick start

```cpp
#include "INA219.h"
#include "board_pins.h"

// Shunt resistor value and maximum expected current: SET THESE TO THE REAL HARDWARE
INA219 pwr(SENSOR_I2C, SDA, SCL, INA219::DEFAULT_ADDRESS, /*shunt Ω*/ 0.002f, /*max A*/ 3.2f);

int main() {
    stdio_init_all();
    if (!pwr.begin()) printf("INA219 not found\n");

    while (true) {
        printf("%.3f V  %.1f mA  %.1f mW\n",
               pwr.readBusVoltage(), pwr.readCurrent_mA(), pwr.readPower_mW());
        sleep_ms(100);
    }
}
```

> **Set the real shunt value.** `Main.cpp` currently uses `INA219_SHUNT_OHMS = 0.002f` and
> `INA219_MAX_AMPS = 3.2f`, marked TODO. Current and power readings are only as accurate as the shunt value you give the driver.

---

## How it works

```
 Supply ──[ shunt R ]── Load ── GND
         VIN+      VIN-
```
* **Shunt voltage** is the voltage across the shunt: VIN+ − VIN−. It's measured in 10 µV steps, up to ±40/80/160/320 mV depending on the PGA gain.
* **Bus voltage** is VIN− to GND, in 4 mV steps, with a 16 V or 32 V range.
* **Current** = shunt voltage × calibration register. It's calculated on the chip and stored in the current register.
* **Power** = current × bus voltage, also calculated on the chip.

The current and power registers only hold meaningful values once the **calibration register** has
been written. The driver works out its value from the shunt resistance and maximum current you give it.

---

## Types and constants

* `DEFAULT_ADDRESS` = `0x40`.
* `enum Reg`: `REG_CONFIG`, `REG_SHUNT_VOLT`, `REG_BUS_VOLT`, `REG_POWER`, `REG_CURRENT`, `REG_CALIBRATION`.
* `enum class BusRange`: `V16`, `V32` (bus voltage full scale).
* `enum class Gain`: `mV40`, `mV80`, `mV160`, `mV320`. The shunt full-scale range; a smaller range gives finer resolution.
* `enum class ADC`. Resolution and averaging, with the conversion time for each:

  | Value | Time | | Value | Time |
  |---|---|---|---|---|
  | `Bits9` | 84 µs | | `Samples8` | 4.26 ms |
  | `Bits10` | 148 µs | | `Samples16` | 8.51 ms |
  | `Bits11` | 276 µs | | `Samples32` | 17.02 ms |
  | `Bits12` | 532 µs | | `Samples64` | 34.05 ms |
  | `Samples2` | 1.06 ms | | `Samples128` | 68.10 ms |
  | `Samples4` | 2.13 ms | | | |

  The `SamplesN` settings average N 12-bit conversions, which reduces noise.
* `enum class Mode`: `PowerDown`, `ShuntTriggered`, `BusTriggered`, `ShuntBusTriggered`, `ADCOff`,
  `ShuntContinuous`, `BusContinuous`, `ShuntBusContinuous`.

---

## Construction and setup

### `INA219(i2c, sdaPin, sclPin, address = 0x40, shuntOhms = 0.1f, maxCurrentA = 3.2f, i2cHz = 400'000)`
Stores the pins and calibration inputs. It doesn't access the hardware.

| Param | Description |
|---|---|
| `shuntOhms` | Shunt resistor value in ohms. The default of 0.1 Ω is only a placeholder. |
| `maxCurrentA` | Highest current you expect to measure. This sets the current resolution (LSB). |

### `bool begin(bool initBus = true)`
1. Sets up the I2C bus and pull-ups (if `initBus` is true and the bus isn't running yet).
2. Checks the chip answers at its address, then resets it.
3. Applies the defaults: **32 V bus range, 12-bit ADCs, shunt + bus continuous**, then
   `setCalibration(shuntOhms, maxCurrentA, autoGain = true)`.

### Other setup methods
| Method | Description |
|---|---|
| `bool reset()` | Chip reset. All registers, including calibration, go back to their power-on values. |
| `bool isConnected()` | `true` if a device ACKs a read of the config register. The INA219 has no ID register, so **any** device at that address counts as "connected". |
| `setAddress(uint8_t)` / `getAddress()` | Changes or reads the I2C address. |

---

## Configuration

| Method | Description |
|---|---|
| `setBusRange(BusRange)` | 16 V or 32 V bus full scale. |
| `setGain(Gain)` | Shunt PGA range. `setCalibration(..., autoGain=true)` picks this for you. |
| `setBusADC(ADC)` / `setShuntADC(ADC)` | Resolution and averaging for each ADC. |
| `setMode(Mode)` | Operating mode. |
| `configure(range, gain, busAdc, shuntAdc, mode)` | Writes the whole config register in one go. |

### Calibration
| Method | Description |
|---|---|
| `bool setCalibration(float shuntOhms, float maxCurrentA, bool autoGain = true)` | Calculates and writes the calibration register (details below). With `autoGain`, it also picks the smallest PGA range that fits `maxCurrentA × shuntOhms`. Returns `false` for values ≤ 0. |
| `bool restoreCalibration()` | Rewrites the stored calibration value. The INA219 loses it on reset or brown-out. |
| `setAutoRecalibrate(bool)` | On by default. The calibration register is rewritten before every current or power read, so a brown-out can't leave you reading garbage. It costs one extra I2C write per read; turn it off if bus time matters. |
| `getShuntOhms()` | Stored shunt value. |
| `getCurrentLSB()` | Real current resolution in A per LSB, after rounding. |
| `getPowerLSB()` | Power resolution in W per LSB (20 × current LSB). |
| `getCalibrationValue()` | Raw calibration register value. |

**How the calibration is calculated** (datasheet section 8.5.1):
```
Current_LSB = maxCurrentA / 32768
Cal         = trunc(0.04096 / (Current_LSB × shuntOhms)),  clamped to 2..65534, bit 0 cleared
Current_LSB = 0.04096 / (Cal × shuntOhms)                  // actual LSB after rounding
Power_LSB   = 20 × Current_LSB
```

With a very small shunt, the calibration value hits its 65534 limit and the current resolution ends
up coarser than `maxCurrentA / 32768`. For example, with **0.002 Ω and 3.2 A** (the current
`Main.cpp` values):

* the ideal calibration value would be about 209 715, so it's clamped to 65534,
* the real current resolution becomes **≈ 0.31 mA/LSB** instead of 0.098 mA/LSB,
* auto-gain picks the **±40 mV** range (3.2 A × 0.002 Ω = 6.4 mV).

This works fine; just check `getCurrentLSB()` if resolution matters to you.

---

## Readings

| Method | Returns | Notes |
|---|---|---|
| `float readShuntVoltage_mV()` | mV across the shunt | `NAN` on failure |
| `float readBusVoltage()` | V, VIN− to GND | `NAN` on failure |
| `float readLoadVoltage()` | V, bus + shunt (the supply-side voltage) | |
| `float readCurrent_mA()` | mA, from the current register | Needs calibration. `NAN` on failure. |
| `float readPower_mW()` | mW, from the power register | Needs calibration. `NAN` on failure. |
| `float readCurrentFromShunt_mA()` | mA = shunt mV / Ω | **Doesn't use the calibration register.** Useful as a cross-check. |
| `int16_t readShuntRaw()` | 10 µV/LSB | Returns 0 on failure (no error signal) |
| `uint16_t readBusRaw()` | Raw bus register | Bits 15:3 = value (4 mV/LSB), bit 1 = CNVR, bit 0 = OVF |
| `int16_t readCurrentRaw()` | Current LSBs | |
| `uint16_t readPowerRaw()` | Power LSBs | |

### Status and triggering
| Method | Description |
|---|---|
| `bool conversionReady()` | CNVR flag: a new conversion is done. Cleared when the power register is read or the config is written. |
| `bool overflow()` | OVF flag: current or power is out of range. Increase `maxCurrentA` or the gain. |
| `bool trigger()` | In the `*Triggered` modes, rewrites the config register to start one conversion. |

Low-power polling pattern:
```cpp
pwr.setMode(INA219::Mode::ShuntBusTriggered);
pwr.trigger();
while (!pwr.conversionReady()) sleep_us(100);
float mA = pwr.readCurrent_mA();
```

---

## Low-level access

| Method | Description |
|---|---|
| `readRegister(reg, uint16_t& value)` | 16-bit big-endian register read. |
| `writeRegister(reg, uint16_t value)` | 16-bit register write. |

## Notes

* The driver keeps a copy of the config register, so the setters don't read it from the chip first. If you write
  `REG_CONFIG` with the raw `writeRegister()`, the next setter call will overwrite your change.
* Shunt voltage is signed, so reverse current reads as negative.
* The 5 ms I2C timeout means a missing chip makes calls return `NAN` quickly, without hanging the bus.

## Bench test

```cpp
#include "INA219_test.h"   // link INA219_test
bool INA219_test(INA219& ina, bool verbose = false, uint32_t samples = 5);
```
Set the real shunt value first. It checks:

* config and calibration read-back
* bus voltage between 0 and 26 V
* that the current register matches shunt V / R, and the power register matches V × I
* that the overflow flag is clear
* that the conversion time follows the averaging setting
* a triggered conversion

It re-runs `begin()` at the end.
