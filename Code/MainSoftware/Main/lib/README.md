# Sensor libraries

Pico SDK (RP2040 / RP2350) C++ drivers for the flight computer's sensors. Each library is
self-contained (its own folder, header, source and `CMakeLists.txt`) and depends only on the
Pico SDK. You can copy any one of them into another project on its own.

Full API docs for each driver (every method, enum, units and gotchas) are in the `README.md` in its folder. Click a library name below.

| Library     | Chip                    | Bus | Units                         |
|-------------|-------------------------|-----|-------------------------------|
| [`ICM45686`](ICM45686/README.md)  | TDK ICM-45686 6-DoF IMU | SPI | g, deg/s, deg C               |
| [`KX134`](KX134/README.md)     | Kionix KX134 ±64 g accel | SPI | g                             |
| [`MMC5983MA`](MMC5983MA/README.md) | MEMSIC magnetometer     | SPI | Gauss (1 G = 100 µT), deg C   |
| [`MS5611`](MS5611/README.md)    | TE barometer            | SPI | mbar (hPa), deg C, m          |
| [`TMP117`](TMP117/README.md)    | TI temperature sensor   | I2C | deg C                         |
| [`INA219`](INA219/README.md)    | TI current/power monitor | I2C | V, mA, mW                     |

## Using them in CMake

`Main/CMakeLists.txt` already has:

```cmake
add_subdirectory(lib)
target_link_libraries(Main ICM45686 MS5611 MMC5983MA KX134 TMP117 INA219)
```

Link only the libraries you need.

## Bench tests

Each folder also has `<Name>_test.h/.cpp`, built as a separate CMake target (`<Name>_test`), so
the driver library itself doesn't include any test code. Each test is one function that takes an
already-constructed driver, prints FAIL lines (and, with `verbose`, every PASS line) over stdio, and returns `true` if every check passed:

```cpp
bool ICM45686_test(ICM45686& imu, bool verbose = false, uint32_t samples = 10);
bool KX134_test(KX134& acc, bool verbose = false, uint32_t samples = 10);
bool MMC5983MA_test(MMC5983MA& mag, bool verbose = false, uint32_t samples = 10);
bool MS5611_test(MS5611& baro, bool verbose = false, uint32_t samples = 10);
bool TMP117_test(TMP117& tmp, bool verbose = false, uint32_t samples = 5);
bool INA219_test(INA219& ina, bool verbose = false, uint32_t samples = 5);
```

In `Main.cpp`, the `RUN_<NAME>_TEST` flags turn each test on or off at boot, `VERBOSE_<NAME>_TEST` controls how much each one prints, and a summary table
is printed at the end. Keep the board still and level, and away from magnets, while the tests run.
Each test checks that the chip responds, reads back the settings it writes, checks that readings
are physically plausible (≈1 g, Earth's magnetic field, room temperature and pressure), and exercises
the data-ready and interrupt pins, the FIFO, the continuous and non-blocking modes, and the offsets
and limits. The tests change settings, so each one finishes by restoring the defaults.

## Common pattern

All drivers use the same structure:

```cpp
Sensor s(bus, pins..., [intPin], [clock]); // constructor stores the pins and settings
s.begin();          // sets up the pins and the bus if it isn't running yet, resets, checks the ID; returns false on failure
s.setXxx(...);      // you can change settings at any time
s.read(...);        // returns false or NAN on bus failure
s.readRegister/writeRegister(...);  // raw register access
```

* `begin(bool initBus = true)` calls `spi_init`/`i2c_init` only if the peripheral isn't already
  enabled. That means several devices can share a bus, and each one can still be used on its own.
  Pass `false` if you configure the bus yourself.
* The SPI drivers set their own baud rate and SPI mode on every transaction. Devices with
  different speed limits can therefore share one bus, including with an SD card driver.
* The I2C drivers use timeouts (5 ms), so a missing device won't hang the bus.
* Interrupt pins are optional (`NO_PIN`). Each driver has helpers for routing interrupts,
  `dataReady()` and `interruptPinLevel()`. Attach GPIO IRQs yourself with `gpio_set_irq_enabled_with_callback`.

## This board (pins from `Main.cpp`)

```cpp
#include "ICM45686.h"
#include "KX134.h"
#include "MMC5983MA.h"
#include "MS5611.h"
#include "TMP117.h"
#include "INA219.h"

// Sensor SPI bus: GPIO 16-19 -> spi0.  Onboard I2C: GPIO 24/25 -> i2c0.
ICM45686  imu (spi0, IMU_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, IMU_INT1);
KX134     acc (spi0, ACC_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, ACC_INT);
MMC5983MA mag (spi0, MAG_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, MAG_INT);
MS5611    baro(spi0, BAR_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN);
TMP117    temp(i2c0, SDA, SCL, TMP117::ADDR_GND);
INA219    pwr (i2c0, SDA, SCL, 0x40, /*shunt ohms*/ 0.1f, /*max A*/ 3.2f);

int main() {
    stdio_init_all();

    // Several of these chips also speak I2C/I3C. Drive every CS high before talking on the
    // shared bus, so a floating CS can't make another chip respond.
    for (uint cs : {IMU_CS, ACC_CS, MAG_CS, BAR_CS, XTRA_SPI_CS}) {
        gpio_init(cs); gpio_set_dir(cs, GPIO_OUT); gpio_put(cs, 1);
    }

    imu.begin();  acc.begin();  mag.begin();  baro.begin();  temp.begin();  pwr.begin();

    imu.setAccelRange(ICM45686::AccelRange::G32);
    imu.setGyroRange(ICM45686::GyroRange::DPS2000);
    imu.setAccelODR(ICM45686::ODR::Hz1600);
    imu.setGyroODR(ICM45686::ODR::Hz1600);
    imu.setAccelLPF(ICM45686::LPF::ODR_4);

    acc.setRange(KX134::Range::G64);
    acc.setODR(KX134::ODR::Hz1600);

    mag.calibrateOffset();
    mag.enableAutoSetReset(true);
    mag.setBandwidth(MMC5983MA::Bandwidth::Hz400);
    mag.startContinuous(MMC5983MA::ContinuousRate::Hz100);

    baro.setOversampling(MS5611::OSR::OSR1024);
    baro.calibrateAltitude(0.0f);   // the pad is 0 m AGL

    while (true) {
        ICM45686::Data i;  imu.read(i);
        KX134::Data a;     acc.read(a);
        MMC5983MA::Data m; mag.read(m);
        if (baro.update()) printf("alt %.2f m\n", baro.getAltitude());   // non-blocking
        printf("imu %.2f %.2f %.2f g | acc %.2f g | mag %.3f G | %.2f C | %.1f mA\n",
               i.ax, i.ay, i.az, a.z, m.x, temp.readTemperature(), pwr.readCurrent_mA());
        sleep_ms(10);
    }
}
```

## Driver notes

### ICM45686
* Register map, soft-reset sequence and indirect (IPREG) access follow TDK's reference driver
  (`hal_tdk/icm456xx`). The driver reads the data byte order from `SREG_CTRL` at `begin()`.
* Low-noise mode supports ODR 6400–12.5 Hz. Low-power mode supports 400–1.5625 Hz.
* `setAccelLPF` / `setGyroLPF` / `setAntiAliasFilter` use indirect writes, so each one takes about 1 ms.
* `calibrateGyro()` stores a software bias. The device has to be still while it runs.
* FIFO and APEX (pedometer/tilt etc.) are not implemented. Low-level register access is available
  if you need them.

### KX134
* Most settings can only change while the chip is in standby (PC1 = 0). The setters handle this
  automatically: they drop to standby, write, then re-enable.
* Has 16-bit sample buffer support (`enableBuffer`, `bufferSampleCount`, `readBuffer`), up to 86 samples.
* `configureInterrupt1()` + `enableDataReadyInterrupt()` drive ACC_INT on data ready. If the
  interrupt is latched, call `clearInterrupts()` to release it.

### MMC5983MA
* Internal Control registers 0–3 are write-only. The driver keeps shadow copies, so always change
  settings through the driver's methods, not raw writes.
* `calibrateOffset()` removes the bridge offset using SET/RESET. Run it in one-shot mode, before
  `startContinuous()`. Hard-iron and soft-iron correction can be applied with `setHardIron` / `setSoftIronScale`.
* 1000 Hz continuous mode requires `Bandwidth::Hz800`. 200 Hz requires `Hz200` or higher.

### MS5611
* `read()` blocks for two conversions (up to about 18.6 ms at OSR 4096).
* `update()` is a non-blocking state machine. Call it every loop. It returns `true` when a new
  pressure/temperature pair is ready. `update(n)` re-measures temperature only every `n` pressure samples.
* The PROM is CRC4-checked at `begin()`, and first- and second-order compensation are both applied.

### TMP117
* Reading the configuration register clears the data-ready and alert flags. `dataReady()` and
  `alertStatus()` both read it.
* `saveToEEPROM()` writes the current config, limits and offset as the power-on defaults.

### INA219
* `setCalibration(shuntOhms, maxCurrentA)` computes the calibration register using the datasheet
  formula, and picks the PGA gain automatically.
* By default the calibration register is re-written before every current/power read, because the
  INA219 loses it on brown-out. Turn this off with `setAutoRecalibrate(false)`.
* `readCurrentFromShunt_mA()` calculates current directly from the shunt voltage, so it doesn't need calibration.
* **Set the real shunt resistor value** in the constructor or with `setCalibration()`. The default (0.1 Ω / 3.2 A) is a placeholder.
