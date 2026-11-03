#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "blink.pio.h"

#include "board_pins.h"

#include "ICM45686_test.h"
#include "KX134_test.h"
#include "MMC5983MA_test.h"
#include "MS5611_test.h"
#include "TMP117_test.h"
#include "INA219_test.h"

// ============================================================================
// Sensor bench tests: set each one to true/false to run it at boot or skip it
// ============================================================================
constexpr bool RUN_ICM45686_TEST  = true;
constexpr bool RUN_KX134_TEST     = true;
constexpr bool RUN_MMC5983MA_TEST = true;
constexpr bool RUN_MS5611_TEST    = true;
constexpr bool RUN_TMP117_TEST    = true;
constexpr bool RUN_INA219_TEST    = true;

// Verbose: true = print every PASS line and the sample readings,
//          false = print only FAIL lines, diagnostics and each test's one-line result
constexpr bool VERBOSE_ICM45686_TEST  = false;
constexpr bool VERBOSE_KX134_TEST     = false;
constexpr bool VERBOSE_MMC5983MA_TEST = false;
constexpr bool VERBOSE_MS5611_TEST    = false;
constexpr bool VERBOSE_TMP117_TEST    = false;
constexpr bool VERBOSE_INA219_TEST    = false;

// INA219 shunt resistor and maximum expected current. TODO: set these to the board's real values.
constexpr float INA219_SHUNT_OHMS = 0.002f;
constexpr float INA219_MAX_AMPS   = 3.2f;

// ============================================================================
// Sensors (pins and buses in include/board_pins.h)
// ============================================================================
ICM45686  imu (SENSOR_SPI, IMU_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, IMU_INT1);
KX134     acc (SENSOR_SPI, ACC_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, ACC_INT);
MMC5983MA mag (SENSOR_SPI, MAG_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, MAG_INT);
MS5611    baro(SENSOR_SPI, BAR_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN);
TMP117    temp(SENSOR_I2C, SDA, SCL, TMP117::ADDR_GND);
INA219    pwr (SENSOR_I2C, SDA, SCL, INA219::DEFAULT_ADDRESS, INA219_SHUNT_OHMS, INA219_MAX_AMPS);

static void deselectAllSpiDevices()
{
    // Several sensors also speak I2C/I3C; keep every CS high so none of them respond to
    // traffic meant for another device on the shared bus.
    for (uint cs : SENSOR_SPI_CS_PINS) {
        gpio_init(cs);
        gpio_set_dir(cs, GPIO_OUT);
        gpio_put(cs, 1);
    }
}

static void runSensorTests()
{
    struct Result { const char* name; bool ran; bool passed; };
    Result results[6] = {
        {"ICM45686", false, false}, {"KX134", false, false}, {"MMC5983MA", false, false},
        {"MS5611", false, false},   {"TMP117", false, false}, {"INA219", false, false},
    };

    if constexpr (RUN_ICM45686_TEST)  { results[0].ran = true; results[0].passed = ICM45686_test(imu, VERBOSE_ICM45686_TEST); }
    if constexpr (RUN_KX134_TEST)     { results[1].ran = true; results[1].passed = KX134_test(acc, VERBOSE_KX134_TEST); }
    if constexpr (RUN_MMC5983MA_TEST) { results[2].ran = true; results[2].passed = MMC5983MA_test(mag, VERBOSE_MMC5983MA_TEST); }
    if constexpr (RUN_MS5611_TEST)    { results[3].ran = true; results[3].passed = MS5611_test(baro, VERBOSE_MS5611_TEST); }
    if constexpr (RUN_TMP117_TEST)    { results[4].ran = true; results[4].passed = TMP117_test(temp, VERBOSE_TMP117_TEST); }
    if constexpr (RUN_INA219_TEST)    { results[5].ran = true; results[5].passed = INA219_test(pwr, VERBOSE_INA219_TEST); }

    printf("\n========== Sensor test summary ==========\n");
    for (const Result& r : results) {
        printf("  %-10s %s\n", r.name, !r.ran ? "skipped" : r.passed ? "PASS" : "FAIL");
    }
    printf("=========================================\n\n");
}

int main()
{
    stdio_init_all();
    sleep_ms(2000); // give the serial terminal time to connect

    deselectAllSpiDevices();
    runSensorTests();

    while (true)
    {
        //printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
