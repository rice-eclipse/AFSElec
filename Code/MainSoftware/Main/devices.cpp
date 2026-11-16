#include "devices.h"
#include "board_pins.h"
#include "config.h"

ICM45686  imu (SENSOR_SPI, IMU_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, IMU_INT1);
KX134     acc (SENSOR_SPI, ACC_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, ACC_INT);
MMC5983MA mag (SENSOR_SPI, MAG_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, MAG_INT);
MS5611    baro(SENSOR_SPI, BAR_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN);
TMP117    temp(SENSOR_I2C, SDA, SCL, TMP117::ADDR_GND);
INA219    pwr (SENSOR_I2C, SDA, SCL, INA219::DEFAULT_ADDRESS, INA219_SHUNT_OHMS, INA219_MAX_AMPS);
SDCard    sd  (SD_SPI, SD_CHIP_SELECT, SPI_SCK_SD, SPI_MOSI_SD, SPI_MISO_SD, SD_CHIP_DETECT);

CSVWriter logFile;   // global: holds a 4 kB buffer

void deselectAllSpiDevices()
{
    // Several sensors also speak I2C/I3C; keep every CS high so none of them respond to
    // traffic meant for another device on the shared bus.
    for (uint cs : SENSOR_SPI_CS_PINS) {
        gpio_init(cs);
        gpio_set_dir(cs, GPIO_OUT);
        gpio_put(cs, 1);
    }
}
