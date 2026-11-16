/**
 * devices.h - The board's hardware objects (pins and buses in include/board_pins.h).
 */
#pragma once

#include "ICM45686.h"
#include "KX134.h"
#include "MMC5983MA.h"
#include "MS5611.h"
#include "TMP117.h"
#include "INA219.h"
#include "SDCard.h"

extern ICM45686  imu;    // 6-DoF IMU          (sensor SPI)
extern KX134     acc;    // ±64 g accelerometer (sensor SPI)
extern MMC5983MA mag;    // magnetometer       (sensor SPI)
extern MS5611    baro;   // barometer          (sensor SPI)
extern TMP117    temp;   // temperature        (onboard I2C)
extern INA219    pwr;    // current / power    (onboard I2C)
extern SDCard    sd;     // microSD card       (SD SPI)

extern CSVWriter logFile;   // the CSV log on the SD card

/** Drives every sensor-bus chip select high, so no chip answers traffic meant for another. */
void deselectAllSpiDevices();
