/**
 * ICM45686_test.h - Bench test for the ICM45686 driver.
 *
 * verbose = false: prints only FAIL lines, diagnostics and the one-line result.
 * verbose = true:  also prints every PASS line and the sample readings.
 *
 * Run with the board lying still. Prints results over stdio and returns true if all checks pass.
 * The test reconfigures the device, and calls begin() again at the end to restore defaults.
 *
 *   ICM45686 imu(spi0, IMU_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, IMU_INT1);
 *   ICM45686_test(imu);
 */
#pragma once

#include "ICM45686.h"

bool ICM45686_test(ICM45686& imu, bool verbose = false, uint32_t samples = 10);
