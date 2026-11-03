/**
 * MMC5983MA_test.h - Bench test for the MMC5983MA driver.
 *
 * verbose = false: prints only FAIL lines, diagnostics and the one-line result.
 * verbose = true:  also prints every PASS line and the sample readings.
 *
 * Keep the board away from magnets/motors. Prints results over stdio and returns true if all
 * checks pass. The test reconfigures the device, and calls begin() again at the end to restore defaults.
 *
 *   MMC5983MA mag(spi0, MAG_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, MAG_INT);
 *   MMC5983MA_test(mag);
 */
#pragma once

#include "MMC5983MA.h"

bool MMC5983MA_test(MMC5983MA& mag, bool verbose = false, uint32_t samples = 10);
