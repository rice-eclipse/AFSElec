/**
 * MS5611_test.h - Bench test for the MS5611 driver.
 *
 * verbose = false: prints only FAIL lines, diagnostics and the one-line result.
 * verbose = true:  also prints every PASS line and the sample readings.
 *
 * Prints results over stdio and returns true if all checks pass. Leaves the device at OSR 4096.
 *
 *   MS5611 baro(spi0, BAR_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN);
 *   MS5611_test(baro);
 */
#pragma once

#include "MS5611.h"

bool MS5611_test(MS5611& baro, bool verbose = false, uint32_t samples = 10);
