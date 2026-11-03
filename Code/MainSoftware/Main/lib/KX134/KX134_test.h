/**
 * KX134_test.h - Bench test for the KX134 driver.
 *
 * verbose = false: prints only FAIL lines, diagnostics and the one-line result.
 * verbose = true:  also prints every PASS line and the sample readings.
 *
 * Run with the board lying still. Prints results over stdio and returns true if all checks pass.
 * The test reconfigures the device, and calls begin() again at the end to restore defaults.
 *
 *   KX134 acc(spi0, ACC_CS, SPI_SCK_SEN, SPI_MOSI_SEN, SPI_MISO_SEN, ACC_INT);
 *   KX134_test(acc);
 */
#pragma once

#include "KX134.h"

bool KX134_test(KX134& acc, bool verbose = false, uint32_t samples = 10);
