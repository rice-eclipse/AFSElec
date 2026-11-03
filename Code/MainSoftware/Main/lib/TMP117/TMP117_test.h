/**
 * TMP117_test.h - Bench test for the TMP117 driver.
 *
 * verbose = false: prints only FAIL lines, diagnostics and the one-line result.
 * verbose = true:  also prints every PASS line and the sample readings.
 *
 * Prints results over stdio and returns true if all checks pass. Does not write EEPROM.
 * The test soft-resets the device at the end to restore power-on defaults.
 *
 *   TMP117 temp(i2c0, SDA, SCL);
 *   TMP117_test(temp);
 */
#pragma once

#include "TMP117.h"

bool TMP117_test(TMP117& tmp, bool verbose = false, uint32_t samples = 5);
