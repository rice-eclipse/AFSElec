/**
 * INA219_test.h - Bench test for the INA219 driver.
 *
 * verbose = false: prints only FAIL lines, diagnostics and the one-line result.
 * verbose = true:  also prints every PASS line and the sample readings.
 *
 * Prints results over stdio and returns true if all checks pass. Calibrate the driver with the
 * real shunt value (constructor or setCalibration()) first; the test re-runs begin() at the end.
 *
 *   INA219 pwr(i2c0, SDA, SCL, 0x40, 0.010f, 8.0f);
 *   INA219_test(pwr);
 */
#pragma once

#include "INA219.h"

bool INA219_test(INA219& ina, bool verbose = false, uint32_t samples = 5);
