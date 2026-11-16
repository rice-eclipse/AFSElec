/**
 * boot.h - Everything that happens once at power-up, before the recording tasks start.
 */
#pragma once

#include <stddef.h>
#include "shared.h"

/** Runs every enabled bench test (config.h) and prints the summary table. Returns true if none failed. */
bool runStartupTests();

/** begin()s every sensor and sets it to its fastest output rate. Returns which ones started. */
SensorStatus startSensorsMaxRate();

/**
 * Creates the next unused log file (flights/flight_NNN.csv) and writes the CSV header.
 * Puts the file name in `name`. Returns false (and prints why) if the SD card can't be used.
 */
bool openLogFile(char* name, size_t nameLen);

/**
 * Creates the next unused file `<prefix>NNN.csv` and opens it in logFile (no header written).
 * Puts the file name in `name`. Returns false (and prints why) if the SD card can't be used.
 */
bool createLogFile(const char* prefix, char* name, size_t nameLen);
