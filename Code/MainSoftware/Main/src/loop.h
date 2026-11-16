/**
 * loop.h - The two runs after the startup tests.
 *
 * 1. Sensor speed test (RECORD_DURATION_MS):
 *   sensorTask (core 0) polls every sensor as fast as possible and pushes a snapshot into
 *                       g_samples whenever any sensor has new data. Never blocks: if the queue is
 *                       full the snapshot is dropped (and counted).
 *   loggerTask (core 1) writes snapshots from g_samples to the CSV log as fast as it can, and
 *                       stops once the sensor task is done and the queue is empty.
 *   Both set their bit in g_events when they finish; printRunReport() then prints the rates.
 *
 * 2. Navigation (forever):
 *   startNavigation() opens flights/nav_NNN.csv and starts the nav task on core 0. The nav task
 *   averages the sensors, aligns while the board is still (NAV_ALIGN_MS), then feeds every IMU,
 *   magnetometer and barometer sample to the NavEKF and pushes a NavRow at NAV_LOG_IDLE_HZ on the pad / landed and NAV_LOG_FLIGHT_HZ in flight.
 *   navLoggerLoop() turns the calling task into the logger: it writes the rows and prints a
 *   status line every NAV_STATUS_PERIOD_MS. Neither ever returns.
 */
#pragma once

void sensorTask(void* unused);
void loggerTask(void* unused);

/** Prints the "Ending write" line and the measured rates from g_stats. */
void printRunReport();

/** Opens the navigation log and starts the nav task on core 0. Returns false if there is no log. */
bool startNavigation();

/** Writes navigation rows to the SD card forever (call from the task that should do the logging). */
[[noreturn]] void navLoggerLoop();
