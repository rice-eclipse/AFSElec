/**
 * FreeRTOS_test.h - Bench test for the FreeRTOS port, configuration and rtos.h wrappers.
 *
 * MUST be called from inside a running FreeRTOS task (after rtos::start()). It creates and deletes
 * its own helper tasks and takes about 1 s.
 *
 * Checks: both cores schedule tasks, delay / Periodic timing, Queue between cores, Mutex under
 * contention from both cores, Semaphore timeout, task notifications, EventGroup, software Timer,
 * Pico SDK sleep_ms() yielding to other tasks (SDK interop), and no heap leaked by the test.
 *
 * verbose = false: prints only FAIL lines, diagnostics and the one-line result.
 * verbose = true:  also prints every PASS line, the measured timings and the task list.
 */
#pragma once

#include "rtos.h"

bool FreeRTOS_test(bool verbose = false);
