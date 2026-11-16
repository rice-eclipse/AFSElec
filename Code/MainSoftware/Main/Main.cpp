/**
 * Main.cpp - AFS flight computer firmware (FreeRTOS on both RP2350 cores).
 *
 * At power-up the boot task:
 *   1. runs every bench test (including the simulated-flight EKF test),
 *   2. sets every sensor to its fastest rate and opens a new CSV log on the SD card,
 *   3. runs the sensor task (core 0) and logger task (core 1) for RECORD_DURATION_MS,
 *   4. prints the measured sensor and logging rates,
 *      (the green LED turns on if every enabled test passed and every sensor started),
 *   5. sweeps servos PWM1-PWM8 +-SERVO_SWEEP_DEG around their centers for SERVO_SWEEP_MS,
 *   6. runs the serial servo tuning console until "Done." is sent,
 *   7. then navigates forever: the NavEKF runs on core 0 at full sensor speed (keep the board
 *      still for NAV_ALIGN_MS first), and this task logs its state to flights/nav_NNN.csv.
 *
 * Files:
 *   config.h            every tunable setting (tests, rates, durations, board geometry, task layout)
 *   src/devices.h/.cpp  the sensor and SD card objects
 *   src/shared.h/.cpp   data passed between tasks (Sample, NavRow, RunStats, queues, events)
 *   src/boot.h/.cpp     startup tests, sensor setup, log file creation
 *   src/servo.h/.cpp    servo PWM outputs and the startup sweep task
 *   src/servo_tune.h/.cpp  serial console to set each servo's pulse width by hand
 *   src/serial_in.h/.cpp   serial (UART0) input, read a line at a time
 *   src/leds.h/.cpp     indicator LEDs on GPIO 0-3
 *   src/flight_state.h/.cpp  flight state machine (LAUNCHPAD ... LANDED)
 *   src/roll_test.h/.cpp     in-flight roll test (fin tab doublets during coast)
 *   src/loop.h/.cpp     speed test (sensor/logger tasks, report) and navigation (nav task, logger)
 *   sim/                replays a flight's nav CSV on the PC with different settings
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "rtos.h"

#include "config.h"
#include "devices.h"
#include "shared.h"
#include "boot.h"
#include "loop.h"
#include "servo.h"
#include "servo_tune.h"
#include "leds.h"
#include "EKF_test.h"

static void bootTask(void*)
{
    ledsInit();
    sleep_ms(2000);   // give the serial terminal time to connect

    // 1. Self-test everything
    deselectAllSpiDevices();
    bool bootOk = runStartupTests();
    if constexpr (RUN_EKF_TEST) bootOk &= EKF_test(VERBOSE_EKF_TEST);

    // 2. Set every sensor to its fastest rate (needed by navigation too)
    g_status = startSensorsMaxRate();
    const SensorStatus& st = g_status;
    bootOk &= st.imu && st.acc && st.mag && st.baro && st.temp && st.pwr;

    // Green LED = every enabled test passed and every sensor started
    ledSet(LED_GREEN, bootOk);
    printf("Boot checks: %s\n", bootOk ? "all passed (green LED on)" : "FAILED (green LED off)");

    if constexpr (RUN_SPEED_TEST) {
        // 3. Speed test: sensors on core 0, SD logging on core 1, until both are done
        g_stats.fileOk = openLogFile(g_logName, sizeof g_logName);
        if (g_stats.fileOk) printf("Starting write: %s\n", g_logName);
        rtos::spawn(loggerTask, "logger",  LOGGER_STACK_WORDS, LOGGER_PRIORITY, nullptr, LOGGER_CORE);
        rtos::spawn(sensorTask, "sensors", SENSOR_STACK_WORDS, SENSOR_PRIORITY, nullptr, SENSOR_CORE);
        g_events.wait(EVT_SENSOR_DONE | EVT_LOGGER_DONE, /*all*/ true);

        // 4. Report
        printRunReport();
    }

    // 5. Servo sweep: wait for it to finish before navigating
    if constexpr (RUN_SERVO_SWEEP) {
        rtos::spawn(servoTask, "servos", SERVO_STACK_WORDS, SERVO_PRIORITY, nullptr, SERVO_CORE);
        g_events.wait(EVT_SERVO_DONE);
    }

    // 6. Servo tuning over the serial port: waits until "Done." is sent
    if constexpr (RUN_SERVO_TUNE) {
        rtos::spawn(servoTuneTask, "servo_tune", SERVO_TUNE_STACK_WORDS, SERVO_PRIORITY, nullptr, SERVO_CORE);
        g_events.wait(EVT_SERVO_TUNE_DONE);
    }

    // 7. Navigate forever: EKF on core 0, this task becomes its logger
    startNavigation();
    navLoggerLoop();   // never returns
}

int main()
{
    stdio_init_all();

    if (!g_samples.valid()) panic("not enough FreeRTOS heap for the sample queue");

    rtos::spawn(bootTask, "boot", BOOT_STACK_WORDS, BOOT_PRIORITY, nullptr, BOOT_CORE);
    rtos::start();   // never returns
}
