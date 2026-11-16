/**
 * servo.h - Hobby-servo outputs on PWM1-PWM8.
 *
 *   PWM1 GPIO 8   PWM2 GPIO 6   PWM3 GPIO 7   PWM4 GPIO 9
 *   PWM5 GPIO 46  PWM6 GPIO 47  PWM7 GPIO 45  PWM8 GPIO 44
 *
 * Standard 50 Hz servo PWM. Each servo has its own center pulse width (SERVO_CENTER_US in config.h)
 * and every command is clamped to that center +- SERVO_LIMIT_DEG (servoMinUs() .. servoMaxUs()),
 * never outside SERVO_MIN_US .. SERVO_MAX_US.
 *
 *   servoTask (any core) sweeps every servo +-SERVO_SWEEP_DEG around its center for SERVO_SWEEP_MS,
 *   returns them to center, sets EVT_SERVO_DONE in g_events and deletes itself.
 */
#pragma once

#include <stdint.h>
#include "config.h"   // SERVO_COUNT

/** Sets up the PWM slices for all servos and centers them. Called by servoTask; safe to call again. */
void servoInit();

/** Sets servo `index` (0..SERVO_COUNT-1 = PWM1..PWM8) to a pulse width in microseconds (clamped to its limits). */
void servoWriteUs(int index, uint32_t pulseUs);

/** Sets servo `index` to `deg` degrees from its center (positive = longer pulse). */
void servoWriteDeg(int index, float deg);

/** Last pulse width commanded to servo `index` (after clamping), us; 0 before servoInit(). */
uint32_t servoPulseUs(int index);

/**
 * Stops the PWM pulses to servo `index` (output held low). Most servos then stop driving and draw
 * only their idle current; some digital servos keep holding their last position. Any servoWrite*()
 * call starts the pulses again. servoPulseUs() reads 0 while released.
 */
void servoRelease(int index);
void servoReleaseAll();

/** Drives every servo to its center (SERVO_CENTER_US). */
void servoCenterAll();

/** Sweeps all servos +-SERVO_SWEEP_DEG for SERVO_SWEEP_MS, centers them, sets EVT_SERVO_DONE and exits. */
void servoTask(void* unused);
