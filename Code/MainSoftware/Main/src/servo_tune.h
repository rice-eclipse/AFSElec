/**
 * servo_tune.h - Interactive servo tuning over the serial port (UART0), run before navigation.
 *
 *   servoTuneTask reads lines from the serial port:
 *     1. type a servo number (1-SERVO_COUNT) to select it
 *     2. type a pulse width in microseconds (within that servo's center +- SERVO_LIMIT_DEG) to move it there;
 *        repeat as often as needed
 *     "change" goes back to step 1 to pick another servo
 *     "Done." (or "done") prints every servo's last pulse width (ready to paste into
 *     SERVO_CENTER_US), sets EVT_SERVO_TUNE_DONE in g_events and deletes the task
 *   Out-of-range values are rejected (the servo isn't moved). Servos keep their last position.
 */
#pragma once

void servoTuneTask(void* unused);
