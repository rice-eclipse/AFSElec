/**
 * servo_tune.cpp - Interactive servo tuning over the serial port (see servo_tune.h).
 */
#include "servo_tune.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "rtos.h"

#include "config.h"
#include "serial_in.h"
#include "servo.h"
#include "shared.h"

/** Blocks (letting other tasks run) until a non-empty line arrives (echoed as typed). */
static void readLine(char* buf, size_t len)
{
    static SerialLineReader reader(SERVO_TUNE_IDLE_MS, /*echo*/ true);
    while (!reader.poll(buf, len)) rtos::delay(5);
}

/** Case-insensitive string compare. */
static bool equalsIgnoreCase(const char* a, const char* b)
{
    for (; *a && *b; a++, b++)
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
    return *a == *b;
}

/** Parses a whole line as a non-negative integer. */
static bool parseUInt(const char* s, long& out)
{
    if (!*s) return false;
    char* end;
    out = strtol(s, &end, 10);
    return *end == '\0' && out >= 0;
}

void servoTuneTask(void*)
{
    servoInit();   // all servos at their centers
    uint32_t pulse[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) pulse[i] = SERVO_CENTER_US[i];

    printf("\n========== Servo tuning ==========\n"
           "Type a servo number (1-%d), then pulse widths in us (each servo: center +-%.0f deg).\n"
           "\"change\" picks another servo, \"Done.\" starts navigation.\n",
           SERVO_COUNT, (double)SERVO_LIMIT_DEG);
    serialFlushInput();   // drop anything received before the console started

    char line[32];
    int selected = -1;   // -1 = choosing a servo
    while (true) {
        if (selected < 0) printf("Select servo (1-%d): ", SERVO_COUNT);
        else printf("Servo %d [%lu us] pulse (%lu-%lu), \"change\" or \"Done.\": ", selected + 1,
                    (unsigned long)pulse[selected], (unsigned long)servoMinUs(selected),
                    (unsigned long)servoMaxUs(selected));

        readLine(line, sizeof line);
        if (!line[0]) continue;   // only spaces

        if (equalsIgnoreCase(line, "done.") || equalsIgnoreCase(line, "done")) break;
        if (equalsIgnoreCase(line, "change")) { selected = -1; continue; }

        long v;
        if (!parseUInt(line, v)) {
            printf("  ? expected a number, \"change\" or \"Done.\"\n");
            continue;
        }
        if (selected < 0) {
            if (v < 1 || v > SERVO_COUNT) { printf("  ! servo must be 1-%d\n", SERVO_COUNT); continue; }
            selected = (int)v - 1;
        } else {
            if (v < (long)servoMinUs(selected) || v > (long)servoMaxUs(selected)) {
                printf("  ! out of bounds: %ld us (servo %d allows %lu-%lu), servo not moved\n", v, selected + 1,
                       (unsigned long)servoMinUs(selected), (unsigned long)servoMaxUs(selected));
                continue;
            }
            pulse[selected] = (uint32_t)v;
            servoWriteUs(selected, pulse[selected]);
        }
    }

    printf("Servo tuning done. Last pulse widths (for SERVO_CENTER_US in config.h):\n    {");
    for (int i = 0; i < SERVO_COUNT; i++) printf("%lu%s", (unsigned long)pulse[i], i + 1 < SERVO_COUNT ? ", " : "}\n");
    printf("==================================\n");
    g_events.set(EVT_SERVO_TUNE_DONE);
    vTaskDelete(nullptr);
}
