/**
 * serial_in.h - Reading text from the serial port (UART0 RX, GPIO 13).
 *
 * Reads straight from the UART hardware FIFO (not through stdio's getchar), which proved reliable.
 * Only one task should read at a time.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/** Next received byte, or -1 if none. Bytes with framing, break or parity errors are dropped. */
int serialReadByte();

/** Drops everything waiting in the receive FIFO. */
void serialFlushInput();

/**
 * Non-blocking line reader: call poll() as often as convenient. A line ends at CR or LF, or after
 * idleMs with no new characters (terminals set to send no line ending). Characters are echoed if
 * `echo` is set.
 */
class SerialLineReader {
public:
    explicit SerialLineReader(uint32_t idleMs, bool echo) : idleMs_(idleMs), echo_(echo) {}

    /** Returns true and puts a complete, trimmed, non-empty line in `out` when one is ready. */
    bool poll(char* out, size_t outLen);

private:
    uint32_t idleMs_;
    bool echo_;
    char buf_[32];
    size_t n_ = 0;
    uint32_t lastCharMs_ = 0;
};
