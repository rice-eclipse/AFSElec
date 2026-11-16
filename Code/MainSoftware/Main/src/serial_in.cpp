/**
 * serial_in.cpp - Reading text from the serial port (see serial_in.h).
 */
#include "serial_in.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "board_pins.h"

int serialReadByte()
{
    uart_hw_t* hw = uart_get_hw(BOARD_UART);
    while (!(hw->fr & UART_UARTFR_RXFE_BITS)) {
        const uint32_t dr = hw->dr;
        if (dr & (UART_UARTDR_FE_BITS | UART_UARTDR_BE_BITS | UART_UARTDR_PE_BITS)) continue;
        return (int)(dr & 0xFF);
    }
    return -1;
}

void serialFlushInput()
{
    while (serialReadByte() >= 0) {}
}

bool SerialLineReader::poll(char* out, size_t outLen)
{
    const uint32_t nowMs = to_ms_since_boot(get_absolute_time());
    bool complete = false;
    int c;
    while ((c = serialReadByte()) >= 0) {
        lastCharMs_ = nowMs;
        if (c == '\r' || c == '\n') {
            if (n_ > 0) { complete = true; break; }
            continue;   // blank line, or the \n of a \r\n
        }
        if (c == '\b' || c == 0x7F) {
            if (n_ > 0) { n_--; if (echo_) printf("\b \b"); }
            continue;
        }
        if (n_ + 1 < sizeof buf_ && isprint(c)) {
            buf_[n_++] = (char)c;
            if (echo_) putchar(c);
        }
    }
    if (!complete && n_ > 0 && nowMs - lastCharMs_ >= idleMs_) complete = true;   // no line ending sent
    if (!complete) return false;

    if (echo_) printf("\n");
    buf_[n_] = '\0';
    // Trim spaces at both ends
    size_t end = n_;
    while (end > 0 && buf_[end - 1] == ' ') end--;
    size_t start = 0;
    while (start < end && buf_[start] == ' ') start++;
    n_ = 0;
    if (start == end) return false;   // only spaces
    size_t len = end - start;
    if (len + 1 > outLen) len = outLen - 1;
    memcpy(out, buf_ + start, len);
    out[len] = '\0';
    return true;
}
