#ifndef _BOARDS_MY_CUSTOM_BOARD_H
#define _BOARDS_MY_CUSTOM_BOARD_H

// For use with RP2350 boards with 16MB flash
#define PICO_BOARD "my_custom_board"

// --- BOARD SPECIFIC PINS ---
// Adjust these based on your RP2350B schematic and GPIO layout
#define PICO_DEFAULT_LED_PIN 25

// --- FLASH CONFIGURATION ---
// Set flash size to 16MB (16 * 1024 * 1024 bytes)
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)

// Optional: Define QSPI settings if using non-default pins/speeds
// #define PICO_FLASH_SPI_CLKDIV 2

#endif
