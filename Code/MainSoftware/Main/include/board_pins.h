/**
 * board_pins.h - GPIO map for the AFS electronics board (Version2, RP2350B QFN-80).
 *
 * "Pin" comments are the RP2350B package pad numbers (U2 in Version2.kicad_pcb), which differ from
 * GPIO numbers wherever IOVDD pads sit in between (e.g. GPIO24 is pad 25).
 */
#pragma once

#include "pico/types.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

// ============================================================================
// General-purpose GPIO
// ============================================================================
constexpr uint GPIO_00 = 0; // Pin 77
constexpr uint GPIO_01 = 1; // Pin 78
constexpr uint GPIO_02 = 2; // Pin 79
constexpr uint GPIO_03 = 3; // Pin 80

// ============================================================================
// PWM outputs
// ============================================================================
constexpr uint PWM2 = 6;  // Pin 3
constexpr uint PWM3 = 7;  // Pin 4
constexpr uint PWM1 = 8;  // Pin 6
constexpr uint PWM4 = 9;  // Pin 7
constexpr uint PWM8 = 44; // Pin 55
constexpr uint PWM7 = 45; // Pin 56
constexpr uint PWM5 = 46; // Pin 57
constexpr uint PWM6 = 47; // Pin 58

// ============================================================================
// External I2C (I2C 2 connector) - GPIO 10/11 = i2c1
// ============================================================================
#define EXT_I2C i2c1
constexpr uint SDA2 = 10; // Pin 8
constexpr uint SCL2 = 11; // Pin 9

// ============================================================================
// UART0 - GPIO 12/13 = uart0
// ============================================================================
#define BOARD_UART uart0
constexpr uint UART0_TX = 12; // Pin 11
constexpr uint UART0_RX = 13; // Pin 12

// ============================================================================
// Onboard sensors
// ============================================================================
// Sensor SPI bus - GPIO 16-19 = spi0 (ICM45686, KX134, MMC5983MA, MS5611, extra CS)
#define SENSOR_SPI spi0
constexpr uint SPI_MISO_SEN = 16; // Pin 16
constexpr uint SPI_SCK_SEN  = 18; // Pin 18
constexpr uint SPI_MOSI_SEN = 19; // Pin 19

constexpr uint IMU_CS      = 17; // Pin 17  ICM45686
constexpr uint BAR_CS      = 20; // Pin 20  MS5611
constexpr uint MAG_CS      = 21; // Pin 21  MMC5983MA
constexpr uint ACC_CS      = 22; // Pin 22  KX134
constexpr uint XTRA_SPI_CS = 27; // Pin 28  spare

constexpr uint IMU_INT1 = 15; // Pin 14  ICM45686 INT1
constexpr uint MAG_INT  = 23; // Pin 23  MMC5983MA INT
constexpr uint ACC_INT  = 26; // Pin 27  KX134 INT1

/** Every chip select on the sensor SPI bus (drive all high before using the bus). */
constexpr uint SENSOR_SPI_CS_PINS[] = {IMU_CS, ACC_CS, MAG_CS, BAR_CS, XTRA_SPI_CS};

// Onboard I2C - GPIO 24/25 = i2c0 (TMP117, INA219)
#define SENSOR_I2C i2c0
constexpr uint SDA = 24; // Pin 25
constexpr uint SCL = 25; // Pin 26 (visual label on schematic shows 25/26)

// ============================================================================
// SD card - GPIO 28-31 = spi1
// ============================================================================
#define SD_SPI spi1
constexpr uint SPI_MISO_SD    = 28; // Pin 36
constexpr uint SD_CHIP_SELECT = 29; // Pin 37
constexpr uint SPI_SCK_SD     = 30; // Pin 38
constexpr uint SPI_MOSI_SD    = 31; // Pin 39
constexpr uint SD_CHIP_DETECT = 35; // Pin 44
