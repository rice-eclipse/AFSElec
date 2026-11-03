#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

// GPIO 0 - 3
constexpr uint GPIO_00 = 0; // Pin 77
constexpr uint GPIO_01 = 1; // Pin 78
constexpr uint GPIO_02 = 2; // Pin 79
constexpr uint GPIO_03 = 3; // Pin 80

// GPIO 4 - 5 (Unconnected / NC)
// GPIO 4 -> Pin 1 (NC)
// GPIO 5 -> Pin 2 (NC)

// PWM Signals
constexpr uint PWM2 = 6; // Pin 3
constexpr uint PWM3 = 7; // Pin 4
constexpr uint PWM1 = 8; // Pin 6
constexpr uint PWM4 = 9; // Pin 7
constexpr uint PWM8 = 44; // Pin 55
constexpr uint PWM7 = 45; // Pin 56
constexpr uint PWM5 = 46; // Pin 57
constexpr uint PWM6 = 47; // Pin 58

// I2C 2 Interface
constexpr uint SDA2 = 10; // Pin 8
constexpr uint SCL2 = 11; // Pin 9

// UART0 Interface
constexpr uint UART0_TX = 12; // Pin 11
constexpr uint UART0_RX = 13; // Pin 12

// Sensors & Onboard Bus
constexpr uint IMU_INT1 = 15;     // Pin 14
constexpr uint SPI_MISO_SEN = 16; // Pin 16
constexpr uint IMU_CS = 17;       // Pin 17
constexpr uint SPI_SCK_SEN = 18;  // Pin 18
constexpr uint SPI_MOSI_SEN = 19; // Pin 19
constexpr uint BAR_CS = 20;       // Pin 20
constexpr uint MAG_CS = 21;       // Pin 21
constexpr uint ACC_CS = 22;       // Pin 22
constexpr uint MAG_INT = 23;      // Pin 23
constexpr uint SDA = 24;          // Pin 25
constexpr uint SCL = 25;          // Pin 26 (Visual label on schematic shows 25/26)
constexpr uint ACC_INT = 26;      // Pin 27
constexpr uint XTRA_SPI_CS = 27;  // Pin 28

// SD Card Interface
constexpr uint SPI_MISO_SD = 28;    // Pin 36
constexpr uint SD_CHIP_SELECT = 29; // Pin 37
constexpr uint SPI_SCK_SD = 30;     // Pin 38
constexpr uint SPI_MOSI_SD = 31;    // Pin 39
constexpr uint SD_CHIP_DETECT = 35; // Pin 44




int main()
{
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    while (true)
    {
        uart_puts(uart0, "Hello, world!\r\n");
        sleep_ms(1000);
    }
}
