# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# Configure the custom RP2350B board before importing the Pico SDK.
set(PICO_BOARD_PATH ${CMAKE_CURRENT_LIST_DIR}/include/boards)
set(PICO_BOARD rp2350b_16mb CACHE STRING "Board type")
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards CACHE STRING "PICO board header directories" FORCE)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Main C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Main Main.cpp )

pico_set_program_name(Main "Main")
pico_set_program_version(Main "0.1")

# Generate PIO header
pico_generate_pio_header(Main ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Main 1)
pico_enable_stdio_usb(Main 0)

# Add the standard library to the build
target_link_libraries(Main
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/include
)

# Add any user requested libraries
target_link_libraries(Main 
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        )

# Sensor driver libraries (lib/)
add_subdirectory(lib)
target_link_libraries(Main
        ICM45686
        MS5611
        MMC5983MA
        KX134
        TMP117
        INA219
        ICM45686_test
        MS5611_test
        MMC5983MA_test
        KX134_test
        TMP117_test
        INA219_test
        )

pico_add_extra_outputs(Main)
