# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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
cmake_minimum_required(VERSION 3.13)

# 1. Tell CMake where to find custom board headers
set(PICO_BOARD_PATH ${CMAKE_CURRENT_LIST_DIR}/include/boards)

# 2. Set the board name to match your header filename (without the .h)
set(PICO_BOARD rp2350b_16mb CACHE STRING "Board type")
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards CACHE STRING "PICO board header directories" FORCE)

# Pull in Raspberry Pi Pico SDK (must be done after setting PICO_BOARD_PATH)
include(pico_sdk_import.cmake)

project(my_rp2350_project C CXX ASM)


set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(test test.cpp )

pico_set_program_name(test "test")
pico_set_program_version(test "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(test 1)
pico_enable_stdio_usb(test 0)

# Add the standard library to the build
target_link_libraries(test
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(test PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(test)

