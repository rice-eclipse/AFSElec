cmake_minimum_required(VERSION 3.13)

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

# Change your executable name to something creative!
set(NAME Blink) # <-- Name your project/executable here!
set(PICO_BOARD pico2_w)
set(PICO_PLATFORM rp2350-arm-s)

#include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

include(pico_sdk_import.cmake)

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 26)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialize the SDK
pico_sdk_init()

# Point this at your FreeRTOS-Kernel folder (forward slashes, even on Windows)
SET(FREERTOS_KERNEL_PATH "C:/Users/angus/FreeRTOS-Kernel-Pi" CACHE STRING "FreeRTOS kernel")
SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel" CACHE STRING "Local Config")
include("${FREERTOS_CONFIG_FILE_DIRECTORY}/freertos_config.cmake")
include(FreeRTOS_Kernel_import.cmake)

add_subdirectory(src)

# Disabled: Unix-only deploy script from the original setup
# install(CODE "execute_process(COMMAND $ENV{HOME}/bin/picoDeploy.sh ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.elf)")

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.uf2
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)