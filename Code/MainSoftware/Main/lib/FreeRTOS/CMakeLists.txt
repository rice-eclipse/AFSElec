# FreeRTOS kernel (SMP, RP2350 ARM port) + rtos.h C++ wrappers.
# Link with: target_link_libraries(<target> FreeRTOS)
#
# The kernel lives in FreeRTOS-Kernel/ (vendored, see VENDORED.txt) and its configuration in
# config/FreeRTOSConfig.h. The port's own CMake defines the FreeRTOS-Kernel* targets.

set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel CACHE PATH "Path to the FreeRTOS kernel" FORCE)
set(FREERTOS_CONFIG_FILE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/config)

if (PICO_PLATFORM STREQUAL "rp2350-arm-s")
    add_subdirectory(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ FreeRTOS-Kernel)
else()
    message(FATAL_ERROR "lib/FreeRTOS only vendors the RP2350 ARM port (PICO_PLATFORM is '${PICO_PLATFORM}')")
endif()

add_library(FreeRTOS INTERFACE)
target_sources(FreeRTOS INTERFACE ${CMAKE_CURRENT_LIST_DIR}/rtos.cpp)
target_include_directories(FreeRTOS INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(FreeRTOS INTERFACE
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4   # first-fit allocator with free-block merging
        pico_stdlib
        )

# Optional bench test (call FreeRTOS_test() from inside a task). Link with: target_link_libraries(<target> FreeRTOS_test)
add_library(FreeRTOS_test INTERFACE)
target_sources(FreeRTOS_test INTERFACE ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS_test.cpp)
target_link_libraries(FreeRTOS_test INTERFACE FreeRTOS)
