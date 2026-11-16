# Flight replay simulator (host PC build, not part of the firmware).
#
# The firmware's flight_state.cpp and roll_test.cpp are copied into the build folder next to
# sim/config.h, so their #include "config.h" picks up the sim's settings instead of the firmware's.
# CMake re-copies them whenever any of these files changes.
cmake_minimum_required(VERSION 3.20)
project(navsim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FW  ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(GEN ${CMAKE_CURRENT_BINARY_DIR}/fw)

foreach(f flight_state.h flight_state.cpp roll_test.h roll_test.cpp)
    configure_file(${FW}/src/${f} ${GEN}/${f} COPYONLY)
endforeach()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h ${GEN}/config.h COPYONLY)

add_executable(navsim
    sim.cpp
    ${GEN}/flight_state.cpp
    ${GEN}/roll_test.cpp
    ${FW}/lib/EKF/NavEKF.cpp
)
target_include_directories(navsim PRIVATE ${GEN} ${CMAKE_CURRENT_SOURCE_DIR}/stubs ${FW}/lib/EKF)

if(MSVC)
    target_compile_options(navsim PRIVATE /W3 /fp:precise)
    target_compile_definitions(navsim PRIVATE _CRT_SECURE_NO_WARNINGS _USE_MATH_DEFINES)
else()
    target_compile_options(navsim PRIVATE -Wall -O2)
endif()
