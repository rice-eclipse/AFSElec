# Asynchronous EKF (generic AsyncEKF<N>, AltitudeEKF, NavEKF 3D INS). Plain C++, no Pico SDK dependency.
# Link with: target_link_libraries(<target> EKF)
add_library(EKF INTERFACE)
target_sources(EKF INTERFACE ${CMAKE_CURRENT_LIST_DIR}/AltitudeEKF.cpp ${CMAKE_CURRENT_LIST_DIR}/NavEKF.cpp)
target_include_directories(EKF INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Optional bench test (simulated flight, no hardware needed). Link with: target_link_libraries(<target> EKF_test)
add_library(EKF_test INTERFACE)
target_sources(EKF_test INTERFACE ${CMAKE_CURRENT_LIST_DIR}/EKF_test.cpp)
target_link_libraries(EKF_test INTERFACE EKF)
