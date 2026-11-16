# SD card driver + FatFs filesystem (Pico SDK). Link with: target_link_libraries(<target> SDCard)
add_library(SDCard INTERFACE)
target_sources(SDCard INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/SDCard.cpp
        ${CMAKE_CURRENT_LIST_DIR}/SDFile.cpp
        ${CMAKE_CURRENT_LIST_DIR}/fatfs/ff.c
        ${CMAKE_CURRENT_LIST_DIR}/fatfs/ffunicode.c
        )
target_include_directories(SDCard INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/fatfs
        )
target_link_libraries(SDCard INTERFACE pico_stdlib hardware_spi)

# Optional bench test. Link with: target_link_libraries(<target> SDCard_test)
add_library(SDCard_test INTERFACE)
target_sources(SDCard_test INTERFACE ${CMAKE_CURRENT_LIST_DIR}/SDCard_test.cpp)
target_link_libraries(SDCard_test INTERFACE SDCard)
