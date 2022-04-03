cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)

project(kitronik C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()
pico_sdk_init()

add_compile_options(-Wall
    )
add_executable(kitronik
    kitronik.c
    main.c
    )
pico_generate_pio_header (kitronik ${CMAKE_CURRENT_LIST_DIR}/kitronik.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources (kitronik PRIVATE kitronik.c main.c)
target_link_libraries(kitronik pico_stdlib hardware_pwm hardware_pio)
pico_enable_stdio_usb(kitronik 1)
pico_enable_stdio_uart(kitronik 0)
pico_add_extra_outputs(kitronik)

