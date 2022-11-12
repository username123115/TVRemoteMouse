# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/Users/danieljin/Desktop/Pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(USBPower C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(USBPower)
pico_generate_pio_header(USBPower ${CMAKE_CURRENT_LIST_DIR}/Capture.pio)
target_sources(USBPower PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/USBPower.c
        ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(USBPower PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(USBPower pico_stdlib tinyusb_device tinyusb_board hardware_dma hardware_pio)

pico_set_program_name(USBPower "USBPower")
pico_set_program_version(USBPower "0.1")

pico_enable_stdio_uart(USBPower 1)
pico_enable_stdio_usb(USBPower 0)

# Add the standard library to the build
target_link_libraries(USBPower pico_stdlib)

pico_add_extra_outputs(USBPower)

