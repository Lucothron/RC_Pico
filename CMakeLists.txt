# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.0/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# project(Transmisor C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_compile_options(-Wall -Wno-unused-function)

# Add executable. Default name is the project name, version 0.1
# add_executable(Transmisor Transmisor.c sdk_inlines.c)

# pico_set_program_name(Transmisor "Transmisor")
# pico_set_program_version(Transmisor "0.1")

# pico_enable_stdio_uart(Transmisor 0)
# pico_enable_stdio_usb(Transmisor 1)

# Add the standard library to the build
# target_link_libraries(Transmisor pico_stdlib hardware_gpio)
# target_link_libraries(Transmisor pico_stdlib hardware_spi)
# target_link_libraries(Transmisor pico_stdlib hardware_adc)
add_subdirectory(inc)
add_subdirectory(prx)
add_subdirectory(ptx_1)
target_link_libraries(ptx_1 pico_stdlib NRF24)

# pico_add_extra_outputs(Transmisor)


