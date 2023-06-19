# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.0/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Pull in FreeRTOS
include("C:/Users/JORGE/Desktop/Trabajos Jorge/10mo semestre/Sistemas Embebidos/FreeRTOSv202212.01/FreeRTOSv202212.01/FreeRTOS/Source/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake")

project(source C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(source source.c nrf24l01.c)

target_include_directories(source PRIVATE ${CMAKE_CURRENT_LIST_DIR})

pico_set_program_name(source "source")
pico_set_program_version(source "0.1")

pico_enable_stdio_uart(source 0)
pico_enable_stdio_usb(source 1)

# Add the standard library to the build
target_link_libraries(source pico_stdlib hardware_spi FreeRTOS-Kernel FreeRTOS-Kernel-Heap4 hardware_gpio hardware_adc hardware_i2c hardware_uart hardware_timer)

pico_add_extra_outputs(source)

# Directory for my_library CMakeLists.txt
# add_subdirectory(my_library_dir)