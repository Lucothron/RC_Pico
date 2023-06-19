# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.0/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)


# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_compile_options(-Wall -Wno-unused-function)

add_subdirectory(inc)
add_subdirectory(prx)
add_subdirectory(ptx_1)
target_link_libraries(ptx_1 pico_stdlib NRF24)


