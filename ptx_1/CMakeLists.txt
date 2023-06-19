include_directories(../inc/)

add_executable(ptx_1 ptx_1.c)

# Pull in FreeRTOS
include("D:/ITESM/Sem 10/Sistemas Embebidos/RaspPico/FreeRTOSv202212.01/FreeRTOSv202212.01/FreeRTOS/Source/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake")

# target_include_directories(tx PUBLIC inc)
# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(ptx_1 pico_stdlib NRF24 SEN0308)
target_link_libraries(ptx_1 pico_stdlib FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)

pico_enable_stdio_usb(ptx_1 1)
pico_enable_stdio_uart(ptx_1 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(ptx_1)