cmake_minimum_required(VERSION 3.24)

# The BootLib library includes a number of low-level drivers and utilities for the Raspberry Pi platform.
# It is designed to be usable in a minimal bootloader, with minimal dependencies.
add_library(BootLib STATIC
    Cpu.cpp             inc/BootLib/Cpu.h
    DeviceTree.cpp      inc/BootLib/DeviceTree.h
    Gpio.cpp            inc/BootLib/Gpio.h
    Uart.cpp            inc/BootLib/Uart.h
    Mmio.cpp            inc/BootLib/Mmio.h

    inc/BootLib/RegisterProxy.h
)

target_include_directories(BootLib PUBLIC inc)

target_link_libraries(BootLib PUBLIC KernelRuntime)
