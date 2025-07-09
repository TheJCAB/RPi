cmake_minimum_required(VERSION 3.24)

add_library(BootLib STATIC
    Cpu.cpp             inc/BootLib/Cpu.h
    Gpio.cpp            inc/BootLib/Gpio.h
    Uart.cpp            inc/BootLib/Uart.h
    Mmio.cpp            inc/BootLib/Mmio.h

    inc/BootLib/RegisterProxy.h
)

target_include_directories(BootLib PUBLIC inc)

target_link_libraries(BootLib PUBLIC KernelRuntime)
