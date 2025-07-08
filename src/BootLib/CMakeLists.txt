cmake_minimum_required(VERSION 3.24)

add_library(BootLib STATIC
    Cpu.cpp             inc/BootLib/Cpu.h
    Gpio.cpp            inc/BootLib/Gpio.h
    Uart.cpp            inc/BootLib/Uart.h
    Mmio.cpp            inc/BootLib/Mmio.h

    inc/BootLib/RegisterProxy.h
)

target_compile_options(BootLib PRIVATE
    -ffreestanding
    -Wno-builtin-macro-redefined
    -O3
    -frtti
    #-fPIC
    -mcmodel=tiny
    #-flto
    #-H
)
target_Link_options(BootLib PRIVATE
    -Wl,--gc-sections
    -ffunction-sections
    -fdata-sections
    -fpie
    #-flto
)

target_include_directories(BootLib PUBLIC inc )

target_link_libraries(BootLib PRIVATE cxx-headers)
