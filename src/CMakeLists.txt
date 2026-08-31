cmake_minimum_required(VERSION 3.24)

add_subdirectory(BootLib)
add_subdirectory(HelloBoot)
add_subdirectory(Playground)
add_subdirectory(KernelLoader)

add_library(CppCrt STATIC
    # Compilery stuff, to make C++ work.
    ClangBuiltins.cpp
    CppException.cpp
    CppTypeInfo.cpp
    Crt.cpp
    emb-stdio.cpp       emb-stdio.h
    Heap.cpp            Heap.h
)
target_link_libraries(CppCrt PUBLIC KernelRuntime)

set(KernelSources
    Boot.S

    # Figuring out the machine we're running on.
    DeviceTree.cpp

    # Kernel OS.
    Cpu.cpp             Cpu.h
    Syscall.cpp
    Exception.cpp       Exception.h
    Mmio.cpp            Mmio.h
    Mmu.cpp             Mmu.h
    Scheduler.cpp       Scheduler.h
    Debugger.cpp        Debugger.h
    Timer.cpp           Timer.h
    Interrupts.cpp      Interrupts.h
    Processor.h
    Task.h

    # Generic devices.
    Uart.cpp            Uart.h

    # Generic USB devices.
    Usb.h
    UsbDriver.cpp       UsbDriver.h
    HidUsbDevices.cpp   HidUsbDevices.h
    Keyboard.cpp        Keyboard.h
    Gamepad.cpp         Gamepad.h

    # Raspberry Pi devices.
    Gpio.cpp            Gpio.h
    SdCard.cpp          SdCard.h
    Mailbox.cpp         Mailbox.h
    Framebuffer.cpp     Framebuffer.h

    # PCIe bus.
    PCIe.cpp            PCIe.h

    # Raspberry Pi 3 (and 4, connected to the power socket) USB controller.
    DesignWareUsb.cpp   DesignWareUsb.h
    DesignWareUsbHost.cpp
    DesignWareUsbChannel.cpp DesignWareUsbChannel.h
    DesignWareUsbDriver.cpp

    # Generic XHCI driver.
    UsbXhci.cpp


    # The program we're running (Snake game).
    Main.cpp
    TimerExample.cpp    TimerExample.h
    Run.cpp             Run.h
)

add_kernel_image(kernel8 RPI ${KernelSources})

target_link_libraries(kernel8 PRIVATE BootLib CppCrt)


# QEMU kernel. Start with:
#   qemu-system-aarch64.exe -machine virt,virtualization=on -cpu cortex-a72 -m 2G -kernel build\clang\bin\kernelvirt.elf -d guest_errors
# add "-S -s" for debugging.
add_kernel_image(kernelvirt VIRT ${KernelSources})

target_link_libraries(kernelvirt PRIVATE BootLib CppCrt)


#set_target_properties(kernel8 PROPERTIES
#    SUFFIX ".elf"
#)
#
#target_compile_options(kernel8 PRIVATE
#    -ffreestanding
#    -Wno-builtin-macro-redefined
#    -O3
#    -frtti
#    #-fPIC
#    -mcmodel=tiny
#    #-flto
#    #-H
#)
#target_Link_options(kernel8 PRIVATE
#    -Wl,--gc-sections
#    -ffunction-sections
#    -fdata-sections
#    -fpie
#    #-flto
#)
#
## Specify the linker script
#set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/kernel.ld")
#set_target_properties(kernel8 PROPERTIES LINK_DEPENDS "${LINKER_SCRIPT}")
#target_link_options(kernel8 PRIVATE "-T${LINKER_SCRIPT}")
#
##add_custom_command(
##    TARGET kernel8
##    PRE_BUILD
##    COMMAND clang -IC:/repos/RPi/libcxxinc/. -E -x c++ - -v < nul
##    COMMAND cmd /c set
##)
#
## Convert the elf file to a binary image and generate assembly listing
#add_custom_command(
#    TARGET kernel8
#    POST_BUILD
#    COMMAND ${CMAKE_OBJCOPY} --set-start=0x80000 -O binary $<TARGET_FILE:kernel8> $<TARGET_FILE_DIR:kernel8>/kernel8.img
#    COMMAND ${CMAKE_OBJDUMP} -l -S -D $<TARGET_FILE:kernel8> > $<TARGET_FILE_DIR:kernel8>/kernel8.asm
#    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_SOURCE_DIR}/boot" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
#    COMMENT "Creating kernel8.img"
#)
