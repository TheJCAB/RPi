cmake_minimum_required(VERSION 3.24)

add_subdirectory(BootLib)
add_subdirectory(HelloBoot)
add_subdirectory(KernelLoader)

add_kernel_image(kernel8
    Boot.S
    Main.cpp
    Crt.cpp
    CppException.cpp
    CppTypeInfo.cpp

    DeviceTree.cpp

    Cpu.cpp             Cpu.h
    Gpio.cpp            Gpio.h
    Uart.cpp            Uart.h
    SdCard.cpp          SdCard.h
    Mailbox.cpp         Mailbox.h
    Framebuffer.cpp     Framebuffer.h
    Exception.cpp       Exception.h
    Run.cpp             Run.h
    Mmio.cpp            Mmio.h
    Mmu.cpp             Mmu.h
    Scheduler.cpp       Scheduler.h
    Timer.cpp           Timer.h
    UsbDevices.cpp      UsbDevices.h
    HidUsbDevices.cpp   HidUsbDevices.h
    Keyboard.cpp        Keyboard.h
    Gamepad.cpp         Gamepad.h
    PCIe.cpp            PCIe.h

    emb-stdio.cpp       emb-stdio.h
    DesignWareUsb.cpp   DesignWareUsb.h
    DesignWareUsbHost.cpp
    DesignWareUsbChannel.cpp DesignWareUsbChannel.h
    Interrupts.cpp      Interrupts.h

    Processor.h
    Task.h
)

target_link_libraries(kernel8 PRIVATE BootLib)

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
