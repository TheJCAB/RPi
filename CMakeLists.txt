cmake_minimum_required(VERSION 3.24)

set(CMAKE_SYSTEM_NAME Generic)

project(RPi VERSION 0.1 LANGUAGES CXX C ASM)

set(CMAKE_MSVC_RUNTIME_LIBRARY "" CACHE STRING "MSVC Runtime Library" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_library(KernelRuntime INTERFACE)

target_compile_options(KernelRuntime INTERFACE
    -ffreestanding
    -ffixed-x18
    -Wno-builtin-macro-redefined
    -O3
    -frtti
    #-fPIC
    -mcmodel=tiny
    -mcpu=cortex-a53
    #-fno-builtin-__atomic_compare_exchange
    -mno-outline-atomics
    #-flto
    #-H
)

target_link_libraries(KernelRuntime INTERFACE cxx-headers)

function(add_kernel_image TargetName)
    add_executable(${TargetName}
        ${ARGN}
    )

    set_target_properties(${TargetName} PROPERTIES
        SUFFIX ".elf"
    )

    target_link_options(${TargetName} PRIVATE
        -Wl,--gc-sections
        -ffunction-sections
        -fdata-sections
        -fpie
        #-flto
    )

    target_link_libraries(${TargetName} PRIVATE KernelRuntime)

    # Specify the linker script
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/kernel.ld")
    set_target_properties(${TargetName} PROPERTIES LINK_DEPENDS "${LINKER_SCRIPT}")
    target_link_options(${TargetName} PRIVATE "-T${LINKER_SCRIPT}")

    #add_custom_command(
    #    TARGET HelloBoot
    #    PRE_BUILD
    #    COMMAND clang -IC:/repos/RPi/libcxxinc/. -E -x c++ - -v < nul
    #    COMMAND cmd /c set
    #)

    # Convert the elf file to a binary image and generate assembly listing
    add_custom_command(
        TARGET ${TargetName}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} --set-start=0x80000 -O binary $<TARGET_FILE:${TargetName}> $<TARGET_FILE_DIR:${TargetName}>/${TargetName}.img
        COMMAND ${CMAKE_OBJDUMP} -l -S -D $<TARGET_FILE:${TargetName}> > $<TARGET_FILE_DIR:${TargetName}>/${TargetName}.asm
        COMMENT "Creating ${TargetName}.img"
    )
endfunction()

#        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_SOURCE_DIR}/boot" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"

add_subdirectory(libcxxinc)
add_subdirectory(kernelloader)
add_subdirectory(src)
