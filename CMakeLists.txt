cmake_minimum_required(VERSION 3.24)

set(CMAKE_SYSTEM_NAME Generic)

project(RPi VERSION 0.1 LANGUAGES CXX C ASM)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_C_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_library(KernelRuntime INTERFACE)

target_compile_options(KernelRuntime INTERFACE
    -ffreestanding
    -ffixed-x18
    -Wno-builtin-macro-redefined
    #-O0
    -O1
    #-O2
    #-O3
    -frtti
    #-fPIC
    -mcmodel=tiny
    -mcpu=cortex-a53
    #-mabi=darwinpcs # explore later
    -mno-outline-atomics
    -mno-unaligned-access # Can fail before MMU is activated.
    #-flto
    #-H

    # Generate debug information
    -g3
    -gdwarf-4
    #-fno-omit-frame-pointer
)

target_link_libraries(KernelRuntime INTERFACE cxx-headers)

function(add_kernel_image TargetName)

    cmake_parse_arguments(
        arg
        "RPI;VIRT"
        ""
        ""
        ${ARGN}
    )

    add_executable(${TargetName}
        ${arg_UNPARSED_ARGUMENTS}
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

        # Generate debug information
        -g3
        -gdwarf-4
    )

    target_link_libraries(${TargetName} PRIVATE KernelRuntime)

    # Specify the linker script
    if(${arg_RPI})
        set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/kernel8.ld")
    elseif(${arg_VIRT})
        set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/kernelvirt.ld")
    else()
        message(FATAL_ERROR "No valid kernel type specified. Use RPI or VIRT.")
    endif()
    set_target_properties(${TargetName} PROPERTIES LINK_DEPENDS "${LINKER_SCRIPT}")
    target_link_options(${TargetName} PRIVATE "-T${LINKER_SCRIPT}" "-Wl,-Map,$<TARGET_FILE_DIR:${TargetName}>/${TargetName}.map")

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
        # --set-start=0x80000 
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TargetName}> $<TARGET_FILE_DIR:${TargetName}>/${TargetName}.img
        COMMAND ${CMAKE_OBJDUMP} -l -S -D $<TARGET_FILE:${TargetName}> > $<TARGET_FILE_DIR:${TargetName}>/${TargetName}.asm
        COMMENT "Creating ${TargetName}.img"
    )
endfunction()

#        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_SOURCE_DIR}/boot" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"

add_subdirectory(libcxxinc)
add_subdirectory(kernelloader)
add_subdirectory(src)
