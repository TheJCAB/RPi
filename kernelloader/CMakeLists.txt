cmake_minimum_required(VERSION 3.24)

add_executable(kernelloader
    #Min2.S
    #Min.S
    kernelloader.S
    #kernelloadernew.S
)

set_target_properties(kernelloader PROPERTIES
    SUFFIX ".elf"
)

target_compile_options(kernelloader PRIVATE
    -O3
    -fPIC
)

target_link_options(kernelloader PRIVATE
    -fpie
)

# Specify the linker script
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/kernelloader.ld")
set_target_properties(kernelloader PROPERTIES LINK_DEPENDS "${LINKER_SCRIPT}")
target_link_options(kernelloader PRIVATE "-T${LINKER_SCRIPT}")

# Convert the elf file to a binary image and generate assembly listing
add_custom_command(
    TARGET kernelloader
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} --set-start=0x80000 -O binary $<TARGET_FILE:kernelloader> $<TARGET_FILE_DIR:kernelloader>/kernelloader.img
    COMMAND ${CMAKE_OBJDUMP} -l -S -D $<TARGET_FILE:kernelloader> > $<TARGET_FILE_DIR:kernelloader>/kernelloader.asm
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_SOURCE_DIR}/boot" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    COMMENT "Creating kernelloader.img"
)
