cmake_minimum_required(VERSION 3.24)

add_kernel_image(KernelLoader RPI
    Boot.S
    Main.cpp
)

target_link_libraries(KernelLoader PRIVATE BootLib)
