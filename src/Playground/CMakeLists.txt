cmake_minimum_required(VERSION 3.24)

add_kernel_image(Playground
    Boot.S
    Main.cpp
)

target_link_libraries(Playground PRIVATE BootLib)
