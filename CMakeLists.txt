cmake_minimum_required(VERSION 3.24)

set(CMAKE_SYSTEM_NAME Generic)

project(RPi VERSION 0.1 LANGUAGES CXX ASM)

set(CMAKE_MSVC_RUNTIME_LIBRARY "" CACHE STRING "MSVC Runtime Library" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_subdirectory(kernelloader)
add_subdirectory(src)
