cmake_minimum_required(VERSION 3.24)

set(CMAKE_SYSTEM_NAME Generic)

project(RPiUnitTests VERSION 0.1 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(RPiUnitTests
    RPiUnitTests.cpp
)

target_include_directories(RPiUnitTests
    PRIVATE
        ../src
        ../src/BootLib/inc
)

# Enable exceptions for the target
target_compile_options(RPiUnitTests PRIVATE /EHsc)

enable_testing()
add_test(NAME RPiUnitTests COMMAND RPiUnitTests)
