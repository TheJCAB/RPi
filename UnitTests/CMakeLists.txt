cmake_minimum_required(VERSION 3.24)

set(CMAKE_SYSTEM_NAME Generic)

project(RPiUnitTests VERSION 0.1 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".exe")

# Set build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_executable(RPiUnitTests
    RPiUnitTests.cpp
)

target_include_directories(RPiUnitTests
    PRIVATE
        ../src
        ../src/BootLib/inc
)

# Enable exceptions for the target and 16-byte atomics (cmpxchg16b instruction)
target_compile_options(RPiUnitTests PRIVATE /EHsc)
target_compile_definitions(RPiUnitTests PRIVATE _WIN64)
# Enable cx16 target feature for clang to support cmpxchg16b
target_compile_options(RPiUnitTests PRIVATE -mcx16)

# Add debug information for debugging
target_compile_options(RPiUnitTests PRIVATE -g)
target_link_options(RPiUnitTests PRIVATE -g)

enable_testing()
add_test(NAME RPiUnitTests COMMAND RPiUnitTests)
