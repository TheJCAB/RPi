
add_library(fmt STATIC format.cc)

target_compile_definitions(fmt PUBLIC FMT_USE_LOCALE=0)

target_include_directories(fmt PUBLIC include)

target_link_libraries(fmt PUBLIC KernelRuntime cxx-headers)
