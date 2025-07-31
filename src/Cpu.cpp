#include "Cpu.h"

#include "Uart.h"
#include "emb-stdio.h"

#include <stdarg.h>

namespace Cpu
{

uint64_t const PerformanceFrequency = GetPerformanceFrequency();

[[noreturn]] void Panic(char const* message)
{
    // Print a panic message and halt the CPU
    Uart::Raw::Puts("Panic! ");
    Uart::Raw::Puts(message);
    Uart::Raw::Puts("\n");
    Halt();
}

[[noreturn]] void Panic(char const* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    // Print a panic message and halt the CPU
    Uart::Raw::Puts("Panic! ");
    Uart::Raw::Puts(buf);
    Uart::Raw::Puts("\n");
    Halt();
}

}
// namespace Cpu
