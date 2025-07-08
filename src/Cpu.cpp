#include "Cpu.h"

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

}
// namespace Cpu
