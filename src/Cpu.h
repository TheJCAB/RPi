#pragma once

#include "Uart.h"

#include <stdint.h>
#include <stddef.h>

namespace Cpu
{

bool IsRpi4();

[[noreturn]] inline void Halt()
{
    // Halt the CPU by entering an infinite loop
    while (true)
    {
        asm volatile("wfe"); // Wait for event (low power state)
    }
}

[[noreturn]] inline void Panic(char const* message)
{
    // Print a panic message and halt the CPU
    Uart::Raw::Puts("Panic! ");
    Uart::Raw::Puts(message);
    Uart::Raw::Puts("\n");
    Halt();
}

}
// namespace Cpu
