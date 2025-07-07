#pragma once

#include "Uart.h"

#include <stdint.h>
#include <stddef.h>

namespace Cpu
{

bool IsRpi4();

extern uint64_t const PerformanceFrequency;

uint64_t GetPerformanceCounter();

inline uint64_t GetPerformanceTicksForUs(uint64_t us)
{
    return (us * PerformanceFrequency / 1'000'000u);
}

void DelayInMicroseconds(uint64_t us);

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
