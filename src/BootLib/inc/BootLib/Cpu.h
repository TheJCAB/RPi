#pragma once

#include <stdint.h>
#include <stddef.h>

namespace BootLib::Cpu
{

bool IsRpi4();

uint64_t GetPerformanceFrequency();

uint64_t GetPerformanceCounter();

inline uint64_t GetPerformanceTicksForUs(uint64_t us)
{
    return (us * GetPerformanceFrequency() / 1'000'000u);
}

void DelayInMicroseconds(uint64_t us);

inline void Yield() { asm volatile("yield"); }

[[noreturn]] inline void Halt()
{
    // Halt the CPU by entering an infinite loop
    while (true)
    {
        asm volatile("wfe"); // Wait for event (low power state)
    }
}

}
// namespace BootLib::Cpu
