#pragma once

#include "Uart.h"

namespace Processor
{

inline void FlushDataCache(void const volatile* buffer, size_t size)
{
    // Flush the data cache for the specified buffer
    asm volatile ("dc civac, %0\n" :: "r"(buffer) : "memory");

    // Advance the pointer by cache line size and flush each line
    constexpr size_t CacheLineSize = 64;
    uintptr_t ptr = reinterpret_cast<uintptr_t>(buffer);
    uintptr_t end = ptr + size + CacheLineSize - 1;
    for (; ptr < end; ptr += CacheLineSize)
    {
        asm volatile ("dc civac, %0\n" :: "r"(ptr) : "memory");
    }

    asm volatile (
        "dsb ish\n"
        "isb\n"
    );
}

inline void InvalidateDataCache(void const volatile* buffer, size_t size)
{
    // Flush the data cache for the specified buffer
    asm volatile ("dc ivac, %0\n" :: "r"(buffer) : "memory");

    // Advance the pointer by cache line size and flush each line
    constexpr size_t CacheLineSize = 64;
    uintptr_t ptr = reinterpret_cast<uintptr_t>(buffer);
    uintptr_t end = ptr + size + CacheLineSize - 1;
    for (; ptr < end; ptr += CacheLineSize)
    {
        asm volatile ("dc ivac, %0\n" :: "r"(ptr) : "memory");
    }

    asm volatile (
        "dsb ish\n"
        "isb\n"
    );
}

[[noreturn]] inline void Halt()
{
    Uart::Puts("The processor has been halted.\n");
    for (;;)
    {
        asm volatile ("wfe"); // Wait for event
    }
}

}
// namespace Processor
