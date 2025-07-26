#pragma once

#include <BootLib/Cpu.h>

namespace Cpu
{

using namespace BootLib::Cpu;

extern uint64_t const PerformanceFrequency;

//[[noreturn]] void Panic(char const* message);
[[noreturn]] void Panic(char const* fmt, ...);

inline bool DisableInterrupts()
{
    uint64_t state;
    asm volatile("mrs %0, daif" : "=r"(state));
    asm volatile("msr daifset, #2" ::: "memory");
    return (state & 0x80) == 0;
}

inline void RestoreInterrupts(bool enabled)
{
    if (enabled)
    {
        // Reenable IRQs
        asm volatile("msr daifclr, #2" ::: "memory");
    }
    else
    {
        // Disable IRQs
        asm volatile("msr daifset, #2" ::: "memory");
    }
}

struct WithInterruptsDisabled
{
    bool const WasEnabled;

    WithInterruptsDisabled() : WasEnabled(DisableInterrupts()) {}
    ~WithInterruptsDisabled() { RestoreInterrupts(WasEnabled); }

    WithInterruptsDisabled(WithInterruptsDisabled const&) = delete;
    WithInterruptsDisabled(WithInterruptsDisabled&&) = delete;
};

}
// namespace Cpu
