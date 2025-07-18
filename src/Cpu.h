#pragma once

#include <BootLib/Cpu.h>

namespace Cpu
{

using namespace BootLib::Cpu;

extern uint64_t const PerformanceFrequency;

//[[noreturn]] void Panic(char const* message);
[[noreturn]] void Panic(char const* fmt, ...);

inline uint64_t DisableInterrupts()
{
    uint64_t state;
    asm volatile("mrs %0, daif" : "=r"(state));
    asm volatile("msr daifset, #2" ::: "memory");
    return state;
}

inline void RestoreInterrupts(uint64_t state)
{
    asm volatile("msr daif, %0" :: "r"(state) : "memory");
}

struct WithInterruptsDisabled
{
    uint32_t const State;

    WithInterruptsDisabled() : State(DisableInterrupts()) {}
    ~WithInterruptsDisabled() { RestoreInterrupts(State); }

    WithInterruptsDisabled(WithInterruptsDisabled const&) = delete;
    WithInterruptsDisabled(WithInterruptsDisabled&&) = delete;
};

}
// namespace Cpu
