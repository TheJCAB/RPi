#pragma once

#include <BootLib/Cpu.h>

#include <chrono>
#include <optional>

using namespace std::literals::chrono_literals;


namespace Cpu
{

//using namespace BootLib::Cpu;
namespace SysRegData = BootLib::Cpu::SysRegData;

using BootLib::Cpu::CurrentEL;
using BootLib::Cpu::sctlr_el1;
using BootLib::Cpu::daif;
using BootLib::Cpu::cpacr_el1;
using BootLib::Cpu::cptr_el2;
using BootLib::Cpu::mpidr_el1;
using BootLib::Cpu::spsr_el1;
using BootLib::Cpu::esr_el1;

// Virtual timer.
using BootLib::Cpu::cntv_tval_el0;
using BootLib::Cpu::cntv_ctl_el0;

using BootLib::Cpu::daifclr;

using BootLib::Cpu::Halt;
using BootLib::Cpu::InnerDataSynchronizationBarrier;
using BootLib::Cpu::InstructionSynchronizationBarrier;
using BootLib::Cpu::Yield;

using BootLib::Cpu::PerformanceTime;
using BootLib::Cpu::PerformanceTimeDiff;
using BootLib::Cpu::GetPerformanceCounter;
using BootLib::Cpu::GetFarFuturePerformanceTime;

// Convert microseconds to performance counter ticks
inline Cpu::PerformanceTimeDiff ToTicks(std::chrono::microseconds us) { return BootLib::Cpu::GetPerformanceTicksForUs(us.count()); }


extern uint64_t const PerformanceFrequency;

inline void Delay(std::chrono::microseconds duration) { BootLib::Cpu::DelayInMicroseconds(duration.count()); }

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

template < typename F >
inline auto WaitUntilWithTimeout(std::chrono::microseconds timeout, F&& function) -> decltype(function())
{
    auto targetTick = GetPerformanceCounter() + ToTicks(timeout);

    for (;;)
    {
        auto result = function();
        if (result) return std::move(result);
        if (targetTick <= GetPerformanceCounter()) return {};
        Yield();
    }
}

}
// namespace Cpu
