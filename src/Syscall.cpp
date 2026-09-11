
#include "Syscall.h"

#include "Scheduler.h"
#include "Exception.h"

#include "Uart.h"

#include <print>
#include <stdint.h>
#include <stddef.h>

namespace Syscall
{

static_assert(Function::SparkReturn == Function{0}, "SparkReturn function must be 0");

extern "C"
__attribute__((naked))
void UserModeSparkEnd()
{
    asm volatile ("svc #0\n"); // Return from the user-mode spark
}

extern "C"
Exception::Spark SyscallDispatcher(ThreadContext* threadContext, uint32_t code)
{
    auto threadInfo = &Scheduler::GetCurrentThreadInfo();

    auto const func = static_cast<Function>(Cpu::esr_el1->ISS);
    std::println("Syscall function {}", static_cast<uint32_t>(func));

    switch (func)
    {
    case Function::YieldToThread:
    {
        std::println("YieldToThread from {:#x} to {:#x}",
            reinterpret_cast<uintptr_t>(&Scheduler::GetCurrentThreadInfo()),
            reinterpret_cast<uintptr_t>(threadContext->X[0]));

        auto newThreadInfo = reinterpret_cast<Scheduler::ThreadInfo*>(threadContext->X[0]);
        return Exception::MakeSpark(std::exchange(newThreadInfo->ContextWhenSuspended, nullptr));
    }

    case Function::HelloFromISS1:
        std::println("Hello from ISS 1");
        break;

    default:
        // Handle default case
        Cpu::Panic("Unknown syscall function: %u\n", static_cast<uint32_t>(func));
        break;
    }

    return {};
}

}
// namespace Syscall
