
#include "Scheduler.h"
#include "Exception.h"

#include "Uart.h"

#include <stdint.h>
#include <stddef.h>

extern "C"
Exception::Spark SyscallDispatcher(ThreadContext* threadContext, uint32_t code)
{
    auto threadInfo = &Scheduler::GetCurrentThreadInfo();

    auto const iss = Cpu::esr_el1->ISS;
    Uart::Puts("Syscall ISS ");
    Uart::PutDec(iss);
    Uart::Puts("\n");

    switch (iss)
    {
    case 0:
    {
        Uart::Puts("YieldToThread from ");
        Uart::PutHex(reinterpret_cast<uintptr_t>(&Scheduler::GetCurrentThreadInfo()));
        Uart::Puts(" to ");
        Uart::PutHex(reinterpret_cast<uintptr_t>(threadContext->X[0]));
        Uart::Puts("\n");

        auto newThreadInfo = reinterpret_cast<Scheduler::ThreadInfo*>(threadContext->X[0]);
        return Exception::MakeSpark(std::exchange(newThreadInfo->ContextWhenSuspended, nullptr));
    }

    case 1:
        Uart::Puts("Hello from ISS 1\n");
        break;
    default:
        // Handle default case
        break;
    }

    return {};
}
