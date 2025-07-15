
#include "Scheduler.h"

#include <stdint.h>
#include <stddef.h>

extern "C"
Scheduler::ThreadInfo* SyscallDispatcher(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2, uintptr_t arg3)
{
    auto threadInfo = &Scheduler::GetCurrentThreadInfo();

    auto const iss = Cpu::esr_el1->ISS;
    Uart::Puts("Syscall ISS ");
    Uart::PutDec(iss);
    Uart::Puts("\n");

    switch (iss)
    {
    case 0:
        Uart::Puts("YieldToThread\n");
        return reinterpret_cast<Scheduler::ThreadInfo*>(arg0);
    case 1:
        Uart::Puts("Hello from ISS 1\n");
        break;
    default:
        // Handle default case
        break;
    }

    return nullptr;
}
