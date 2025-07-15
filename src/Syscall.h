#pragma once

#include <stdint.h>
#include <stddef.h>

namespace Scheduler
{
struct ThreadInfo;
}

namespace Syscall
{

template < uint64_t syscall >
__attribute__((naked))
inline void Syscall(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2, uintptr_t arg3)
{
    asm volatile (
        "svc #%0\n"     // Make the syscall
        "ret\n"        // Return from the syscall
        : // No output operands
        : "n"(syscall)
    );
}

inline void YieldToThread(Scheduler::ThreadInfo& thread)
{
    Syscall<0>(reinterpret_cast<uintptr_t>(&thread), 0, 0, 0);
}


}
// namespace Syscall
