#pragma once

#include <stdint.h>
#include <stddef.h>

namespace Scheduler
{
struct ThreadInfo;
}

namespace Syscall
{

enum class Function : uint32_t
{
    SparkReturn     = 0,
    HelloFromISS1   = 1,
    YieldToThread   = 2,
    // Add more syscall functions as needed
};

template < Function func >
__attribute__((naked))
inline void Syscall(auto...)
{
    asm volatile (
        "svc #%0\n"     // Make the syscall
        "ret\n"        // Return from the syscall
        : // No output operands
        : "n"(static_cast<uint32_t>(func))
    );
}

inline void YieldToThread(Scheduler::ThreadInfo& thread)
{
    Syscall<Function::YieldToThread>(reinterpret_cast<uintptr_t>(&thread));
}


}
// namespace Syscall
