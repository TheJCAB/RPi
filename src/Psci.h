#pragma once

#include <stdint.h>

namespace Psci
{

enum class Result : int32_t
{
    Success           =  0,
    NotSupported      = -1,
    InvalidParameters = -2,
    Denied            = -3,
    AlreadyOn         = -4,
    OnPending         = -5,
    InternalFailure   = -6,
    NotPresent        = -7,
    Disabled          = -8,
    InvalidAddress    = -9,
};

__attribute__((naked))
inline Result Call(auto...)
{
    asm volatile (
        //"smc #0\n"
        "hvc #0\n"
        "ret\n"
        : // No output operands
        : // No input operands
    );
}

inline Result StartCore(uint32_t mpidr, void (*entryPoint)(uintptr_t), uintptr_t argument)
{
    return Call(0xC400'0003u, mpidr, entryPoint, argument);
}

[[noreturn]]
inline void SystemOff()
{
    Call(0x8400'0008u);
    __builtin_unreachable();
}

}
// namespace Psci
