#pragma once

#define ThreadContext_X            0
#define ThreadContext_Sp    (31 * 8)
#define ThreadContext_Pc    (32 * 8)
#define ThreadContext_Spsr  (33 * 8)
#define ThreadContext_V     (34 * 8)

#ifdef __cplusplus

#include "Cpu.h"

#include <stdint.h>
#include <stddef.h>

typedef __attribute__((neon_vector_type(16))) unsigned char __n128;

struct ThreadContext
{
    uint64_t X[31];                 // General purpose registers
    uint64_t Sp;                    // Stack pointer
    uint64_t Pc;                    // Program counter
    Cpu::SysRegData::spsr_el1 Spsr; // Saved Program Status Register (for exception handling)
    //uint64_t Esr;                 // Exception Syndrome Register (for exception handling)
    //uint64_t Far;                 // Fault Address Register (for exception handling)
    __n128   V[32];                 // SIMD and floating-point registers (128-bit each)
};

#endif // __cplusplus
