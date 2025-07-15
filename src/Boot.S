.section .text

// Each core gets its function. This simplifies things a bit.

.extern Core0
.extern Core1
.extern Core2
.extern Core3

// This is where our code starts running.
// Note that it must be located/loaded at a specific address. 0x8'0000 on the Raspberry Pi 3.
// Upon boot, core zero starts here in EL2.
// The MMU is not configured so all the core sees is physical memory in "device" mode.
// Code runs quite slowly. Carefully, one could say :-).
//
// When the Core0 code spins up the other three cores, they also start here.
// This code just sets the stack pointer and dispatches to one of the Core# functions,
// depending on which core is running at the time.
.global _start
_start:
    // Enable the Floating point and SIMD unit for EL2
    // We're about to dive straight into C++,
    // and the compiler might decide to use all those handy registers right away.
    mrs x4, cptr_el2
    orr x4, x4, #0x300000
    msr cptr_el2, x4
    isb // Make it stick immediately

    // All cores start here, so find out which one is running
    mrs x4, mpidr_el1
    and x4, x4, #3          // x4 now contains the core number.

    // Set up the stack pointer.
    // Stack buffers will use the 512 KB of space before the kernel image.
    // Each core takes 64 KB of stack space. That's enough room to get comfortably started.
    adr x5, _start
    sub x5, x5, x4, LSL #16
    mov sp, x5

    // And dispatch straight into C++.
    adr x5, CoreFunctionTable
    ldr x4, [x5, w4, UXTW #3]
    br x4

    .align 3
CoreFunctionTable:
    .quad Core0
    .quad Core1
    .quad Core2
    .quad Core3

// Goes down from EL2 to EL1 and returns straight to the calling code.
.global el2_to_el1_return
el2_to_el1_return:
    //set up el1 to AARCH64
    mov     x1,#(1 << 31) // AARCH64
    orr     x1,x1,#(1 << 1) // SWIO
    msr     hcr_el2,x1

    // Set up EL1 stack pointer
    mov x0, sp
    msr sp_el1, x0

    // Set up SPSR_EL2 for return to EL1h, interrupts enabled
    mov x0, #(0b0101 << 0)  // M[3:0]=0101 (EL1 with SP_EL1), rest 0
    orr x0, x0, #(0b1111 << 6)  // DAIF all interrupts masked
    msr spsr_el2, x0

    // Set ELR_EL1 to return address
    msr elr_el2, lr

    eret

// Push the entire set of volatile registers onto the stack.
PreserveVolatileRegisters:
    stp     x0 , x1 , [sp, #-16]!
    stp     x2 , x3 , [sp, #-16]!
    stp     x4 , x5 , [sp, #-16]!
    stp     x6 , x7 , [sp, #-16]!
    stp     x8 , x9 , [sp, #-16]!
    stp     x10, x11, [sp, #-16]!
    stp     x12, x13, [sp, #-16]!
    stp     x14, x15, [sp, #-16]!
    stp      q0,  q1, [sp, #-32]!
    stp      q2,  q3, [sp, #-32]!
    stp      q4,  q5, [sp, #-32]!
    stp      q6,  q7, [sp, #-32]!
    stp      q8,  q9, [sp, #-32]!
    stp     q10, q11, [sp, #-32]!
    stp     q12, q13, [sp, #-32]!
    stp     q14, q15, [sp, #-32]!
    stp     q16, q17, [sp, #-32]!
    stp     q18, q19, [sp, #-32]!
    stp     q20, q21, [sp, #-32]!
    stp     q22, q23, [sp, #-32]!
    stp     q24, q25, [sp, #-32]!
    stp     q26, q27, [sp, #-32]!
    stp     q28, q29, [sp, #-32]!
    stp     q30, q31, [sp, #-32]!
    ret

// Pop the entire set of volatile registers onto the stack.
RestoreVolatileRegisters:
    ldp     q30, q31, [sp], #32
    ldp     q28, q29, [sp], #32
    ldp     q26, q27, [sp], #32
    ldp     q24, q25, [sp], #32
    ldp     q22, q23, [sp], #32
    ldp     q20, q21, [sp], #32
    ldp     q18, q19, [sp], #32
    ldp     q16, q17, [sp], #32
    ldp     q14, q15, [sp], #32
    ldp     q12, q13, [sp], #32
    ldp     q10, q11, [sp], #32
    ldp      q8,  q9, [sp], #32
    ldp      q6,  q7, [sp], #32
    ldp      q4,  q5, [sp], #32
    ldp      q2,  q3, [sp], #32
    ldp      q0,  q1, [sp], #32
    // Skip x16-18 and LR, they are handled elsewhere
    ldp     x14, x15, [sp], #16
    ldp     x12, x13, [sp], #16
    ldp     x10, x11, [sp], #16
    ldp     x8 , x9 , [sp], #16
    ldp     x6 , x7 , [sp], #16
    ldp     x4 , x5 , [sp], #16
    ldp     x2 , x3 , [sp], #16
    ldp     x0 , x1 , [sp], #16
    ret

#include "ThreadContext.h"

// Pop the entire set of volatile registers onto the stack.
MoveAllRegistersToThreadContext:
    stp      x0,  x1, [x18, #(ThreadContext_X          )]
    stp      x2,  x3, [x18, #(ThreadContext_X +  1 * 16)]
    stp      x4,  x5, [x18, #(ThreadContext_X +  2 * 16)]
    stp      x6,  x7, [x18, #(ThreadContext_X +  3 * 16)]
    stp      x8,  x9, [x18, #(ThreadContext_X +  4 * 16)]
    stp     x10, x11, [x18, #(ThreadContext_X +  5 * 16)]
    stp     x12, x13, [x18, #(ThreadContext_X +  6 * 16)]
    stp     x14, x15, [x18, #(ThreadContext_X +  7 * 16)]
    // x16-18 and x30 are saved elsewhere, to free up registers for a context switch
    // x19 is just along for the ride. Consider moving x30 here, even if it looks ugly.
    //stp     x16, x17, [x18, #(ThreadContext_X +  8 * 16)]
    //stp     x18, x19, [x18, #(ThreadContext_X +  9 * 16)]
    //str     x30,      [x18, #(ThreadContext_X + 15 * 16)]
    stp     x20, x21, [x18, #(ThreadContext_X + 10 * 16)]
    stp     x22, x23, [x18, #(ThreadContext_X + 11 * 16)]
    stp     x24, x25, [x18, #(ThreadContext_X + 12 * 16)]
    stp     x26, x27, [x18, #(ThreadContext_X + 13 * 16)]
    stp     x28, x29, [x18, #(ThreadContext_X + 14 * 16)]
    // We got sp in x17
    str     x17,      [x18, #(ThreadContext_Sp)]
    mrs      x0, ELR_EL1
    str      x0,      [x18, #(ThreadContext_Pc)]
    mrs      x0, SPSR_EL1
    str      x0,      [x18, #(ThreadContext_Spsr)]
    stp      q0,  q1, [x18, #(ThreadContext_V          )]
    stp      q2,  q3, [x18, #(ThreadContext_V +  1 * 32)]
    stp      q4,  q5, [x18, #(ThreadContext_V +  2 * 32)]
    stp      q6,  q7, [x18, #(ThreadContext_V +  3 * 32)]
    stp      q8,  q9, [x18, #(ThreadContext_V +  4 * 32)]
    stp     q10, q11, [x18, #(ThreadContext_V +  5 * 32)]
    stp     q12, q13, [x18, #(ThreadContext_V +  6 * 32)]
    stp     q14, q15, [x18, #(ThreadContext_V +  7 * 32)]
    stp     q16, q17, [x18, #(ThreadContext_V +  8 * 32)]
    stp     q18, q19, [x18, #(ThreadContext_V +  9 * 32)]
    stp     q20, q21, [x18, #(ThreadContext_V + 10 * 32)]
    stp     q22, q23, [x18, #(ThreadContext_V + 11 * 32)]
    stp     q24, q25, [x18, #(ThreadContext_V + 12 * 32)]
    stp     q26, q27, [x18, #(ThreadContext_V + 13 * 32)]
    stp     q28, q29, [x18, #(ThreadContext_V + 14 * 32)]
    stp     q30, q31, [x18, #(ThreadContext_V + 15 * 32)]
    ret

// Pop the entire set of volatile registers onto the stack.
GetAllRegistersFromThreadContext:
    ldp     q30, q31, [x18, #(ThreadContext_V + 15 * 32)]
    ldp     q28, q29, [x18, #(ThreadContext_V + 14 * 32)]
    ldp     q26, q27, [x18, #(ThreadContext_V + 13 * 32)]
    ldp     q24, q25, [x18, #(ThreadContext_V + 12 * 32)]
    ldp     q22, q23, [x18, #(ThreadContext_V + 11 * 32)]
    ldp     q20, q21, [x18, #(ThreadContext_V + 10 * 32)]
    ldp     q18, q19, [x18, #(ThreadContext_V +  9 * 32)]
    ldp     q16, q17, [x18, #(ThreadContext_V +  8 * 32)]
    ldp     q14, q15, [x18, #(ThreadContext_V +  7 * 32)]
    ldp     q12, q13, [x18, #(ThreadContext_V +  6 * 32)]
    ldp     q10, q11, [x18, #(ThreadContext_V +  5 * 32)]
    ldp      q8,  q9, [x18, #(ThreadContext_V +  4 * 32)]
    ldp      q6,  q7, [x18, #(ThreadContext_V +  3 * 32)]
    ldp      q4,  q5, [x18, #(ThreadContext_V +  2 * 32)]
    ldp      q2,  q3, [x18, #(ThreadContext_V +  1 * 32)]
    ldp      q0,  q1, [x18, #(ThreadContext_V          )]
    ldr      x0,      [x18, #(ThreadContext_Spsr)]
    msr      SPSR_EL1, x0
    ldr      x0,      [x18, #(ThreadContext_Pc)]
    msr      ELR_EL1, x0
    // We put sp in x17
    ldr     x17,      [x18, #(ThreadContext_Sp)]
    mov     sp,  x17
    ldp     x28, x29, [x18, #(ThreadContext_X + 14 * 16)]
    ldp     x26, x27, [x18, #(ThreadContext_X + 13 * 16)]
    ldp     x24, x25, [x18, #(ThreadContext_X + 12 * 16)]
    ldp     x22, x23, [x18, #(ThreadContext_X + 11 * 16)]
    ldp     x20, x21, [x18, #(ThreadContext_X + 10 * 16)]
    // Note: assuming x18 remains unchanged.
    // TODO: Do better without the assumption?
    ldp     x18, x19, [x18, #(ThreadContext_X +  9 * 16)]
    ldp     x16, x17, [x18, #(ThreadContext_X +  8 * 16)]
    // x30 (LR) needs to be restored elsewhere, because we're using it here.
    //ldr     x30,      [x18, #(ThreadContext_X + 15 * 16)]
    ldp     x14, x15, [x18, #(ThreadContext_X +  7 * 16)]
    ldp     x12, x13, [x18, #(ThreadContext_X +  6 * 16)]
    ldp     x10, x11, [x18, #(ThreadContext_X +  5 * 16)]
    ldp      x8,  x9, [x18, #(ThreadContext_X +  4 * 16)]
    ldp      x6,  x7, [x18, #(ThreadContext_X +  3 * 16)]
    ldp      x4,  x5, [x18, #(ThreadContext_X +  2 * 16)]
    ldp      x2,  x3, [x18, #(ThreadContext_X +  1 * 16)]
    ldp      x0,  x1, [x18, #(ThreadContext_X          )]
    ret

    // Exception vectors must be properly aligned to 2048 bytes.
    // Each vector must be aligned to 128 bytes, leaving room for up to 32 instructions per vector.
    // Vectors are organized of four groups of four vectors each for a total of 16.
    .align 11
.global ExceptionVectors
ExceptionVectors:

    // synchronous
    .align  7
SPEL0SynchronousExceptionVector:
    mov x1, sp
    and x0, x1, #0xF
    sub sp, x1, x0
    mov     x0, #0
    bl       MainExceptionHandler
    eret

    // IRQ
    .align  7
SPEL0IrqExceptionVector:
    mov     x0, #1
    bl       MainExceptionHandler
    eret

    // FIQ
    .align  7
SPEL0FiqExceptionVector:
    mov     x0, #2
    bl       MainExceptionHandler
    eret

    // SError
    .align  7
SPEL0ErrorExceptionVector:
    mov     x0, #3
    bl       MainExceptionHandler
    eret

    // synchronous
    .align  7
SynchronousExceptionVector:
// Align the stack?
//    mov x1, sp
//    and x0, x1, #0xF
//    sub sp, x1, x0

    // 32 instructions max!
    // Preserve these first. They'll become available to use.
    stp     x16, x17, [sp, #-16]!
    stp     x18, x30, [sp, #-16]!
    bl      PreserveVolatileRegisters
    mrs     x9, ESR_EL1
    lsr     x10, x9, #26
    and     x10, x10, #0x3F
    cmp     x10, #0x15
    b.ne    1f
    bl      SyscallDispatcher
    b       2f
1:  mov     x0, #0x10
    bl      MainExceptionHandler
2:  mov     x16, x0
    bl      RestoreVolatileRegisters
    cbnz    x16, CompleteContextSwitch
    ldp     x18, x30, [sp], #16
    ldp     x16, x17, [sp], #16
    eret

    // IRQ
    .align  7
IrqExceptionVector:
    // 32 instructions max!
    // Preserve these first. They'll become available to use.
    stp     x16, x17, [sp, #-16]!
    stp     x18, x30, [sp, #-16]!
    bl      PreserveVolatileRegisters
    mov     x0, #0x11
    bl      InterruptDispatcher
    mov     x16, x0
    bl      RestoreVolatileRegisters
    cbnz    x16, CompleteContextSwitch
    ldp     x18, x30, [sp], #16
    ldp     x16, x17, [sp], #16
    eret

    // FIQ
    .align  7
FiqExceptionVector:
    mov     x0, #0x12
    bl       MainExceptionHandler
    eret

    // SError
    .align  7
ErrorExceptionVector:
    mov     x0, #0x13
    bl       MainExceptionHandler
    eret

    // synchronous
    .align  7
G2SynchronousExceptionVector:
    mov     x0, #0x20
    bl       MainExceptionHandler
    eret

    // IRQ
    .align  7
G2IrqExceptionVector:
    mov     x0, #0x21
    bl       MainExceptionHandler
    eret

    // FIQ
    .align  7
G2FiqExceptionVector:
    mov     x0, #0x22
    bl       MainExceptionHandler
    eret

    // SError
    .align  7
G2ErrorExceptionVector:
    mov     x0, #0x23
    bl       MainExceptionHandler
    eret

    // synchronous
    .align  7
G3SynchronousExceptionVector:
    mov     x0, #0x30
    bl       MainExceptionHandler
    eret

    // IRQ
    .align  7
G3IrqExceptionVector:
    mov     x0, #0x31
    bl       MainExceptionHandler
    eret

    // FIQ
    .align  7
G3FiqExceptionVector:
    mov     x0, #0x32
    bl       MainExceptionHandler
    eret

    // SError
    .align  7
G3ErrorExceptionVector:
    mov     x0, #0x33
    bl       MainExceptionHandler
    eret

CompleteContextSwitch:
    // We'll switch the context. x16 has the new ThreadInfo.
    // We need to put the actual SP in X17.
    // We haven't popped x16-18 or x30(lr) yet, so we skip over them.
    add     x17, sp, #32
    ldp     x18, x30, [sp], #16
    stp     x18, x19, [x18, #(ThreadContext_X +  9 * 16)]
    str     x30,      [x18, #(ThreadContext_X + 15 * 16)]
    bl      MoveAllRegistersToThreadContext
    // Add x16 and x17 now.
    ldp      x0,  x1, [sp], #16
    stp      x0,  x1, [x18, #(ThreadContext_X +  8 * 16)]
    mov     x18, x16
    bl      GetAllRegistersFromThreadContext
    ldr     x30,      [x18, #(ThreadContext_X + 15 * 16)]
    eret
