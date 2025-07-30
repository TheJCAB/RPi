#include "ThreadContext.h"

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
    mov fp, #0
    mov lr, #0

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
    mov     x1, #0
    bl      MainExceptionHandler
    eret

    // IRQ
    .align  7
SPEL0IrqExceptionVector:
    mov     x1, #1
    bl      MainExceptionHandler
    eret

    // FIQ
    .align  7
SPEL0FiqExceptionVector:
    mov     x1, #2
    bl      MainExceptionHandler
    eret

    // SError
    .align  7
SPEL0ErrorExceptionVector:
    mov     x1, #3
    bl      MainExceptionHandler
    eret

    // synchronous
    .align  7
SynchronousExceptionVector:
    // Detect the user mode spark return first.
    stp     x0, x1, [sp, #-16]!
    mrs     x0, ESR_EL1
    ubfx    x1, x0, #26, #6
    sub     x1, x1, #0x15
    cbz     x1, DispatchToSyscall
    ldp     x0, x1, [sp], #16

    // 32 instructions max!
    // Preserve the parameter registers first.
    stp      q0,  q1, [sp, #-ThreadContext_VSize]!
    stp      q2,  q3, [sp, #( 2 * 16)]
    stp      q4,  q5, [sp, #( 4 * 16)]
    stp      q6,  q7, [sp, #( 6 * 16)]
    stp      x0,  x1, [sp, #-ThreadContext_V]!
    stp      x2,  x3, [sp, #( 2 *  8)]
    stp      x4,  x5, [sp, #( 4 *  8)]
    stp      x6,  x7, [sp, #( 6 *  8)]
    // And LR which we're about to use.
    str     x30,      [sp, #(30 *  8)]

    // Otherwise this is like an interrupt, we'll need to preserve registers
    bl      PreserveVolatileRegisters
    bl      PreserveNonvolatileRegisters
    mov     x0, sp
    mov     x1, #0x10
    bl      MainExceptionHandler

    cbz     x1, RestoreAllRegistersAndERet // No spark, so exit. This should be common.
2:
    tbnz    x1, #0, RunUserModeSpark  // Bit 0 is set for user-mode sparks.
    blr     x1
    cbnz    x1, 2b
    b       RestoreAllRegistersAndERet

    // IRQ
    .align  7
IrqExceptionVector:
    // 32 instructions max!
    // Preserve the parameter registers first.
    stp      q0,  q1, [sp, #-ThreadContext_VSize]!
    stp      q2,  q3, [sp, #( 2 * 16)]
    stp      q4,  q5, [sp, #( 4 * 16)]
    stp      q6,  q7, [sp, #( 6 * 16)]
    stp      x0,  x1, [sp, #-ThreadContext_V]!
    stp      x2,  x3, [sp, #( 2 *  8)]
    stp      x4,  x5, [sp, #( 4 *  8)]
    stp      x6,  x7, [sp, #( 6 *  8)]
    // And LR which we're about to use.
    str     x30,      [sp, #(30 *  8)]
    mov     x1, #0x11
    bl      PreserveVolatileRegisters
    bl      PreserveNonvolatileRegisters // Not strictly necessary, but helps with debugging.
//mov x0,#'#'
//bl RawPutc

1:
    mov     x0, sp
    bl      InterruptDispatcher
    cbz     x1, GetAndRunNextScheduledSpark // RestoreVolatilesAndERet // No spark, so exit. This should be common.
2:
    tbnz    x1, #0, RunUserModeSpark  // Bit 0 is set for user-mode sparks.
    blr     x1
    cbnz    x1, 2b
    b       1b

    // FIQ
    .align  7
FiqExceptionVector:
    mov     x1, #0x12
    bl      MainExceptionHandler
    eret

    // SError
    .align  7
ErrorExceptionVector:
    mov     x1, #0x13
    bl      MainExceptionHandler
    eret

    // synchronous
    .align  7
G2SynchronousExceptionVector:
    mov     x1, #0x20
    bl      MainExceptionHandler
    eret

    // IRQ
    .align  7
G2IrqExceptionVector:
    mov     x1, #0x21
    bl      MainExceptionHandler
    eret

    // FIQ
    .align  7
G2FiqExceptionVector:
    mov     x1, #0x22
    bl      MainExceptionHandler
    eret

    // SError
    .align  7
G2ErrorExceptionVector:
    mov     x1, #0x23
    bl      MainExceptionHandler
    eret

    // synchronous
    .align  7
G3SynchronousExceptionVector:
    mov     x1, #0x30
    bl      MainExceptionHandler
    eret

    // IRQ
    .align  7
G3IrqExceptionVector:
    mov     x1, #0x31
    bl      MainExceptionHandler
    eret

    // FIQ
    .align  7
G3FiqExceptionVector:
    mov     x1, #0x32
    bl      MainExceptionHandler
    eret

    // SError
    .align  7
G3ErrorExceptionVector:
    mov     x1, #0x33
    bl      MainExceptionHandler
    eret

// TODO: Consider using the aarch64_vector_pcs calling convention everywhere,
// which would require saving less registers here.
// https://clang.llvm.org/docs/AttributeReference.html#id430
// https://github.com/ARM-software/abi-aa/blob/main/vfabia64/vfabia64.rst#definitions

// Zero out the entire set of volatile registers on the stack, in ThreadContext layout.
// Except the arguments x0-7 and q0-7 which were preserved with proper values
// and may have been updated with result values.
ZeroVolatileRegisters:
    mov      x9, #0
    stp      x9,  x9, [sp, #( 8 * 8)]
    stp      x9,  x9, [sp, #(10 * 8)]
    stp      x9,  x9, [sp, #(12 * 8)]
    stp      x9,  x9, [sp, #(14 * 8)]
    stp      x9,  x9, [sp, #(16 * 8)]
    add      x8,  sp, #ThreadContext_V // Can't reach far enough using SP.
    dup      v8.2D, x9
    stp      q8,  q8, [x8, #( 8 * 16)]
    stp      q8,  q8, [x8, #(10 * 16)]
    stp      q8,  q8, [x8, #(12 * 16)]
    stp      q8,  q8, [x8, #(14 * 16)]
    stp      q8,  q8, [x8, #(16 * 16)]
    stp      q8,  q8, [x8, #(18 * 16)]
    stp      q8,  q8, [x8, #(20 * 16)]
    stp      q8,  q8, [x8, #(22 * 16)]
    stp      q8,  q8, [x8, #(24 * 16)]
    stp      q8,  q8, [x8, #(26 * 16)]
    stp      q8,  q8, [x8, #(28 * 16)]
    stp      q8,  q8, [x8, #(30 * 16)]
    ret

// Write the entire set of volatile registers onto the stack, in ThreadContext layout.
// Except the arguments x0-7 and q0-7 which have already been done.
PreserveVolatileRegisters:
    stp     x8 , x9 , [sp, #( 8 * 8)]
    stp     x10, x11, [sp, #(10 * 8)]
    stp     x12, x13, [sp, #(12 * 8)]
    stp     x14, x15, [sp, #(14 * 8)]
    stp     x16, x17, [sp, #(16 * 8)]
    add      x8, sp, #ThreadContext_V // Can't reach far enough using SP.
    stp      q8,  q9, [x8, #( 8 * 16)]
    stp     q10, q11, [x8, #(10 * 16)]
    stp     q12, q13, [x8, #(12 * 16)]
    stp     q14, q15, [x8, #(14 * 16)]
    stp     q16, q17, [x8, #(16 * 16)]
    stp     q18, q19, [x8, #(18 * 16)]
    stp     q20, q21, [x8, #(20 * 16)]
    stp     q22, q23, [x8, #(22 * 16)]
    stp     q24, q25, [x8, #(24 * 16)]
    stp     q26, q27, [x8, #(26 * 16)]
    stp     q28, q29, [x8, #(28 * 16)]
    stp     q30, q31, [x8, #(30 * 16)]
    ret

// Read the entire set of volatile registers from the stack, in ThreadContext layout.
// Except the arguments x0-7 and q0-7 which have already been done.
// TODO: AAPCS indicates x0-7 can be used to return, but in practice only x0-1 seem to be used.
// So we could tighten things up here.
// 
RestoreVolatileRegisters:
    // TODO: Use a flag to indicate volatile registers should be zeroed instead of restored.
    add      x8, sp, #ThreadContext_V // Can't reach far enough using SP.
    ldp     q30, q31, [x8, #(30 * 16)]
    ldp     q28, q29, [x8, #(28 * 16)]
    ldp     q26, q27, [x8, #(26 * 16)]
    ldp     q24, q25, [x8, #(24 * 16)]
    ldp     q22, q23, [x8, #(22 * 16)]
    ldp     q20, q21, [x8, #(20 * 16)]
    ldp     q18, q19, [x8, #(18 * 16)]
    ldp     q16, q17, [x8, #(16 * 16)]
    ldp     q14, q15, [x8, #(14 * 16)]
    ldp     q12, q13, [x8, #(12 * 16)]
    ldp     q10, q11, [x8, #(10 * 16)]
    ldp      q8,  q9, [x8, #( 8 * 16)]
    ldp     x16, x17, [sp, #(16 * 8)]
    ldp     x14, x15, [sp, #(14 * 8)]
    ldp     x12, x13, [sp, #(12 * 8)]
    ldp     x10, x11, [sp, #(10 * 8)]
    ldp     x8 , x9 , [sp, #( 8 * 8)]
    ret

// Write the entire set of nonvolatile registers onto the stack (context).
PreserveNonvolatileRegisters:
    stp     x18, x19, [sp, #(18 * 8)]
    stp     x20, x21, [sp, #(20 * 8)]
    stp     x22, x23, [sp, #(22 * 8)]
    stp     x24, x25, [sp, #(24 * 8)]
    stp     x26, x27, [sp, #(26 * 8)]
    stp     x28, x29, [sp, #(28 * 8)]
    // Note: we preserve the original SP (after popping the context).
    add      x8, sp, #ThreadContext_Size
    str      x8,      [sp, #(ThreadContext_Sp)]
    mrs      x8, ELR_EL1
    str      x8,      [sp, #(ThreadContext_Pc)]
    mrs      x8, SPSR_EL1
    str      x8,      [sp, #(ThreadContext_Spsr)]
    ret

// Read the entire set of volatile registers from the stack (context).
RestoreNonvolatileRegisters:
    ldr      x0,      [sp, #(ThreadContext_Spsr)]
    msr      SPSR_EL1, x0
    ldr      x0,      [sp, #(ThreadContext_Pc)]
    msr      ELR_EL1, x0
    ldp     x28, x29, [sp, #(28 * 8)]
    ldp     x26, x27, [sp, #(26 * 8)]
    ldp     x24, x25, [sp, #(24 * 8)]
    ldp     x22, x23, [sp, #(22 * 8)]
    ldp     x20, x21, [sp, #(20 * 8)]
    ldp     x18, x19, [sp, #(18 * 8)]
    ret

// Back to "user mode" to run a spark in x1 (with bit zero set) and context in x0.
// All "user mode" registers must be preserved in a context just pushed to the stack.
RunUserModeSpark:
    and     x1, x1, #~1
    msr     ELR_EL1, x1
    adr     lr, UserModeSparkEnd
    eret

// We need to dispatch a syscall.
// Syndrome in x0, and x0 and z1 pushed on the stack.
// Volatile registers can be trashed, as this represents a function call in "user mode".
DispatchToSyscall:
    ubfx    x8, x0, #0, #16
    ldp     x0, x1, [sp], #16
    cbz     x8, GetAndRunNextScheduledSpark

    stp      q0,  q1, [sp, #-ThreadContext_VSize]!
    stp      q2,  q3, [sp, #( 2 * 16)]
    stp      q4,  q5, [sp, #( 4 * 16)]
    stp      q6,  q7, [sp, #( 6 * 16)]
    stp      x0,  x1, [sp, #-ThreadContext_V]!
    stp      x2,  x3, [sp, #( 2 *  8)]
    stp      x4,  x5, [sp, #( 4 *  8)]
    stp      x6,  x7, [sp, #( 6 *  8)]
    // And LR which we're about to use.
    str     x30,      [sp, #(30 *  8)]
    bl      PreserveNonvolatileRegisters

    mov     x0, sp // ThreadContext as a parameter.
    bl      SyscallDispatcher
    b       RunSpark

// We return from the user-mode spark with a SVC #0.
// We don't need to preserve anything, just see if there are more sparks to run.
GetAndRunNextScheduledSpark:
    bl      GetNextScheduledSpark

RunSpark:
    cbz     x1, RestoreAllRegistersAndERet
    tbnz    x1, #0, RunUserModeSpark  // Bit 0 is set for user-mode sparks.
    blr     x1
    cbnz    x1, RunSpark

    // No more "kernel mode" sparks.
    // Go again to see if there are more "user mode" sparks.
    b       GetAndRunNextScheduledSpark

.global ThreadContextSwitchSpark
ThreadContextSwitchSpark:
    // We'll switch the context. x0 has the new ThreadInfo.
    // We need to put the actual SP in X17.
    // We haven't popped x16-18 or x30(lr) yet, so we skip over them.
    bl      PreserveNonvolatileRegisters
    // And put the stack pointer (pointing to the context) in the ThreadInfo
    mov     x8, sp
    str     x8, [x18]

    mov     sp, x0
    // fallthrough

RestoreAllRegistersAndERet:
    bl      RestoreNonvolatileRegisters
    // fallthrough

RestoreVolatilesAndERet:
//    mov x0,#'.'
//    bl RawPutc
    bl      RestoreVolatileRegisters

RestoreParametersAndERet:
    ldr     x30,      [sp, #(30 *  8)]
    ldp      x6,  x7, [sp, #( 6 *  8)]
    ldp      x4,  x5, [sp, #( 4 *  8)]
    ldp      x2,  x3, [sp, #( 2 *  8)]
    ldp      x0,  x1, [sp], #ThreadContext_V
    ldp      q6,  q7, [sp, #( 6 * 16)]
    ldp      q4,  q5, [sp, #( 4 * 16)]
    ldp      q2,  q3, [sp, #( 2 * 16)]
    ldp      q0,  q1, [sp], #ThreadContext_VSize
    eret
