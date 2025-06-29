.section .text

.extern Core0
.extern Core1
.extern Core2
.extern Core3

.global _start
_start:
    mrs x0, mpidr_el1

    adr x1, _start // Stack buffer is the 512 KB space before the kernel image.
    and x0, x0, #3
    sub x1, x1, x0, LSL #16
    mov sp, x1

    adr x1, CoreFunctionTable
    ldr x0, [x1, w0, UXTW #3]
    br x0

CoreFunctionTable:
    .quad Core0
    .quad Core1
    .quad Core2
    .quad Core3


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

PreserveSimd:
    stp     q0, q1, [sp, #-32]!
    stp     q2, q3, [sp, #-32]!
    stp     q4, q5, [sp, #-32]!
    stp     q6, q7, [sp, #-32]!
    stp     q8, q9, [sp, #-32]!
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

RestoreSimd:
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
    ldp     q8, q9, [sp], #32
    ldp     q6, q7, [sp], #32
    ldp     q4, q5, [sp], #32
    ldp     q2, q3, [sp], #32
    ldp     q0, q1, [sp], #32
    ret

    // important, code has to be properly aligned
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
    mov x1, sp
    and x0, x1, #0xF
    sub sp, x1, x0
    mov     x0, #0x10
    bl       MainExceptionHandler
    eret

    // IRQ
    .align  7
IrqExceptionVector:
    // 32 instructions max!
    stp     x0 , x1 , [sp, #-16]!
    stp     x2 , x3 , [sp, #-16]!
    stp     x4 , x5 , [sp, #-16]!
    stp     x6 , x7 , [sp, #-16]!
    stp     x8 , x9 , [sp, #-16]!
    stp     x10, x11, [sp, #-16]!
    stp     x12, x13, [sp, #-16]!
    stp     x14, x15, [sp, #-16]!
    stp     x16, x17, [sp, #-16]!
    stp     x18, x30, [sp, #-16]!
    bl PreserveSimd
    mov     x0, #0x11
    bl       MainExceptionHandler
    bl RestoreSimd
    ldp     x18, x30, [sp], #16
    ldp     x16, x17, [sp], #16
    ldp     x14, x15, [sp], #16
    ldp     x12, x13, [sp], #16
    ldp     x10, x11, [sp], #16
    ldp     x8 , x9 , [sp], #16
    ldp     x6 , x7 , [sp], #16
    ldp     x4 , x5 , [sp], #16
    ldp     x2 , x3 , [sp], #16
    ldp     x0 , x1 , [sp], #16
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

