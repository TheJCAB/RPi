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

    // important, code has to be properly aligned
    .align 11
.global ExceptionVectors
ExceptionVectors:

    // synchronous
    .align  7
SPEL0SynchronousExceptionVector:
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
    mov     x0, #0x11
    bl       MainExceptionHandler
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
