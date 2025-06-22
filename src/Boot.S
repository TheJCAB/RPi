.section .text

.extern KernelMain

.global _start
_start:
    adr x0, _start // Stack buffer is the 512 KB space before the kernel image.
    mov sp, x0
    b   KernelMain

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
    mov     x0, #0x11
    bl       MainExceptionHandler
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
