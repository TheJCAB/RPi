.section .text

.extern KernelMain
.extern StackBuffer

.global _start
_start:
    adr x0, StackBuffer
    add sp, x0, #0x10000
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