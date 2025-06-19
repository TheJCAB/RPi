.section .text

.extern KernelMain
.extern StackBuffer

.global _start
_start:
    adr x0, StackBuffer
    add sp, x0, #0x10000
    b   KernelMain
