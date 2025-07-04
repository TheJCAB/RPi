
.section .text
.global _start

.equ PERIPHERAL_BASE, 0xFE000000
.equ AUX_BASE,        (PERIPHERAL_BASE + 0x00215000)
.equ AUX_ENABLES,     (AUX_BASE + 0x04)
.equ AUX_MU_IO_REG,   (AUX_BASE + 0x40)
.equ AUX_MU_IER_REG,  (AUX_BASE + 0x44)
.equ AUX_MU_LCR_REG,  (AUX_BASE + 0x4C)
.equ AUX_MU_CNTL_REG, (AUX_BASE + 0x60)
.equ AUX_MU_BAUD_REG, (AUX_BASE + 0x68)
.equ AUX_MU_LSR_REG,  (AUX_BASE + 0x54)

_start:
    ldr w0, =0x30000000
    ldr w1, =0x3FFFFFFF
    ldr w2, =0xF100F100
1:
    str w2, [x0]
    add x0, x0, #4
    cmp x0, x1
    blt 1b

    // Enable Mini UART
    ldr x0, =AUX_ENABLES
    mov x1, #1
    str w1, [x0]

    // Disable interrupts
    ldr x0, =AUX_MU_IER_REG
    mov x1, #0
    str w1, [x0]

    // Set 8-bit mode
    ldr x0, =AUX_MU_LCR_REG
    mov x1, #3
    str w1, [x0]

    // Disable TX/RX
    ldr x0, =AUX_MU_CNTL_REG
    mov x1, #0
    str w1, [x0]

    // Set baud rate (115200 assuming 500 MHz clock)
    ldr x0, =AUX_MU_BAUD_REG
    mov x1, #(500000000 / (8 * 115200) - 1)
    str w1, [x0]

    // Enable TX/RX
    ldr x0, =AUX_MU_CNTL_REG
    mov x1, #3
    str w1, [x0]

    // Print "Hello, World!\n"
    ldr x0, =message
print_loop:
    ldrb w1, [x0], #1
    cbz w1, done
wait:
    ldr x2, =AUX_MU_LSR_REG
    ldrb w3, [x2]
    tst w3, #0x20
    beq wait
    ldr x2, =AUX_MU_IO_REG
    strb w1, [x2]
    b print_loop

done:
    b .

.section .rodata
message:
    .ascii "Hello, World!\n\0"
