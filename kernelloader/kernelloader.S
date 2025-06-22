.section .text

.macro NOP_LOOP count=150
    mov x28, #\count
1:
    nop
    subs x28, x28, #1
    bne 1b
.endm

#define AUX_ENABLES     0x04
#define AUX_MU_IO_REG   0x40
#define AUX_MU_IER_REG  0x44
#define AUX_MU_IIR_REG  0x48
#define AUX_MU_LCR_REG  0x4C
#define AUX_MU_MCR_REG  0x50
#define AUX_MU_LSR_REG  0x54
#define AUX_MU_CNTL_REG 0x60
#define AUX_MU_BAUD_REG 0x68

.macro Put ch='X'
    // Wait until transmitter FIFO has space
1:
    ldr w5, [x4, #AUX_MU_LSR_REG]
    tst w5, #(1 << 5)
    beq 1b

    mov w5, \ch
    str w5, [x4, #AUX_MU_IO_REG]
.endm

.macro PutByteHex reg
    // Print a byte in hexadecimal (two ASCII chars)
    // Input: \reg = byte value (lower 8 bits)
    and w6, \reg, #0xFF
    lsr w7, w6, #4
    bl 1f
    and w7, w6, #0xF
    bl 1f
    b 3f
1:
    cmp w7, #9
    add w8, w7, #0x30
    ble 2f
    add w8, w7, #(0x41 - 10)
2:
    Put w8
    ret
3:
.endm

.macro PutDWordHex reg
    // Print a 32-bit value in hexadecimal (8 ASCII chars)
    // Input: \reg = 32-bit value
    mov w9, \reg
    lsr w8, w9, #24
    PutByteHex w8
    lsr w8, w9, #16
    and w8, w8, #0xFF
    PutByteHex w8
    lsr w8, w9, #8
    and w8, w8, #0xFF
    PutByteHex w8
    and w8, w9, #0xFF
    PutByteHex w8
.endm

.macro Get reg
    // Wait until receiver FIFO has data
1:
    ldr w5, [x4, #AUX_MU_LSR_REG]
    tst w5, #1
    beq 1b

    ldr w5, [x4, #AUX_MU_IO_REG]
    and \reg, w5, #0xFF
.endm

// Usage: Puts "Hello, world!"
.macro Puts str
    adr x6, 1f
    b 2f
1:
    .asciz "\str"

    .align 2
2:
    ldrb w7, [x6], #1
    cbz w7, 3f
    Put w7
    b 2b
3:
.endm

.global _start
_start:
    adr x4, _start // Stack buffer is the 512 KB space before the kernel image.
    mov sp, x4

    adr x4, 2f
    adr x5, _end
    mov x7, #-0x1000
1:
    ldr x6, [x4]
    str x6, [x4, x7]
    add x4, x4, #8
    cmp x4, x5
    blt 1b

    adr x5, 3f
    sub x5, x5, #0x1000
    br x5

    .align 3
2:

MMIO_BASE:
    .word 0x3F215000

GP_BASE:
    .word 0x3F200080

#define GPFSEL1         (0x04-0x80)
#define GPPUD           (0x94-0x80)
#define GPPUDCLK0       (0x98-0x80)

3:
    ldr w4, GP_BASE
    str wzr, [x4, #GPPUD]
    NOP_LOOP
    mov w5, #((1 << 14) | (1 << 15))
    str w5, [x4, #GPPUDCLK0]
    NOP_LOOP
    str wzr, [x4, #GPPUDCLK0]

    ldr w5, [x4, #GPFSEL1]
    and w5, w5, #(~((7 << 12) | (7 << 15))) // clear bits for GPIO14, GPIO15
    orr w5, w5, #(2 << 12)    // set ALT5
    orr w5, w5, #(2 << 15)
    str w5, [x4, #GPFSEL1]

    ldr w4, MMIO_BASE

    // Enable Mini UART
    ldr w5, [x4, #AUX_ENABLES]
    orr w5, w5, #1
    str w5, [x4, #AUX_ENABLES]

    // Disable TX/RX
    str wzr, [x4, #AUX_MU_CNTL_REG]

    // Disable interrupts
    str wzr, [x4, #AUX_MU_IER_REG]

    // Enable 8-bit mode
    mov w5, #3
    str w5, [x4, #AUX_MU_LCR_REG]

    // RTS line high
    str wzr, [x4, #AUX_MU_MCR_REG]

    // Clear FIFO
    mov w5, #0xC6
    str w5, [x4, #AUX_MU_IIR_REG]

    // Set baud rate to 115200 (assuming 400 MHz system clock)
    mov w5, #((400000000 / (8 * 115200)) - 1)
    str w5, [x4, #AUX_MU_BAUD_REG]

    // Enable TX/RX
    mov w5, #3
    str w5, [x4, #AUX_MU_CNTL_REG]

    Puts "Hello world!!\r\n"

    Puts "We're running from "
    adr x9, 4f
    PutDWordHex w9

    Put #3
    Put #3
    Put #3

    Get w11
    Get w11
    Get w10
    lsl w10, w10, #8
    add w11, w11, w10
    Get w10
    lsl w10, w10, #16
    add w11, w11, w10
    Get w10
    lsl w10, w10, #24
    add w11, w11, w10

    Puts "OK"

    Puts "Size: "
    PutDWordHex w11
    Puts "\r\n"

    mov x13, sp
    mov x12, #0
4:
    Get w10
    strb w10, [x13, x12]
    add x12, x12, #1

//    tst x12, #0x4
//    bne 5f
//    Put #' '
//
//    tst x12, #0x10
//    bne 5f
//    Puts "\r\n"
//5:
    cmp x12, x11
    blt 4b

    mov x13, sp
    mov x12, #0

    Puts "Base: "
    PutDWordHex w13
    Puts "\r\n"

//6:
//    ldrb w10, [x13, x12]
//    PutByteHex w10
//    Put #' '
//    add x12, x12, #1
//    cmp x12, x11
//    blt 6b


    Puts "Jumping into the new kernel...\r\n"

    br x13


//    Puts "Size is: "
//
//    // Print 4-digit decimal number in w11
//    mov w10, #1000
//    udiv w12, w11, w10         // w12 = thousands
//    add w13, w12, #'0'
//    Put w13
//    msub w11, w12, w10, w11    // w11 = w11 - w12 * w10
//
//    mov w10, #100
//    udiv w12, w11, w10         // w12 = hundreds
//    add w13, w12, #'0'
//    Put w13
//    msub w11, w12, w10, w11    // w11 = w11 - w12 * w10
//
//    mov w10, #10
//    udiv w12, w11, w10         // w12 = tens
//    add w13, w12, #'0'
//    Put w13
//    msub w11, w12, w10, w11    // w11 = w11 - w12 * w10
//
//    add w13, w11, #'0'         // w11 = units
//    Put w13

//    Puts "\r\n"

    .align 3
_end: