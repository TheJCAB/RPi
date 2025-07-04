.section .text

.macro NOP_LOOP count=(150*2)
    mov x28, #\count
1:
    nop
    subs x28, x28, #1
    bne 1b
.endm

#define PL011_DR        0x00
#define PL011_FR        0x18
#define PL011_IBRD      0x24
#define PL011_FBRD      0x28
#define PL011_LCRH      0x2C
#define PL011_CR        0x30
#define PL011_IMSC      0x38
#define PL011_ICR       0x44

#define GPFSEL1         (0x04-0x80)
#define GPSET0          (0x1C-0x80)
#define GPCLR0          (0x28-0x80)
#define GPPUD           (0x94-0x80)
#define GPPUDCLK0       (0x98-0x80)
#define GPPUPPDN0       (0xE4-0x80)

.macro Put ch='X'
    // Wait until transmitter FIFO has space
1:
    ldr w5, [x20, #PL011_FR]
    tst w5, #(1 << 5)
    beq 1b

    mov w5, \ch
    str w5, [x20, #PL011_DR]
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
    ldr w5, [x20, #PL011_FR]
    tst w5, #0x10
    beq 1b

    ldr w5, [x20, #PL011_DR]
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

//    // Detect Raspberry Pi 4 by checking the ARM CPU ID
//    mrs x6, midr_el1
//    lsr x6, x6, #4
//    and x6, x6, #0xFFF
//    cmp x6, #0xD08  // Cortex-A72 part number
//    b.ne 1f
//
//    // This is RPi4, using high addresses
//    ldr x20, =0x47E201000 // UART
//    ldr x21, =0x47E200080 // GPIO
//
//    ldr w5, [x21, #GPPUPPDN0]
//    and w5, w5, #0x0FFFFFFF
//    str w5, [x21, #GPPUPPDN0]
//
//    b 2f
//1:

    // This is RPi3
    ldr x20, =0x3F201000 // UART
    ldr x21, =0x3F200080 // GPIO

    str wzr, [x21, #GPPUD]
    NOP_LOOP
    mov w5, #((1 << 14) | (1 << 15))
    str w5, [x21, #GPPUDCLK0]
    NOP_LOOP
    str wzr, [x21, #GPPUDCLK0]

2:

    ldr w5, [x21, #GPFSEL1]
    and w5, w5, #(~((7 << 12) | (7 << 15))) // clear bits for GPIO14, GPIO15
    //mov w4, #((4 << 0) | (4 << 3))
    //orr w5, w5, w4, LSL #12
    orr w5, w5, #(4 << 12)
    orr w5, w5, #(4 << 15)
    str w5, [x21, #GPFSEL1]

    // Disable UART first
    str wzr, [x20, #PL011_CR]

    // Wait for UART to finish transmitting/receiving
1:
    ldr w5, [x20, #PL011_FR]
    tst w5, #((1 << 3) | (1 << 4))  // Check BUSY and RXFE flags
    b.ne 1b

    // Disable FIFO & 8 bit data transmission (1 stop bit, no parity)
    mov w5, #(3 << 5)
    str w5, [x20, #PL011_LCRH]

    // Clear pending interrupts
    mov w5, #0x7FF
    str w5, [x20, #PL011_ICR]

    // Set integer & fractional part of baud rate
    // Baud = 115200, UARTCLK = 48 MHz (default for Pi 3)
    // Divider = UARTCLK / (16 * Baud) = 48,000,000 / (16*115200) = 26.0416
    mov w5, #26
    str w5, [x20, #PL011_IBRD]
    mov w5, #3
    str w5, [x20, #PL011_FBRD]

    // Enable FIFO & 8 bit data transmission (1 stop bit, no parity)
    mov w5, #((1 << 4) | (3 << 5))
    str w5, [x20, #PL011_LCRH]

    // Mask all interrupts
    mov w5, #((1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10))
    str w5, [x20, #PL011_IMSC]

    // Enable UART0, receive & transmit
    mov w5, #((1 << 0) | (1 << 8) | (1 << 9))
    str w5, [x20, #PL011_CR]

    Puts "Hello world!!\r\n"

    adr x4, 2f
    adr x5, _end
    mov x7, #-0x4000
1:
    ldr x6, [x4]
    str x6, [x4, x7]
    add x4, x4, #8
    cmp x4, x5
    blt 1b

    adr x5, 2f
    sub x5, x5, #0x4000
    br x5

    .align 3
2:

    Puts "We're running from "
    adr x9, 4f
    PutDWordHex w9

    Put #3
    Put #3
    Put #3

    Get w11 // Sink one byte
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
