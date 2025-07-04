.section .text
.global _start

// PL011 UART base address (for Pi 4)
.equ MMIO_BASE, 0x47E000000

// PL011 UART base address (for Pi 4)
.equ UART0_BASE, MMIO_BASE + 0x200000

// UART registers (offsets from UART0_BASE)
.equ UART_DR,     0x00
.equ UART_FR,     0x18
.equ UART_IBRD,   0x24
.equ UART_FBRD,   0x28
.equ UART_LCRH,   0x2C
.equ UART_CR,     0x30
.equ UART_IMSC,   0x38
.equ UART_ICR,    0x44


// Mailbox MMIO base (channel 8 = property channel)
.equ MAILBOX_BASE,     MMIO_BASE + 0x00B880
.equ MAILBOX_READ,     (MAILBOX_BASE + 0x00)
.equ MAILBOX_STATUS,   (MAILBOX_BASE + 0x18)
.equ MAILBOX_WRITE,    (MAILBOX_BASE + 0x20)
.equ MAILBOX_FULL,     (1 << 31)
.equ MAILBOX_EMPTY,    (1 << 30)
.equ MAILBOX_CHANNEL,  8

_start:
    // Prepare mailbox message
//    ldr x0, =mbox_buffer
//    mov x1, #9 * 4
//    str w1, [x0, #0]        // Total size
//    mov w1, #0
//    str w1, [x0, #4]        // Request code
//
//    ldr w1, =0x00038002     // Set Clock Rate tag
//    str w1, [x0, #8]
//    mov w1, #12
//    str w1, [x0, #12]       // Value buffer size
//    str w1, [x0, #16]       // Request size
//
//    mov w1, #2              // Clock ID: PL011 UART
//    str w1, [x0, #20]
//    ldr w1, =48000000       // Clock rate: 48 MHz
//    str w1, [x0, #24]
//    mov w1, #0              // Skip turbo: 0
//    str w1, [x0, #28]
//
//    mov w1, #0              // End tag
//    str w1, [x0, #32]

//    // Send message to mailbox
//    adr x1, fbuf
//    lsr x1, x1, #4          // Mailbox requires address >> 4
//    orr x1, x1, #MAILBOX_CHANNEL
//
//wait_write:
//    ldr x2, =MAILBOX_STATUS
//    ldr w3, [x2]
//    tst w3, #MAILBOX_FULL
//    b.ne wait_write
//
//    ldr x2, =MAILBOX_WRITE
//    str w1, [x2]
//
//wait_response:
//    ldr x2, =MAILBOX_STATUS
//    ldr w3, [x2]
//    tst w3, #MAILBOX_EMPTY
//    b.ne wait_response
//
//    ldr x2, =MAILBOX_READ
//    ldr w3, [x2]
//    and w3, w3, #0xF
//    cmp w3, #MAILBOX_CHANNEL
//    b.ne wait_response



    // Disable UART
    ldr x0, =UART0_BASE + UART_CR
    mov w1, #0
    str w1, [x0]

    // Clear interrupts
    ldr x0, =UART0_BASE + UART_ICR
    mov w1, #0x7FF
    str w1, [x0]

    // Set integer & fractional baud rate (115200 @ 48 MHz UART clock)
    ldr x0, =UART0_BASE + UART_IBRD
    mov w1, #26        // 48_000_000 / (16 * 115200) = ~26.04
    str w1, [x0]

    ldr x0, =UART0_BASE + UART_FBRD
    mov w1, #3         // Fractional part: (.04 * 64 + 0.5) = ~3
    str w1, [x0]

    // Line control: 8N1, FIFO enabled
    ldr x0, =UART0_BASE + UART_LCRH
    mov w1, #(0x3 << 5) | (1 << 4)  // 8-bit, FIFO
    str w1, [x0]

    // Enable UART, TX, RX
    ldr x0, =UART0_BASE + UART_CR
    mov w1, #(1 << 9) | (1 << 8) | 1
    str w1, [x0]

    // Print message
    ldr x0, =message
print_loop:
    ldrb w1, [x0], #1
    cbz w1, hang

wait_ready:
    ldr x2, =UART0_BASE + UART_FR
    ldr w3, [x2]
    tst w3, #(1 << 5)         // TXFF (Transmit FIFO Full)
    b.ne wait_ready

    ldr x2, =UART0_BASE + UART_DR
    strb w1, [x2]
    b print_loop

hang:
    b hang

message:
    .ascii "Hello, World!\n\0"

    .align 4
fbuf:
    .word 0 // Size
    .word 0 // Request

    .word 0x48003; .word 8; .word 0; .word 1280; .word 720; // Set phys size
    .word 0x48004; .word 8; .word 0; .word 1280; .word 720; // Set virt size
    .word 0x48005; .word 4; .word 0; .word 16; // Set depth
    .word 0x48006; .word 4; .word 0; .word 0; // Set pixel order
    .word 0x40001; .word 8; .word 0; .word 16; .word 0; // Allocate buffer
    .word 0x40008; .word 4; .word 0; .word 0; // Get pitch
    .word 0; // End tag


// Mailbox buffer (must be 16-byte aligned)
.align 4
mbox_buffer:
    .skip 9 * 4
