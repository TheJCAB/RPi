#include <BootLib/Uart.h>

#include <BootLib/Cpu.h>
#include <BootLib/Mmio.h>
#include <BootLib/Gpio.h>

#include <atomic>

namespace BootLib
{

union PL011Uart::PL011Registers
{
    Register<uint32_t, 0x00> DR   ; // Data Register
    Register<uint32_t, 0x04> RSR  ; // Receive Status Register
    Register<uint32_t, 0x18> FR   ; // Flag Register
    Register<uint32_t, 0x20> ILPR ; // IrDA Low-Power Counter Register
    Register<uint32_t, 0x24> IBRD ; // Integer Baud Rate Divisor
    Register<uint32_t, 0x28> FBRD ; // Fractional Baud Rate Divisor
    Register<uint32_t, 0x2C> LCRH ; // Line Control Register
    Register<uint32_t, 0x30> CR   ; // Control Register
    Register<uint32_t, 0x34> IFLS ; // Interrupt FIFO Level Select Register
    Register<uint32_t, 0x38> IMSC ; // Interrupt Mask Set/Clear Register
    Register<uint32_t, 0x3C> RIS  ; // Raw Interrupt Status Register
    Register<uint32_t, 0x40> MIS  ; // Masked Interrupt Status Register
    Register<uint32_t, 0x44> ICR  ; // Interrupt Clear Register
    Register<uint32_t, 0x48> DMACR; // DMA Control Register
};

void PL011Uart::Init()
{
    // Clear pending interrupts
    Registers.ICR = 0x7FF;

    // Set integer & fractional part of baud rate
    // Baud = 115200, UARTCLK = 48 MHz (default for Pi 3)
    // Divider = UARTCLK / (16 * Baud) = 48,000,000 / (16*115200) = 26.0416
    Registers.IBRD = 26;
    Registers.FBRD = 3;

    // Enable FIFO & 8 bit data transmission (1 stop bit, no parity)
    Registers.LCRH = (1 << 4) | (3 << 5); // FIFO enable, 8 bit

    // Mask all interrupts
    Registers.IMSC = (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
                 (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10);

    // Enable UART0, receive & transmit
    Registers.CR = (1 << 0) | (1 << 8) | (1 << 9);

    // Drain the receive FIFO.
    while (!(Registers.FR & 0x10)) {
        (void)Registers.DR.get();
    }
}

char PL011Uart::Getc()
{
    // Wait until data is ready in receiver FIFO
    while (Registers.FR & 0x10) {}
    return static_cast<char>(Registers.DR & 0xFF);
}

char PL011Uart::TryGetc()
{
    if (Registers.FR & 0x10)
    {
        return (char)0; // No data available
    }
    return static_cast<char>(Registers.DR & 0xFF);
}

void PL011Uart::Putc(char c)
{
    // Wait until transmitter FIFO has space
    while (Registers.FR & (1 << 5)) {}
    Registers.DR = c;
}

void PL011Uart::Puts(char const* str)
{
    while (*str)
    {
        Putc(*str++);
    }
}

void PL011Uart::PutHex(auto value)
{
    char const* hexDigits = "0123456789ABCDEF";

    Putc('0');
    Putc('x');
    for (int i = sizeof(value) * 8 - 4; i >= 0; i -= 4)
    {
        Putc(hexDigits[(value >> i) & 0xF]);
        if (i > 0 && i % 16 == 0)
        {
            Putc('\''); // Add digit separator for readability
        }
    }
}

void PL011Uart::PutBin(auto value)
{
    const char* binDigits = "01";
    Putc('0');
    Putc('b');
    for (int i = sizeof(value) * 8 - 1; i >= 0; --i)
    {
        Putc('0' + ((value >> i) & 0x1));
        if (i > 0 && i % 4 == 0)
        {
            Putc('\''); // Add digit separator for readability
        }
    }
}

void PL011Uart::PutDec(auto value)
{
    if (value == 0)
    {
        Putc('0');
        return;
    }

    char buffer[20]; // Enough for 64-bit integer
    int index = 0;

    while (value > 0)
    {
        buffer[index++] = '0' + (value % 10);
        value /= 10;
    }

    // Print in reverse order
    for (int i = index - 1; i >= 0; --i)
    {
        Putc(buffer[i]);
        if (i > 0 && i % 3 == 0)
        {
            Putc('\''); // Add digit separator for readability
        }
    }
}

template void PL011Uart::PutHex(uint64_t value);
template void PL011Uart::PutHex(uint32_t value);
template void PL011Uart::PutHex(uint16_t value);
template void PL011Uart::PutHex(uint8_t  value);
template void PL011Uart::PutHex(bool     value);

template void PL011Uart::PutBin(uint64_t value);
template void PL011Uart::PutBin(uint32_t value);
template void PL011Uart::PutBin(uint16_t value);
template void PL011Uart::PutBin(uint8_t  value);
template void PL011Uart::PutBin(bool     value);

template void PL011Uart::PutDec(uint64_t value);
template void PL011Uart::PutDec(uint32_t value);
template void PL011Uart::PutDec(uint16_t value);
template void PL011Uart::PutDec(uint8_t  value);
template void PL011Uart::PutDec(bool     value);

}
// namespace BootLib::Uart
