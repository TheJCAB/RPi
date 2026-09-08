#include <BootLib/Uart.h>

#include <BootLib/Cpu.h>
#include <BootLib/Gpio.h>
#include <BootLib/Mmio.h>

#include <atomic>

namespace BootLib::Uart
{

union MiniUart::Registers
{
    Register<uint32_t, 0x04> ENABLES;
    Register<uint32_t, 0x54> LSR;
};

void MiniUart::Disable(uintptr_t registersBase)
{
    auto& registers(*reinterpret_cast<Registers*>(registersBase));
    if (registers.ENABLES & 1u)
    {
        // It's enabled.
        // Drain the transmit FIFO.
        while (!(registers.LSR & 0x40)) { // IDLE
            Cpu::Yield();
        }

        registers.ENABLES &= ~1u; // Disable UART if it was enabled
    }
}

union PL011Uart::Registers
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

void PL011Uart::Disable(uintptr_t registersBase)
{
    auto& registers(*reinterpret_cast<Registers*>(registersBase));
    if (registers.CR & 1u)
    {
        // It's enabled.
        // Drain the transmit FIFO.
        while (registers.FR & 0x08) { // BUSY
            Cpu::Yield();
        }

        registers.CR &= ~1u; // Disable UART if it was enabled
    }
}

PL011Uart::PL011Uart(uintptr_t registersBase)
    : registers(*reinterpret_cast<Registers*>(registersBase))
{
    Disable(registersBase);

    // Clear pending interrupts
    registers.ICR = 0x7FF;

    // Set integer & fractional part of baud rate
    // Baud = 115200, UARTCLK = 48 MHz (default for Pi 3)
    // Divider = UARTCLK / (16 * Baud) = 48,000,000 / (16*115200) = 26.0416
    registers.IBRD = 26;
    registers.FBRD = 3;

    // Enable FIFO & 8 bit data transmission (1 stop bit, no parity)
    registers.LCRH = (1 << 4) | (3 << 5); // FIFO enable, 8 bit

    // Mask all interrupts
    registers.IMSC = (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
                    (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10);

    // Enable UART0, receive & transmit
    registers.CR = (1 << 0) | (1 << 8) | (1 << 9);
}

char PL011Uart::Getc()
{
    // Wait until data is ready in receiver FIFO
    while (registers.FR & 0x10) {}
    return static_cast<char>(registers.DR & 0xFF);
}

char PL011Uart::TryGetc()
{
    if (registers.FR & 0x10)
    {
        return (char)0; // No data available
    }
    return static_cast<char>(registers.DR & 0xFF);
}

void PL011Uart::PutcImpl(char c)
{
    // Wait until transmitter FIFO has space
    while (registers.FR & (1 << 5)) {}
    registers.DR = c;
}

void PL011Uart::PutsImpl(std::string_view str)
{
    for (char c : str)
    {
        PutcImpl(c);
    }
}

}
// namespace BootLib::Uart
