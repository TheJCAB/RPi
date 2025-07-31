#include "Uart.h"

#include "Timer.h"

#include "Cpu.h"
#include "Mmio.h"
#include "Gpio.h"

#include <atomic>
#include <mutex>

namespace Uart
{

constexpr uint32_t PL011_MMIO_OFFSET = 0x20'1000u;

struct PL011Registers
{
    Mmio::BaseRegisterProxy<uint32_t> DR   { PL011_MMIO_OFFSET + 0x00 };// 0x00 - Data Register
    Mmio::BaseRegisterProxy<uint32_t> RSR  { PL011_MMIO_OFFSET + 0x04 };// 0x04 - Receive Status Register
    Mmio::BaseRegisterProxy<uint32_t> FR   { PL011_MMIO_OFFSET + 0x18 };// 0x18 - Flag Register
    Mmio::BaseRegisterProxy<uint32_t> ILPR { PL011_MMIO_OFFSET + 0x20 };// 0x20 - IrDA Low-Power Counter Register
    Mmio::BaseRegisterProxy<uint32_t> IBRD { PL011_MMIO_OFFSET + 0x24 };// 0x24 - Integer Baud Rate Divisor
    Mmio::BaseRegisterProxy<uint32_t> FBRD { PL011_MMIO_OFFSET + 0x28 };// 0x28 - Fractional Baud Rate Divisor
    Mmio::BaseRegisterProxy<uint32_t> LCRH { PL011_MMIO_OFFSET + 0x2C };// 0x2C - Line Control Register
    Mmio::BaseRegisterProxy<uint32_t> CR   { PL011_MMIO_OFFSET + 0x30 };// 0x30 - Control Register
    Mmio::BaseRegisterProxy<uint32_t> IFLS { PL011_MMIO_OFFSET + 0x34 };// 0x34 - Interrupt FIFO Level Select Register
    Mmio::BaseRegisterProxy<uint32_t> IMSC { PL011_MMIO_OFFSET + 0x38 };// 0x38 - Interrupt Mask Set/Clear Register
    Mmio::BaseRegisterProxy<uint32_t> RIS  { PL011_MMIO_OFFSET + 0x3C };// 0x3C - Raw Interrupt Status Register
    Mmio::BaseRegisterProxy<uint32_t> MIS  { PL011_MMIO_OFFSET + 0x40 };// 0x40 - Masked Interrupt Status Register
    Mmio::BaseRegisterProxy<uint32_t> ICR  { PL011_MMIO_OFFSET + 0x44 };// 0x44 - Interrupt Clear Register
    Mmio::BaseRegisterProxy<uint32_t> DMACR{ PL011_MMIO_OFFSET + 0x48 };// 0x48 - DMA Control Register
};

static constexpr PL011Registers PL011{};

bool useMutex = false;
//std::atomic<bool> Mutex;
std::mutex Mutex;

void Init()
{
    Gpio::SetFunction(14, Gpio::Function::Alt0); // GPIO14 (TXD0)
    Gpio::SetFunction(15, Gpio::Function::Alt0); // GPIO15 (RXD0)
    Gpio::SetPullUpDown(14, Gpio::PullUpDown::None); // Disable pull-up/down for GPIO14
    Gpio::SetPullUpDown(15, Gpio::PullUpDown::None); // Disable pull-up/down for GPIO15

    // Clear pending interrupts
    PL011.ICR = 0x7FF;

    // Set integer & fractional part of baud rate
    // Baud = 115200, UARTCLK = 48 MHz (default for Pi 3)
    // Divider = UARTCLK / (16 * Baud) = 48,000,000 / (16*115200) = 26.0416
    PL011.IBRD = 26;
    PL011.FBRD = 3;

    // Enable FIFO & 8 bit data transmission (1 stop bit, no parity)
    PL011.LCRH = (1 << 4) | (3 << 5); // FIFO enable, 8 bit

    // Mask all interrupts
    PL011.IMSC = (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
                 (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10);

    // Enable UART0, receive & transmit
    PL011.CR = (1 << 0) | (1 << 8) | (1 << 9);
}

namespace Raw
{

void NoMmuPutc(char c)
{
    // Wait until transmitter FIFO has space
    if (Cpu::IsRpi4())
    {
        while (*(volatile unsigned int*)0x4'7E20'1018ull & (1 << 5)) {}
        *(volatile unsigned int*)0x4'7E20'1000ull = c;
    }
    else
    {
        while (*(volatile unsigned int*)0x3F20'1018ull & (1 << 5)) {}
        *(volatile unsigned int*)0x3F20'1000ull = c;
    }
}

void NoMmuPuts(char const* str)
{
    while (*str)
    {
        NoMmuPutc(*str++);
    }
}


char Getc()
{
    // Wait until data is ready in receiver FIFO
    while (PL011.FR & 0x10) {}
    return static_cast<char>(PL011.DR & 0xFF);
}

char TryGetc()
{
    if (PL011.FR & 0x10)
    {
        return (char)0; // No data available
    }
    return static_cast<char>(PL011.DR & 0xFF);
}

void Putc(char c)
{
    // Wait until transmitter FIFO has space
    while (PL011.FR & (1 << 5)) {}
    PL011.DR = c;
}

void Puts(char const* str)
{
    while (*str)
    {
        Raw::Putc(*str++);
    }
}

void PutHex(std::integral auto value)
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

void PutHex(void const volatile* value)
{
    PutHex(reinterpret_cast<uintptr_t>(value));
}

void PutBin(std::integral auto value)
{
    char const* binDigits = "01";
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

void PutDec(std::integral auto value)
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

template void PutHex(uint64_t value);
template void PutHex(uint32_t value);
template void PutHex(uint16_t value);
template void PutHex(uint8_t  value);
template void PutHex(bool     value);

template void PutBin(uint64_t value);
template void PutBin(uint32_t value);
template void PutBin(uint16_t value);
template void PutBin(uint8_t  value);
template void PutBin(bool     value);

template void PutDec(uint64_t value);
template void PutDec(uint32_t value);
template void PutDec(uint16_t value);
template void PutDec(uint8_t  value);
template void PutDec(bool     value);

} // namespace Raw

LockedStream::LockedStream(bool tryOnly)
{
    if (useMutex)
    {
        wasEnabled = Cpu::DisableInterrupts();
        if (tryOnly)
        {
            locked = Mutex.try_lock();
            if (!locked) Cpu::RestoreInterrupts(wasEnabled);
        }
        else
        {
            // Lock the mutex
            Mutex.lock();
            locked = true;
        }
    }
}

LockedStream::~LockedStream()
{
    if (locked && useMutex)
    {
        Mutex.unlock();
        Cpu::RestoreInterrupts(wasEnabled);
    }
}

void LockedStream::Unlock()
{
    if (locked && useMutex)
    {
        Mutex.unlock();
        Cpu::RestoreInterrupts(wasEnabled);
    }
    locked = false;
}

void LockedStream::Putc(char c)
{
    if (locked) Raw::Putc(c);
}

char LockedStream::Getc()
{
    if (!locked) return 0;
    return Raw::Getc();
}

char LockedStream::TryGetc()
{
    if (!locked) return 0;
    return Raw::TryGetc();
}

void LockedStream::Puts(char const* str)
{
    if (locked) Raw::Puts(str);
}

void LockedStream::PutHex(std::integral auto value)
{
    if (locked) Raw::PutHex(value);
}

void LockedStream::PutHex(void const volatile* value)
{
    if (locked) Raw::PutHex(value);
}

void LockedStream::PutBin(std::integral auto value)
{
    if (locked) Raw::PutBin(value);
}

void LockedStream::PutDec(std::integral auto value)
{
    if (locked) Raw::PutDec(value);
}

template void LockedStream::PutHex(uint64_t value);
template void LockedStream::PutHex(uint32_t value);
template void LockedStream::PutHex(uint16_t value);
template void LockedStream::PutHex(uint8_t  value);
template void LockedStream::PutHex(bool     value);

template void LockedStream::PutBin(uint64_t value);
template void LockedStream::PutBin(uint32_t value);
template void LockedStream::PutBin(uint16_t value);
template void LockedStream::PutBin(uint8_t  value);
template void LockedStream::PutBin(bool     value);

template void LockedStream::PutDec(uint64_t value);
template void LockedStream::PutDec(uint32_t value);
template void LockedStream::PutDec(uint16_t value);
template void LockedStream::PutDec(uint8_t  value);
template void LockedStream::PutDec(bool     value);


void Putc(char c)
{
    LockedStream{}.Putc(c);
}

char Getc()
{
    return LockedStream{}.Getc();
}

char TryGetc()
{
    return LockedStream{}.TryGetc();
}

void Puts(char const* str)
{
    LockedStream{}.Puts(str);
}

void PutHex(std::integral auto value)
{
    LockedStream{}.PutHex(value);
}

void PutHex(void const volatile* value)
{
    LockedStream{}.PutHex(value);
}

void PutBin(std::integral auto value)
{
    LockedStream{}.PutBin(value);
}

void PutDec(std::integral auto value)
{
    LockedStream{}.PutDec(value);
}

template void PutHex(uint64_t value);
template void PutHex(uint32_t value);
template void PutHex(uint16_t value);
template void PutHex(uint8_t  value);
template void PutHex(bool     value);

template void PutBin(uint64_t value);
template void PutBin(uint32_t value);
template void PutBin(uint16_t value);
template void PutBin(uint8_t  value);
template void PutBin(bool     value);

template void PutDec(uint64_t value);
template void PutDec(uint32_t value);
template void PutDec(uint16_t value);
template void PutDec(uint8_t  value);
template void PutDec(bool     value);


}
// namespace Uart

extern "C" void RawPutc(char c)
{
    Uart::Raw::Putc(c);
}
