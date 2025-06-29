#include "Uart.h"

#include "Mmio.h"

#include <atomic>

namespace Uart
{

#define PL011_BASE      (MMIO_BASE + 0x201000)

#define UART_DR         ((volatile unsigned int*)(PL011_BASE + 0x00))
#define UART_FR         ((volatile unsigned int*)(PL011_BASE + 0x18))
#define UART_IBRD       ((volatile unsigned int*)(PL011_BASE + 0x24))
#define UART_FBRD       ((volatile unsigned int*)(PL011_BASE + 0x28))
#define UART_LCRH       ((volatile unsigned int*)(PL011_BASE + 0x2C))
#define UART_CR         ((volatile unsigned int*)(PL011_BASE + 0x30))
#define UART_IMSC       ((volatile unsigned int*)(PL011_BASE + 0x38))
#define UART_ICR        ((volatile unsigned int*)(PL011_BASE + 0x44))

#define GPFSEL1         ((volatile unsigned int*)(MMIO_BASE + 0x200004))
#define GPPUD           ((volatile unsigned int*)(MMIO_BASE + 0x200094))
#define GPPUDCLK0       ((volatile unsigned int*)(MMIO_BASE + 0x200098))

bool useMutex = false;

std::atomic<bool> Mutex;

void Init()
{
    // Disable UART0
    *UART_CR = 0;

    // Setup GPIO14 and GPIO15 to ALT0 (UART0 TX/RX)
    unsigned int r = *GPFSEL1;
    r &= ~((7 << 12) | (7 << 15)); // clear bits for GPIO14, GPIO15
    r |= (4 << 12) | (4 << 15);    // set ALT0
    *GPFSEL1 = r;

    // Disable pull-up/down for pins 14 and 15
    *GPPUD = 0;
    for (volatile int i = 0; i < 1500; i = i + 1) {}
    *GPPUDCLK0 = (1 << 14) | (1 << 15);
    for (volatile int i = 0; i < 1500; i = i + 1) {}
    *GPPUDCLK0 = 0;

    // Clear pending interrupts
    *UART_ICR = 0x7FF;

    // Set integer & fractional part of baud rate
    // Baud = 115200, UARTCLK = 48 MHz (default for Pi 3)
    // Divider = UARTCLK / (16 * Baud) = 48,000,000 / (16*115200) = 26.0416
    *UART_IBRD = 26;
    *UART_FBRD = 3;

    // Enable FIFO & 8 bit data transmission (1 stop bit, no parity)
    *UART_LCRH = (1 << 4) | (3 << 5); // FIFO enable, 8 bit

    // Mask all interrupts
    *UART_IMSC = (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
                 (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10);

    // Enable UART0, receive & transmit
    *UART_CR = (1 << 0) | (1 << 8) | (1 << 9);
}

namespace Raw
{

char Getc()
{
    // Wait until data is ready in receiver FIFO
    while (*UART_FR & 0x10) {}
    return static_cast<char>(*UART_DR & 0xFF);
}

char TryGetc()
{
    if (*UART_FR & 0x10)
    {
        return (char)0; // No data available
    }
    return static_cast<char>(*UART_DR & 0xFF);
}

void Putc(char c)
{
    // Wait until transmitter FIFO has space
    while (*UART_FR & (1 << 5)) {}
    *UART_DR = c;
}

void Puts(char const* str)
{
    while (*str)
    {
        Raw::Putc(*str++);
    }
}

void PutHex(auto value)
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

void PutBin(auto value)
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

void PutDec(auto value)
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

} // namespace Raw



void Putc(char c)
{
    if (useMutex) while (Mutex.exchange(true)) {} // Spin until mutex is available
    Raw::Putc(c);
    if (useMutex) Mutex.store(false); // Release mutex
}

char Getc()
{
    if (useMutex) while (Mutex.exchange(true)) {} // Spin until mutex is available
    char c = Raw::Getc();
    if (useMutex) Mutex.store(false); // Release mutex
    return c;
}

char TryGetc()
{
    if (useMutex && Mutex.exchange(true)) return 0;
    char c = Raw::TryGetc();
    if (useMutex) Mutex.store(false); // Release mutex
    return c;
}

void Puts(char const* str)
{
    if (useMutex) while (Mutex.exchange(true)) {} // Spin until mutex is available
    Raw::Puts(str);
    if (useMutex) Mutex.store(false); // Release mutex
}

void PutHex(auto value)
{
    if (useMutex) while (Mutex.exchange(true)) {} // Spin until mutex is available
    Raw::PutHex(value);
    if (useMutex) Mutex.store(false); // Release mutex
}

void PutBin(auto value)
{
    if (useMutex) while (Mutex.exchange(true)) {} // Spin until mutex is available
    Raw::PutBin(value);
    if (useMutex) Mutex.store(false); // Release mutex
}

void PutDec(auto value)
{
    if (useMutex) while (Mutex.exchange(true)) {} // Spin until mutex is available
    Raw::PutDec(value);
    if (useMutex) Mutex.store(false); // Release mutex
}

template void PutHex(uint64_t value);
template void PutHex(uint32_t value);
template void PutHex(uint16_t value);
template void PutHex(uint8_t value);

template void PutBin(uint64_t value);
template void PutBin(uint32_t value);
template void PutBin(uint16_t value);
template void PutBin(uint8_t value);

template void PutDec(uint64_t value);
template void PutDec(uint32_t value);
template void PutDec(uint16_t value);
template void PutDec(uint8_t value);

LockedStream::LockedStream(bool tryOnly)
{
    if (useMutex)
    {
        // Spin until mutex is available
        while (Mutex.exchange(true))
        {
            if (tryOnly) return;
        }
    }
    locked = true;
}

LockedStream::~LockedStream()
{
    if (locked && useMutex)
    {
        Mutex.store(false); // Release mutex
    }
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

void LockedStream::PutHex(auto value)
{
    if (locked) Raw::PutHex(value);
}

void LockedStream::PutBin(auto value)
{
    if (locked) Raw::PutBin(value);
}

void LockedStream::PutDec(auto value)
{
    if (locked) Raw::PutDec(value);
}

template void LockedStream::PutHex(uint64_t value);
template void LockedStream::PutHex(uint32_t value);
template void LockedStream::PutHex(uint16_t value);
template void LockedStream::PutHex(uint8_t value);

template void LockedStream::PutBin(uint64_t value);
template void LockedStream::PutBin(uint32_t value);
template void LockedStream::PutBin(uint16_t value);
template void LockedStream::PutBin(uint8_t value);

template void LockedStream::PutDec(uint64_t value);
template void LockedStream::PutDec(uint32_t value);
template void LockedStream::PutDec(uint16_t value);
template void LockedStream::PutDec(uint8_t value);

}
// namespace Uart