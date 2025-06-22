#include "Uart.h"

#include "Mmio.h"

namespace Uart
{

#define AUX_BASE        (MMIO_BASE + 0x215000)

#define AUX_ENABLES     ((volatile unsigned int*)(AUX_BASE + 0x04))
#define AUX_MU_IO_REG   ((volatile unsigned int*)(AUX_BASE + 0x40))
#define AUX_MU_IER_REG  ((volatile unsigned int*)(AUX_BASE + 0x44))
#define AUX_MU_IIR_REG  ((volatile unsigned int*)(AUX_BASE + 0x48))
#define AUX_MU_LCR_REG  ((volatile unsigned int*)(AUX_BASE + 0x4C))
#define AUX_MU_MCR_REG  ((volatile unsigned int*)(AUX_BASE + 0x50))
#define AUX_MU_LSR_REG  ((volatile unsigned int*)(AUX_BASE + 0x54))
#define AUX_MU_CNTL_REG ((volatile unsigned int*)(AUX_BASE + 0x60))
#define AUX_MU_BAUD_REG ((volatile unsigned int*)(AUX_BASE + 0x68))

#define GPFSEL1         ((volatile unsigned int*)(MMIO_BASE + 0x200004))
#define GPPUD           ((volatile unsigned int*)(MMIO_BASE + 0x200094))
#define GPPUDCLK0       ((volatile unsigned int*)(MMIO_BASE + 0x200098))

void Init()
{
    // Disable pull-ups/downs
    *GPPUD = 0;
    for (volatile int i = 0; i < 1500; i = i + 1) {}
    *GPPUDCLK0 = (1 << 14) | (1 << 15);
    for (volatile int i = 0; i < 1500; i = i + 1) {}
    *GPPUDCLK0 = 0;

    // Configure GPIO14 & 15 to ALT5
    unsigned int r = *GPFSEL1;
    r &= ~((7 << 12) | (7 << 15)); // clear bits for GPIO14, GPIO15
    r |= 2 << 12 | 2 << 15;        // set ALT5
    *GPFSEL1 = r;

    // Enable Mini UART
    *AUX_ENABLES |= 1;

    // Disable TX/RX
    *AUX_MU_CNTL_REG = 0;

    // Disable interrupts
    *AUX_MU_IER_REG = 0;

    // Enable 8-bit mode
    *AUX_MU_LCR_REG = 3;

    // RTS line high
    *AUX_MU_MCR_REG = 0;

    // Clear FIFO
    *AUX_MU_IIR_REG = 0xC6;

    // Set baud rate to 115200 (assuming 250 MHz system clock)
    *AUX_MU_BAUD_REG = (400'000'000 / (8 * 115'200)) - 1;

    // Enable TX/RX
    *AUX_MU_CNTL_REG = 3;
}

void Putc(char c)
{
    // Wait until transmitter FIFO has space
    while (!(*AUX_MU_LSR_REG & (1 << 5)))
    {
        // Bit 5 == Transmitter FIFO can accept data
    }

    *AUX_MU_IO_REG = c;
}

char Getc()
{
    // Wait until data is ready in receiver FIFO
    while (!(*AUX_MU_LSR_REG & 0x01))
    {
        // Bit 0 == Data ready
    }
    return static_cast<char>(*AUX_MU_IO_REG & 0xFF);
}

char TryGetc()
{
    if (!(*AUX_MU_LSR_REG & 0x01))
    {
        return (char)0; // No data available
    }
    return static_cast<char>(*AUX_MU_IO_REG & 0xFF);
}

void Puts(char const* str)
{
    while (*str)
    {
        Putc(*str++);
    }
}

void PutHex(auto value)
{
    const char* hexDigits = "0123456789ABCDEF";
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

}
// namespace Uart