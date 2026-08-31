#pragma once

#include <stdint.h>
#include <stddef.h>

namespace BootLib::Uart
{

class PL011Uart
{
    union PL011Registers;

public:
    static constexpr uint32_t Uart0RegistersOffset = 0x20'1000u; // TODO: Move. This is RPi-specific

    PL011Uart(uintptr_t registersBase);

    void SetGpio(uint8_t tx, uint8_t rx);

    void Putc(char c);
    char Getc();
    char TryGetc();
    void Puts(char const* str);
    void PutHex(auto value);
    void PutBin(auto value);
    void PutDec(auto value);

private:
    PL011Registers& Registers;
};

inline char Getc   (PL011Uart* uart) { return uart ? uart->Getc()    : 0; }
inline char TryGetc(PL011Uart* uart) { return uart ? uart->TryGetc() : 0; }

inline void Putc   (PL011Uart* uart, char        c    ) { if (uart) uart->Putc  (c    ); }
inline void Puts   (PL011Uart* uart, char const* str  ) { if (uart) uart->Puts  (str  ); }
inline void PutHex (PL011Uart* uart, auto        value) { if (uart) uart->PutHex(value); }
inline void PutBin (PL011Uart* uart, auto        value) { if (uart) uart->PutBin(value); }
inline void PutDec (PL011Uart* uart, auto        value) { if (uart) uart->PutDec(value); }

}
// namespace BootLib::Uart

namespace BootLib
{

using Uart::PL011Uart;

}
// namespace BootLib
