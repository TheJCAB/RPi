#pragma once

#include <BootLib/StreamOut.h>

#include <stdint.h>
#include <stddef.h>

namespace BootLib::Uart
{

class MiniUart
{
    union Registers;

public:
    static constexpr uint32_t UartRegistersOffset = 0x21'5000u; // TODO: Move. This is RPi-specific

    static void Disable(uintptr_t registersBase);
};

class PL011Uart
{
    union Registers;

public:
    static constexpr uint32_t Uart0RegistersOffset = 0x20'1000u; // TODO: Move. This is RPi-specific

    static void Disable(uintptr_t registersBase);

    PL011Uart(uintptr_t registersBase);

    void SetGpio(uint8_t tx, uint8_t rx);

    void PutcImpl(char);
    void PutsImpl(std::string_view);

    char Getc();
    char TryGetc();

    friend void Puts  (PL011Uart const& out, std::string_view            str  ) { using namespace Stream; Puts  (out.Out(), str  ); };
    friend void PutHex(PL011Uart const& out, std::unsigned_integral auto value) { using namespace Stream; PutHex(out.Out(), value); };
    friend void PutBin(PL011Uart const& out, std::unsigned_integral auto value) { using namespace Stream; PutBin(out.Out(), value); };
    friend void PutDec(PL011Uart const& out, std::unsigned_integral auto value) { using namespace Stream; PutDec(out.Out(), value); };

    Stream::Out Out() const { return static_cast<Stream::Out>(*this); }

    operator Stream::Out() const { return {
        .context = reinterpret_cast<uintptr_t>(this),
        .putc = [](uintptr_t context, char             c){ reinterpret_cast<PL011Uart*>(context)->PutcImpl(c); },
        .puts = [](uintptr_t context, std::string_view s){ reinterpret_cast<PL011Uart*>(context)->PutsImpl(s); },
    }; }

private:
    Registers& registers;
};

inline char Getc   (PL011Uart* uart) { return uart ? uart->Getc()    : 0; }
inline char TryGetc(PL011Uart* uart) { return uart ? uart->TryGetc() : 0; }

inline void Puts   (PL011Uart* uart, std::string_view str  ) { if (uart) Puts  (*uart, str  ); }
inline void PutHex (PL011Uart* uart, auto             value) { if (uart) PutHex(*uart, value); }
inline void PutBin (PL011Uart* uart, auto             value) { if (uart) PutBin(*uart, value); }
inline void PutDec (PL011Uart* uart, auto             value) { if (uart) PutDec(*uart, value); }

}
// namespace BootLib::Uart

namespace BootLib
{

using Uart::PL011Uart;

}
// namespace BootLib
