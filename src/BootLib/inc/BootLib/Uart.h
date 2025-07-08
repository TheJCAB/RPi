#pragma once

#include <stdint.h>
#include <stddef.h>

namespace BootLib
{

class PL011Uart
{
    union PL011Registers;

public:
    static constexpr uint32_t Uart0RegistersOffset = 0x201000u;

    PL011Uart(uintptr_t registersBase) : Registers(*reinterpret_cast<PL011Registers*>(registersBase)) {}

    void SetGpio(uint8_t tx, uint8_t rx);

    void Init();

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

}
// namespace BootLib::Uart
