#pragma once

#include <stdint.h>
#include <stddef.h>

namespace BootLib
{

class Gpio
{
    union GpioRegisters;

public:
    enum class Pull : uint8_t
    {
        None     = 0b00,
        Down     = 0b01,
        Up       = 0b10,
        Reserved = 0b11,
    };

    enum class Function : uint8_t
    {
        Input  = 0b000,
        Output = 0b001,
        Alt0   = 0b100,
        Alt1   = 0b101,
        Alt2   = 0b110,
        Alt3   = 0b111,
        Alt4   = 0b011,
        Alt5   = 0b010,
    };

    static constexpr uint32_t RegistersOffset = 0x20'0000u;

    Gpio(uintptr_t registersBase) : Registers(*reinterpret_cast<GpioRegisters*>(registersBase)) {}

    void SetFunction(uint32_t pin, Function);
    void SetPull    (uint32_t pin, Pull);

    void SetUart0_14_15();

private:
    GpioRegisters& Registers;
};

}
// namespace BootLib::Gpio
