#pragma once

#include <stdint.h>
#include <stddef.h>

namespace Gpio
{

enum class PullUpDown : uint8_t
{
    None     = 0b00,
    PullDown = 0b01,
    PullUp   = 0b10,
    Reserved = 0b11,
};

enum class Function : uint8_t
{
    Input       = 0b000,
    Output      = 0b001,
    Alt0        = 0b100,
    Alt1        = 0b101,
    Alt2        = 0b110,
    Alt3        = 0b111,
    Alt4        = 0b011,
    Alt5        = 0b010,
};

void InitRpi3();
void InitRpi4();

void SetFunction  (uint32_t pin, Function func);
void SetPullUpDown(uint32_t pin, PullUpDown pud);

}
// namespace Gpio
