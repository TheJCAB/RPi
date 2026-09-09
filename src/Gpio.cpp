#include "Gpio.h"

#include "Cpu.h"
#include "Mmio.h"

namespace Gpio
{

constexpr uintptr_t GpioMmioOffset = 0x200000u;

struct BaseGpio
{
    Mmio::BaseRegisterArrayProxy<uint32_t, 5, 1, GpioMmioOffset + 0x00> GPFSEL;
    Mmio::BaseRegisterArrayProxy<uint32_t, 2, 1, GpioMmioOffset + 0x1C> GPSET ;
    Mmio::BaseRegisterArrayProxy<uint32_t, 2, 1, GpioMmioOffset + 0x28> GPCLR ;
    Mmio::BaseRegisterArrayProxy<uint32_t, 2, 1, GpioMmioOffset + 0x64> GPHEN ;
};

struct Rpi3Gpio
{
    Mmio::BaseRegisterProxy     <uint32_t,       GpioMmioOffset + 0x94> GPPUD   ;
    Mmio::BaseRegisterArrayProxy<uint32_t, 2, 1, GpioMmioOffset + 0x98> GPPUDCLK;
};

struct Rpi4Gpio
{
    Mmio::BaseRegisterArrayProxy<uint32_t, 4, 1, GpioMmioOffset + 0xE4> GPIO_PUP_PDN_CNTRL_REG;
};

constexpr Gpio::BaseGpio BaseRegisters;
constexpr Gpio::Rpi3Gpio Rpi3Registers;
constexpr Gpio::Rpi4Gpio Rpi4Registers;

void Init()
{
}

void SetFunction(uint32_t pin, Function func)
{
    auto const shift = (pin % 10) * 3;
    auto&& selreg = BaseRegisters.GPFSEL[pin / 10];

    selreg = [shift, func](uint32_t& reg){
        reg &= ~(0b111 << shift);
        reg |= static_cast<uint32_t>(func) << shift;
    };
}

void SetHighDetectEnable(uint32_t pin, bool enable)
{
    auto const shift = (pin % 32);
    auto&& henreg = BaseRegisters.GPHEN[pin / 32];

    henreg = [shift, enable](uint32_t& reg){
        if (enable)
        {
            reg |= (1u << shift);
        }
        else
        {
            reg &= ~(1u << shift);
        }
    };
}

void SetPullUpDown(uint32_t pin, PullUpDown pud)
{
    if (BootLib::Cpu::IsRpi4())
    {
        auto const shift = (pin % 16) * 2;
        auto&& cntrlreg = Rpi4Registers.GPIO_PUP_PDN_CNTRL_REG[pin / 16];
        cntrlreg = [shift, pud](uint32_t& reg){
            reg &= ~(0b11 << shift);
            reg |= static_cast<uint32_t>(pud) << shift;
        };
    }
    else
    {
        auto&& pudreg = Rpi3Registers.GPPUD;

        auto const shift = pin % 32;
        auto&& clkreg = Rpi3Registers.GPPUDCLK[pin / 32];

        pudreg = static_cast<uint32_t>(pud);
        Cpu::Delay(150us); // Wait for 150 cycles (we use 150us)
        clkreg = static_cast<uint32_t>(pin) << shift;
        Cpu::Delay(150us); // Wait for 150 cycles (we use 150us)
        if (static_cast<uint32_t>(pud) != 0)
        {
            pudreg = 0;
        }
        clkreg = 0;
    }
}

}
// namespace Gpio
