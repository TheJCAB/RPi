#include <BootLib/Gpio.h>

#include <BootLib/Cpu.h>
#include <BootLib/Mmio.h>

namespace BootLib
{

constexpr uintptr_t GpioMmioOffset = 0x200000u;

union Gpio::GpioRegisters
{
    RegisterArray<uint32_t, 0x00, 5> GPFSEL;
    RegisterArray<uint32_t, 0x1C, 2> GPSET ;
    RegisterArray<uint32_t, 0x28, 2> GPCLR ;

    Register     <uint32_t, 0x94   > Pi3_GPPUD   ;
    RegisterArray<uint32_t, 0x98, 2> Pi3_GPPUDCLK;

    RegisterArray<uint32_t, 0xE4, 4> Pi4_GPIO_PUP_PDN_CNTRL_REG;
};

void Gpio::SetFunction(uint32_t pin, Function func)
{
    auto const shift = (pin % 10) * 3;
    auto&& selreg = Registers.GPFSEL[pin / 10];

    selreg = [shift, func](uint32_t& reg){
        reg &= ~(0b111 << shift);
        reg |= static_cast<uint32_t>(func) << shift;
    };
}

void Gpio::SetPull(uint32_t pin, Pull pull)
{
    if (Cpu::IsRpi4())
    {
        auto const shift = (pin % 16) * 2;
        auto&& cntrlreg = Registers.Pi4_GPIO_PUP_PDN_CNTRL_REG[pin / 16];
        cntrlreg = [shift, pull](uint32_t& reg){
            reg &= ~(0b11 << shift);
            reg |= static_cast<uint32_t>(pull) << shift;
        };
    }
    else
    {
        auto&& pudreg = Registers.Pi3_GPPUD;

        auto const shift = pin % 32;
        auto&& clkreg = Registers.Pi3_GPPUDCLK[pin / 32];

        pudreg = static_cast<uint32_t>(pull);
        Cpu::DelayInMicroseconds(150); // Wait for 150 cycles (we use 150us)
        clkreg = static_cast<uint32_t>(pin) << shift;
        Cpu::DelayInMicroseconds(150); // Wait for 150 cycles (we use 150us)
        if (static_cast<uint32_t>(pull) != 0)
        {
            pudreg = 0;
        }
        clkreg = 0;
    }
}

void Gpio::SetUart0_14_15()
{
    SetFunction(14, Function::Alt0);
    SetFunction(15, Function::Alt0);
    SetPull(14, Pull::None);
    SetPull(15, Pull::None);
}

}
// namespace BootLib
