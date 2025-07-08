#include <stdint.h>
#include <stddef.h>

#include <BootLib/Cpu.h>
#include <BootLib/Mmio.h>
#include <BootLib/Gpio.h>
#include <BootLib/Uart.h>

extern "C" [[noreturn]] void KernelMain()
{
    BootLib::Mmio::Init();

    BootLib::Gpio gpio{ BootLib::Mmio::Base + BootLib::Gpio::RegistersOffset };

    gpio.SetUart0_14_15();

    BootLib::PL011Uart uart0{ BootLib::Mmio::Base + BootLib::PL011Uart::Uart0RegistersOffset };

    uart0.Init();

    uart0.Puts("\r\n\nHello from the kernel!\r\n\n");
    if (BootLib::Cpu::IsRpi4())
    {
        uart0.Puts("Running on a Raspberry Pi 4!\n");
    }
    else
    {
        uart0.Puts("Running on a Raspberry Pi 3!\n");
    }

    BootLib::Cpu::Halt();
}

