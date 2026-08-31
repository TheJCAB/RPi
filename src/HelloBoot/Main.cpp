#include <stdint.h>
#include <stddef.h>

#include <BootLib/Cpu.h>
#include <BootLib/Mmio.h>
#include <BootLib/Gpio.h>
#include <BootLib/Uart.h>

extern "C" [[noreturn]] void KernelMain()
{
    auto const peripheralsBase = BootLib::Mmio::GetPeripheralsPhysicalBase();

    BootLib::Gpio gpio{ peripheralsBase + BootLib::Gpio::RegistersOffset };

    gpio.SetUart0_14_15();

    BootLib::PL011Uart uart0{ peripheralsBase + BootLib::PL011Uart::Uart0RegistersOffset };

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

