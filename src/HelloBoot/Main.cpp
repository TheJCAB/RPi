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

    Puts(uart0, "\r\n\nHello from the kernel!\r\n\n");
    if (BootLib::Cpu::IsRpi4())
    {
        Puts(uart0, "Running on a Raspberry Pi 4!\n");
    }
    else
    {
        Puts(uart0, "Running on a Raspberry Pi 3!\n");
    }

    BootLib::Cpu::Halt();
}

