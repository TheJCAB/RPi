#include <BootLib/Mmio.h>

#include <BootLib/Cpu.h>

namespace BootLib::Mmio
{

uintptr_t GetPeripheralsPhysicalBase()
{
    // Figure out the MMIO base address.
    if (Cpu::IsRpi4())
    {
        // Note that for the Raspberry Pi 4 we wish to use the high-peripherals address.
        // This is configured in the config.txt file.
        // TODO: Figure out a way to detect that setting automatically (from the devicetree? Or reading the config file ourselves?
        return Rpi4Base;
    }
    else
    {
        return Rpi3Base;
    }
}

}
// namespace BootLib::Mmio
