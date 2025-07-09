#include <BootLib/Mmio.h>

#include <BootLib/Cpu.h>

namespace BootLib::Mmio
{

uintptr_t GetPeripheralsPhysicalBase()
{
    // Figure out the MMIO base address.
    if (Cpu::IsRpi4())
    {
        // Note that for the Raspberry Pi 4 we use the high-peripherals address.
        // This is configured in the config.txt file.
        // TODO: Figure out a way to detect that setting automatically.
        return 0x4'7E00'0000u;
    }
    else
    {
        return 0x3F00'0000u;
    }
}

}
// namespace BootLib::Mmio
