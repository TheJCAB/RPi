#include <BootLib/Mmio.h>

#include <BootLib/Cpu.h>

namespace BootLib::Mmio
{

// We handle the MMIO base dynamically so we can adjust it for the version of Raspberry Pi.
uintptr_t Base;

void Init()
{
    // Figure out the MMIO base address.
    if (Cpu::IsRpi4())
    {
        // Note that for the Raspberry Pi 4 we use the high-peripherals address.
        // This is configured in the config.txt file.
        // TODO: Figure out a way to detect that setting automatically.
        Base = 0x4'7E00'0000u;
    }
    else
    {
        Base = 0x3F00'0000u;
    }
}

}
// namespace BootLib::Mmio
