#include "Cpu.h"

namespace Cpu
{

bool IsRpi4()
{
    static bool checked = false;
    static bool isRpi4 = false;

    if (!checked)
    {
        uint64_t midr = 0;
        asm volatile ("mrs %0, midr_el1" : "=r"(midr));

        // Extract implementer (bits 31-24) and part number (bits 15-4)
        uint32_t implementer = (midr >> 24) & 0xFF;
        uint32_t part_number = (midr >> 4) & 0xFFF;
        
        // BCM2711 (RPi4) has Cortex-A72 cores (part number 0xD08)
        // BCM2837 (RPi3) has Cortex-A53 cores (part number 0xD03)
        isRpi4  = (implementer == 0x41 && part_number == 0xD08);
        checked = true;
    }

    return isRpi4;
}

}
// namespace Cpu
