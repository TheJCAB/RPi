#include <BootLib/Cpu.h>

namespace BootLib::Cpu
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

uint64_t GetPerformanceFrequency()
{
    uint64_t freq = 0;
    asm volatile ("mrs %0, cntfrq_el0" : "=r"(freq));
    return freq;
}

uint64_t GetPerformanceCounter()
{
    uint64_t counter = 0;
    asm volatile ("mrs %0, cntvct_el0" : "=r"(counter));
    return counter;
}

void DelayInMicroseconds(uint64_t us)
{
    uint64_t start = GetPerformanceCounter();
    uint64_t end = start + (us * GetPerformanceFrequency() / 1'000'000u);
    while (GetPerformanceCounter() < end) {
        //asm volatile ("wfe"); // This is bad unless we know there will be some event.
        asm volatile ("yield");
    }
}

}
// namespace BootLib::Cpu
