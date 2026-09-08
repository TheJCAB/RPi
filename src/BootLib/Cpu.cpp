#include <BootLib/Cpu.h>

namespace BootLib::Cpu
{

bool IsQemu()
{
    return reinterpret_cast<uintptr_t>(&IsQemu) >= 0x4000'0000u;
}

bool IsRpi4()
{
    if (IsQemu()) return false;

    static bool checked = false;
    static bool isRpi4 = false;

    if (!checked)
    {
        uint64_t midr = 0;
        asm volatile ("mrs %0, midr_el1" : "=r"(midr));

        // Extract implementer (bits 31-24) and part number (bits 15-4)
        uint32_t implementer = (midr >> 24) & 0xFF;
        uint32_t part_number = (midr >> 4) & 0xFFF;
        
        // BCM2711 (Rpi4) has Cortex-A72 cores (part number 0xD08)
        // BCM2837 (Rpi3) has Cortex-A53 cores (part number 0xD03)
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

PerformanceTime GetPerformanceCounter()
{
    uint64_t counter = 0;
    asm volatile ("mrs %0, cntvct_el0" : "=r"(counter));
    return static_cast<PerformanceTime>(counter);
}

void DelayUntilPerformanceTime(PerformanceTime time)
{
    while (GetPerformanceCounter() < time)
    {
        //asm volatile ("wfe"); // This is bad unless we know there will be some event.
        asm volatile ("yield");
    }
}

PerformanceTimeDiff GetPerformanceTicksForUs(uint64_t us) { return static_cast<PerformanceTimeDiff>(us * GetPerformanceFrequency() / 1'000'000u); }
PerformanceTimeDiff GetPerformanceTicksForMs(uint64_t ms) { return static_cast<PerformanceTimeDiff>(ms * GetPerformanceFrequency() /     1'000u); }

PerformanceTimeDiff GetPerformanceTicksForUs(int64_t us) { return static_cast<PerformanceTimeDiff>(us * static_cast<int64_t>(GetPerformanceFrequency()) / 1'000'000); }
PerformanceTimeDiff GetPerformanceTicksForMs(int64_t ms) { return static_cast<PerformanceTimeDiff>(ms * static_cast<int64_t>(GetPerformanceFrequency()) /     1'000); }

int64_t GetUsForPerformanceTicks(PerformanceTimeDiff timeDiff) { return static_cast<int64_t>(timeDiff) * 1'000'000 / static_cast<int64_t>(GetPerformanceFrequency()); }
int64_t GetMsForPerformanceTicks(PerformanceTimeDiff timeDiff) { return static_cast<int64_t>(timeDiff) *     1'000 / static_cast<int64_t>(GetPerformanceFrequency()); }

void DelayInMicroseconds(uint64_t us) { DelayUntilPerformanceTime(GetPerformanceCounter() + GetPerformanceTicksForUs(us)); }
void DelayInMilliseconds(uint64_t ms) { DelayUntilPerformanceTime(GetPerformanceCounter() + GetPerformanceTicksForMs(ms)); }

[[noreturn]] void Halt()
{
    // Halt the CPU by entering an infinite loop
    while (true)
    {
        asm volatile("wfe"); // Wait for event (low power state)
    }
}

}
// namespace BootLib::Cpu

namespace std { inline namespace ABI {
    void __libcpp_verbose_abort(char const* fmt, ...) { BootLib::Cpu::Halt(); }
}}
