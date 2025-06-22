#include "Timer.h"

namespace Timer
{

uint64_t GetPerformanceFrequency()
{
    uint64_t freq = 0;
    asm volatile ("mrs %0, cntfrq_el0" : "=r"(freq));
    return freq;
}

uint64_t const PerformanceFrequency = GetPerformanceFrequency();

uint64_t GetPerformanceCounter()
{
    uint64_t counter = 0;
    asm volatile ("mrs %0, cntvct_el0" : "=r"(counter));
    return counter;
}

void Delay(uint64_t us)
{
    uint64_t const freq = GetPerformanceFrequency();
    uint64_t start = GetPerformanceCounter();
    uint64_t end = start + (us * freq / 1'000'000u);
    while (GetPerformanceCounter() < end) {
        //asm volatile ("wfe"); // This is bad unless we know there will be some event.
        asm volatile ("yield");
    }
}

}
// namespace Timer
