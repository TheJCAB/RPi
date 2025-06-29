#pragma once

#include <stdint.h>

namespace Timer
{

extern uint64_t const PerformanceFrequency;

uint64_t GetPerformanceFrequency();
uint64_t GetPerformanceCounter();

inline uint64_t GetPerformanceTicksForUs(uint64_t us)
{
    return (us * PerformanceFrequency / 1'000'000u);
}

void Delay(uint64_t us);

void SetPeriodicInterrupt(uint64_t us);
void HandlePeriodicInterrupt();

}
// namespace Timer
