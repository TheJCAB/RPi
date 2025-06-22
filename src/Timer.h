#pragma once

#include <stdint.h>

namespace Timer
{

extern uint64_t const PerformanceFrequency;

uint64_t GetPerformanceFrequency();
uint64_t GetPerformanceCounter();
void Delay(uint64_t us);

}
// namespace Timer
