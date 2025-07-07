#pragma once

#include <stdint.h>

namespace Timer
{

void SetPeriodicVirtualTimerInterrupt(uint64_t us);
void HandleArmVirtualTimerInterrupt();

}
// namespace Timer
