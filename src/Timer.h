#pragma once

#include <stdint.h>

namespace Timer
{

void SetPeriodicVirtualTimerInterrupt(uint32_t us);
void HandleArmVirtualTimerInterrupt();

}
// namespace Timer
