#pragma once

#include "Scheduler.h"

#include <stdint.h>

namespace Timer
{

// Handle for scheduled callbacks (can be used to cancel)
using SparkHandle = uint32_t;
constexpr SparkHandle INVALID_HANDLE = UINT32_MAX;

// Schedule a callback to be called after a specific delay in microseconds
SparkHandle ScheduleSpark(Cpu::PerformanceTimeDiff delay_us, Scheduler::Spark const&);

// Schedule a callback to be called at a specific absolute time (in performance counter ticks)
SparkHandle ScheduleSparkAtTime(Cpu::PerformanceTime absolute_time_ticks, Scheduler::Spark const&);

// Cancel a scheduled callback
bool CancelSpark(SparkHandle);

// Get current time in performance counter ticks
inline Cpu::PerformanceTime GetCurrentTimeTicks() { return Cpu::GetPerformanceCounter(); }

// Convert microseconds to performance counter ticks
inline Cpu::PerformanceTimeDiff MicrosecondsToTicks(uint64_t us) { return Cpu::GetPerformanceTicksForUs(us); }

}
// namespace Timer
