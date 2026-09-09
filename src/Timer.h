#pragma once

#include "Cpu.h"
#include "Scheduler.h"

#include <stdint.h>

namespace Timer
{

// Handle for scheduled callbacks (can be used to cancel)
using SparkHandle = uint32_t;
constexpr SparkHandle INVALID_HANDLE = UINT32_MAX;

// Schedule a callback to be called after a specific delay in microseconds
SparkHandle ScheduleSpark(Cpu::PerformanceTimeDiff delay_ticks, Scheduler::Spark const&);

// Schedule a callback to be called at a specific absolute time (in performance counter ticks)
SparkHandle ScheduleSparkAtTime(Cpu::PerformanceTime absolute_time_ticks, Scheduler::Spark const&);

// Cancel a scheduled callback
bool CancelSpark(SparkHandle);

inline SparkHandle ScheduleSpark(std::chrono::microseconds delay, Scheduler::Spark const& spark) {  return ScheduleSpark(Cpu::ToTicks(delay), spark); }

}
// namespace Timer
