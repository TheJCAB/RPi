#pragma once

#include "Scheduler.h"

#include <stdint.h>

namespace Timer
{

// Handle for scheduled callbacks (can be used to cancel)
using SparkHandle = uint32_t;
constexpr SparkHandle INVALID_HANDLE = UINT32_MAX;

// Schedule a callback to be called after a specific delay in microseconds
SparkHandle ScheduleSpark(uint64_t delay_us, Scheduler::Spark const&);

// Schedule a callback to be called at a specific absolute time (in performance counter ticks)
SparkHandle ScheduleSparkAtTime(uint64_t absolute_time_ticks, Scheduler::Spark const&);

// Cancel a scheduled callback
bool CancelSpark(SparkHandle);

// Get current time in performance counter ticks
uint64_t GetCurrentTimeTicks();

// Convert microseconds to performance counter ticks
uint64_t MicrosecondsToTicks(uint64_t us);

}
// namespace Timer
