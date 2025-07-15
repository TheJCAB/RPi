# Scheduled Timer Implementation for ARM64 Bare Metal OS

## Overview
This implementation provides a sophisticated scheduled timer system for ARM64 bare metal OS development, built on top of the ARM64 virtual timer (`cntv_tval_el0`). The system supports both one-shot scheduled callbacks and maintains backward compatibility with the original periodic timer functionality.

## Features

### Core Functionality
- **Scheduled Callbacks**: Schedule callbacks to execute after a specific delay or at an absolute time
- **Multiple Timers**: Support for up to 16 concurrent scheduled timers
- **Handle-based Management**: Each scheduled timer returns a handle that can be used to cancel it
- **High Resolution**: Uses the ARM64 performance counter for microsecond precision
- **Backward Compatibility**: Existing periodic timer functionality remains unchanged

### Key Functions

#### `SparkHandle ScheduleSpark(uint64_t delay_us, ScheduledCallback callback)`
Schedules a callback to execute after a specified delay in microseconds.
- **Parameters**: 
  - `delay_us`: Delay in microseconds before the callback should fire
  - `callback`: Function pointer to call when the timer expires
- **Returns**: Handle that can be used to cancel the timer, or `INVALID_HANDLE` on failure

#### `SparkHandle ScheduleSparkAtTime(uint64_t absolute_time_ticks, ScheduledCallback callback)`
Schedules a callback to execute at a specific absolute time.
- **Parameters**:
  - `absolute_time_ticks`: Absolute time in performance counter ticks when callback should fire
  - `callback`: Function pointer to call when the timer expires
- **Returns**: Handle that can be used to cancel the timer, or `INVALID_HANDLE` on failure

#### `bool CancelCallback(SparkHandle handle)`
Cancels a previously scheduled callback.
- **Parameters**: `handle`: Handle returned by `ScheduleSpark` or `ScheduleSparkAtTime`
- **Returns**: `true` if the timer was successfully canceled, `false` if not found or already fired

#### Utility Functions
- `uint64_t GetCurrentTimeTicks()`: Returns current time in performance counter ticks
- `uint64_t MicrosecondsToTicks(uint64_t us)`: Converts microseconds to performance counter ticks

## Implementation Details

### Data Structures
The system uses a simple array-based approach with a fixed maximum of 16 concurrent timers:

```cpp
struct ScheduledTimer
{
    uint64_t trigger_time_ticks;  // Absolute time when callback should be triggered
    ScheduledCallback callback;   // Function to call
    SparkHandle handle;        // Unique handle for this timer
    bool active;                  // Whether this timer slot is active
};
```

### Timer Resolution Strategy
The interrupt handler uses an intelligent approach to minimize timer interrupts:

1. **Find Next Timer**: Scans all active scheduled timers to find the one that should fire soonest
2. **Calculate Interval**: Sets the hardware timer to fire exactly when the next timer should trigger
3. **Dynamic Recalculation**: After each interrupt, recalculates the next timer interval
4. **Fallback to Periodic**: If no scheduled timers are active, falls back to periodic mode (if enabled)

### Interrupt Handler Logic
```cpp
void HandleArmVirtualTimerInterrupt()
{
    uint64_t current_time = Cpu::GetPerformanceCounter();
    
    // Check for scheduled timers that should fire
    for (size_t i = 0; i < MAX_SCHEDULED_TIMERS; ++i)
    {
        if (scheduled_timers[i].active && 
            static_cast<int64_t>(scheduled_timers[i].trigger_time_ticks - current_time) <= 0)
        {
            // Fire the timer and mark as inactive (one-shot)
            ScheduledCallback callback = scheduled_timers[i].callback;
            scheduled_timers[i].active = false;
            
            if (callback != nullptr)
            {
                callback();
            }
        }
    }
    
    // Set up timer for next scheduled event
    SetupTimerForNext();
}
```

## Usage Examples

### Basic Scheduled Timer
```cpp
#include "Timer.h"

void MyCallback()
{
    Uart::Puts("Timer fired!\n");
}

void SetupTimer()
{
    // Schedule a callback to fire in 1 second (1,000,000 microseconds)
    Timer::SparkHandle handle = Timer::ScheduleSpark(1000000, MyCallback);
    
    if (handle != Timer::INVALID_HANDLE)
    {
        Uart::Puts("Timer scheduled successfully\n");
    }
}
```

### Absolute Time Scheduling
```cpp
void ScheduleAtAbsoluteTime()
{
    uint64_t current_time = Timer::GetCurrentTimeTicks();
    uint64_t future_time = current_time + Timer::MicrosecondsToTicks(5000000); // 5 seconds from now
    
    Timer::ScheduleSparkAtTime(future_time, []() {
        Uart::Puts("Absolute timer fired!\n");
    });
}
```

### Timer Cancellation
```cpp
void CancellationExample()
{
    // Schedule a timer
    Timer::SparkHandle handle = Timer::ScheduleSpark(10000000, []() {
        Uart::Puts("This will be canceled\n");
    });
    
    // Cancel it after 1 second
    Timer::ScheduleSpark(1000000, [handle]() {
        if (Timer::CancelCallback(handle))
        {
            Uart::Puts("Successfully canceled timer\n");
        }
    });
}
```

## Performance Characteristics

### Memory Usage
- **Fixed Size**: 16 timer slots × 32 bytes = 512 bytes maximum
- **No Dynamic Allocation**: All memory is statically allocated
- **Handle Management**: Simple counter-based handle generation

### Time Complexity
- **Scheduling**: O(n) where n = number of active timers (max 16)
- **Cancellation**: O(n) linear search through active timers
- **Interrupt Handling**: O(n) to find and fire all expired timers

### Precision
- **Resolution**: Limited by ARM64 performance counter frequency (typically MHz range)
- **Accuracy**: Hardware timer accuracy, typically sub-microsecond
- **Jitter**: Minimal jitter due to direct hardware timer usage

## Limitations

1. **Maximum Timers**: Limited to 16 concurrent scheduled timers
2. **Linear Search**: Timer management is O(n) but with small constant (max 16)
3. **One-Shot Only**: Scheduled timers are one-shot; for recurring timers, reschedule in the callback
4. **No Priority**: All timers have equal priority; execution order depends on callback registration order

## Thread Safety

⚠️ **Important**: This implementation is **not thread-safe**. It's designed for single-threaded bare metal environments. If used in a multi-threaded context, external synchronization is required around timer operations.

## Future Enhancements

Potential improvements for production use:
- **Priority Queue**: Use a heap or priority queue for O(log n) timer management
- **Recurring Timers**: Add support for automatically recurring scheduled timers
- **Timer Pools**: Dynamic allocation of timer slots for unlimited concurrent timers
- **Priority Support**: Add timer priorities for execution ordering
- **Statistics**: Add timing statistics and performance monitoring
