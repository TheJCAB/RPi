// Example demonstrating the scheduled timer functionality
// This file shows how to use the new Timer scheduling API

#include "Timer.h"
#include "Uart.h"

namespace TimerExample
{

// Global variable to store handle for cancellation example
static Timer::SparkHandle cancel_handle = Timer::INVALID_HANDLE;

// Example callback functions
void CallbackA(uintptr_t)
{
    Uart::LockedStream stream(true);
    stream.Puts("Timer A fired!\n");
}

void CallbackB(uintptr_t)
{
    Uart::LockedStream stream(true);
    stream.Puts("Timer B fired!\n");
}

void CallbackC(uintptr_t)
{
    Uart::LockedStream stream(true);
    stream.Puts("Timer C fired - scheduling another timer in 2 seconds!\n");

    // Schedule another timer from within a callback
    Timer::ScheduleSpark(Timer::MicrosecondsToTicks(2'000'000), { CallbackA }); // 2 seconds = 2,000,000 microseconds
}

void AbsoluteTimeCallback(uintptr_t)
{
    Uart::LockedStream stream(true);
    stream.Puts("Absolute time timer fired at 3 seconds!\n");
}

void NeverCallCallback(uintptr_t)
{
    Uart::LockedStream stream(true);
    stream.Puts("This should never print - timer was canceled!\n");
}

void CancelTimerCallback(uintptr_t)
{
    Uart::LockedStream stream(true);
    if (Timer::CancelSpark(cancel_handle))
    {
        stream.Puts("Successfully canceled a timer!\n");
    }
    else
    {
        stream.Puts("Failed to cancel timer (maybe it already fired?)\n");
    }
}

void DemonstrateScheduledTimers()
{
    Uart::Puts("=== Scheduled Timer Demonstration ===\n");
    
    // Schedule some timers at different intervals
    Timer::SparkHandle handle_a = Timer::ScheduleSpark(Timer::MicrosecondsToTicks(1'000'000), { CallbackA });  // 1 second
    Timer::SparkHandle handle_b = Timer::ScheduleSpark(Timer::MicrosecondsToTicks(2'500'000), { CallbackB });  // 2.5 seconds
    Timer::SparkHandle handle_c = Timer::ScheduleSpark(Timer::MicrosecondsToTicks(5'000'000), { CallbackC });  // 5 seconds

    Uart::Puts("Scheduled 3 timers:\n");
    Uart::Puts("  Timer A: 1 second\n");
    Uart::Puts("  Timer B: 2.5 seconds\n");
    Uart::Puts("  Timer C: 5 seconds (will schedule another)\n");
    
    // Example of scheduling at absolute time
    auto current_time = Timer::GetCurrentTimeTicks();
    auto future_time = current_time + Timer::MicrosecondsToTicks(3'000'000); // 3 seconds from now
    Timer::SparkHandle handle_abs = Timer::ScheduleSparkAtTime(future_time, { AbsoluteTimeCallback });

    // Example of canceling a timer
    Uart::Puts("  Timer D: 0.5 seconds (will be cancelled)\n");
    cancel_handle = Timer::ScheduleSpark(Timer::MicrosecondsToTicks(500'000), { NeverCallCallback });

    // Cancel the timer after 100ms
    Timer::ScheduleSpark(Timer::MicrosecondsToTicks(100'000), { CancelTimerCallback });

    Uart::Puts("Timers set up. Watch for callbacks over the next 10 seconds...\n");
}

}
// namespace TimerExample
