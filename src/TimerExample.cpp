// Example demonstrating the scheduled timer functionality
// This file shows how to use the new Timer scheduling API

#include "Timer.h"
#include "Interrupts.h"
#include "Uart.h"

#include <fmt/format.h>

namespace TimerExample
{

// Global variable to store handle for cancellation example
static Timer::SparkHandle cancel_handle = Timer::INVALID_HANDLE;

// Example callback functions
Interrupts::Spark CallbackA(uintptr_t)
{
    Uart::LockedStream stream(true);
    stream.Puts("Timer A fired!\n");
    return {};
}

Interrupts::Spark CallbackB(uintptr_t)
{
    Uart::LockedStream stream(true);
    stream.Puts("Timer B fired!\n");
    return {};
}

Interrupts::Spark CallbackC(uintptr_t)
{
    Uart::LockedStream stream(true);
    stream.Puts("Timer C fired - scheduling another timer in 2 seconds!\n");

    // Schedule another timer from within a callback
    Timer::ScheduleSpark(2s, { .Func = CallbackA }); // 2 seconds = 2,000,000 microseconds

    return {};
}

Interrupts::Spark AbsoluteTimeCallback(uintptr_t)
{
    Uart::LockedStream stream(true);
    stream.Puts("Absolute time timer fired at 3 seconds!\n");
    return {};
}

Interrupts::Spark NeverCallCallback(uintptr_t)
{
    Uart::LockedStream stream(true);
    stream.Puts("This should never print - timer was canceled!\n");
    return {};
}

Interrupts::Spark CancelTimerCallback(uintptr_t)
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
    return {};
}

Interrupts::Spark DemonstrateScheduledTimers()
{
    fmt::println("=== Scheduled Timer Demonstration ===");
    
    // Schedule some timers at different intervals
    Timer::SparkHandle handle_a = Timer::ScheduleSpark(1'000ms, { .Func = CallbackA });  // 1 second
    Timer::SparkHandle handle_b = Timer::ScheduleSpark(2'500ms, { .Func = CallbackB });  // 2.5 seconds
    Timer::SparkHandle handle_c = Timer::ScheduleSpark(5'000ms, { .Func = CallbackC });  // 5 seconds

    fmt::println("Scheduled 3 timers:");
    fmt::println("  Timer A: 1 second");
    fmt::println("  Timer B: 2.5 seconds");
    fmt::println("  Timer C: 5 seconds (will schedule another)");
    
    // Example of scheduling at absolute time
    auto current_time = Cpu::GetPerformanceCounter();
    auto future_time = current_time + Cpu::ToTicks(3s); // 3 seconds from now
    Timer::SparkHandle handle_abs = Timer::ScheduleSparkAtTime(future_time, { .Func = AbsoluteTimeCallback });

    // Example of canceling a timer
    fmt::println("  Timer D: 0.5 seconds (will be cancelled)");
    cancel_handle = Timer::ScheduleSpark(Cpu::ToTicks(500ms), { .Func = NeverCallCallback });

    // Cancel the timer after 100ms
    Timer::ScheduleSpark(Cpu::ToTicks(100ms), { .Func = CancelTimerCallback });

    fmt::println("Timers set up. Watch for callbacks over the next 10 seconds...");
    return {};
}

}
// namespace TimerExample
