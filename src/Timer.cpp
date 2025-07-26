#include "Timer.h"

#include "Cpu.h"
#include "Mmio.h"
#include "Uart.h"
#include "Interrupts.h"

#include "emb-stdio.h"

#include <atomic>

namespace Timer
{

// Structure to hold scheduled callback information
struct ScheduledTimer
{
    Cpu::PerformanceTime          trigger_time_ticks;  // Absolute time when callback should be triggered
    std::atomic<Scheduler::Spark> Spark;  // Function to call

    explicit operator bool() const { return Spark.load(std::memory_order_relaxed).Func != nullptr; }
};

// Maximum number of scheduled timers
constexpr uint32_t MAX_SCHEDULED_TIMERS = 64;

struct CoreTimerData
{
    // Array to store scheduled timers
    ScheduledTimer scheduled_timers[MAX_SCHEDULED_TIMERS];

    bool timerIsEnabled = false;
};

CoreTimerData CoreData[4];

Interrupts::Spark HandleArmVirtualTimerInterrupt();

// Helper function to find the next timer that should fire
uint32_t FindNextScheduledTimerTriggerTime()
{
    auto const coreId = Cpu::mpidr_el1->CoreId;
    auto& coreData = CoreData[coreId];

    uint32_t next_timer = INVALID_HANDLE;
    auto earliest_time = Cpu::GetFarFuturePerformanceTime();
    
    for (uint32_t i = 0; i < MAX_SCHEDULED_TIMERS; ++i)
    {
        auto& timer = coreData.scheduled_timers[i];
        if (timer && timer.trigger_time_ticks < earliest_time)
        {
            earliest_time = timer.trigger_time_ticks;
            next_timer = i;
        }
    }

    return next_timer;
}

// Helper function to set up the timer hardware for the next scheduled event
void SetupTimerForNext()
{
    auto const coreId = Cpu::mpidr_el1->CoreId;
    auto& coreData = CoreData[coreId];

    for (;;)
    {
        uint32_t next_time = FindNextScheduledTimerTriggerTime();
        if (next_time == INVALID_HANDLE)
        {
            // No timers active, disable the timer
            coreData.timerIsEnabled = false;
            Interrupts::DisableCoreVirtualTimerInterrupt();
            return;
        }

        auto& timer = coreData.scheduled_timers[next_time];

        auto current_time = Cpu::GetPerformanceCounter();

        // Calculate how many ticks until the next timer should fire
        int64_t ticks_until_fire = static_cast<int64_t>(timer.trigger_time_ticks - current_time);

        // Ensure we don't set a negative or zero timer value
        if (ticks_until_fire <= 0)
        {
            auto const spark = coreData.scheduled_timers[next_time].Spark.exchange({}, std::memory_order_acquire);
            Scheduler::AddSpark(spark);

            // TODO: The scheduler should use the timer to handle scheduling on its own
            Scheduler::YieldToSparks();
        }
        else
        {
            // Set the timer interval
            Cpu::cntv_tval_el0 = static_cast<uint64_t>(ticks_until_fire);

            // Enable timer interrupts if this is the first scheduled timer
            if (!coreData.timerIsEnabled)
            {
                Interrupts::EnableCoreVirtualTimerInterrupt(HandleArmVirtualTimerInterrupt);
            }

            return;
        }
    }
}

// Public API functions

SparkHandle ScheduleSpark(Cpu::PerformanceTimeDiff delay_us, Scheduler::Spark const& spark)
{
    auto trigger_time = GetCurrentTimeTicks() + delay_us;
    return ScheduleSparkAtTime(trigger_time, spark);
}

SparkHandle ScheduleSparkAtTime(Cpu::PerformanceTime absolute_time_ticks, Scheduler::Spark const& spark)
{
    if (spark.Func == nullptr)
    {
        return INVALID_HANDLE;
    }

    static uint32_t currentStart = 0;

    auto const coreId = Cpu::mpidr_el1->CoreId;
    auto& coreData = CoreData[coreId];

    // Find an empty slot
    for (uint32_t i = 0; i < MAX_SCHEDULED_TIMERS; ++i)
    {
        uint32_t handle = (i + currentStart) % MAX_SCHEDULED_TIMERS;
        auto& timer = coreData.scheduled_timers[handle];
        if (!timer)
        {
            timer.trigger_time_ticks = absolute_time_ticks;
            timer.Spark.store(spark, std::memory_order_relaxed);

            // Set up the timer for the next event (might be this one)
            SetupTimerForNext();

            ++currentStart;
            return handle;
        }
    }
    
    // No free slots available
    return INVALID_HANDLE;
}

bool CancelSpark(SparkHandle handle)
{
    if (handle >= MAX_SCHEDULED_TIMERS)
    {
        return false;
    }

    auto const coreId = Cpu::mpidr_el1->CoreId;
    auto& coreData = CoreData[coreId];

    if (coreData.scheduled_timers[handle])
    {
        coreData.scheduled_timers[handle].Spark.exchange({}, std::memory_order_release);

        // Recalculate the next timer
        SetupTimerForNext();
        
        return true;
    }
    
    return false;
}



/*
enum TIMER_PRESCALE : uint32_t
{
	Clkdiv1 = 0b00,										// 0
	Clkdiv16 = 0b01,									// 1
	Clkdiv256 = 0b10,									// 2
	Clkdiv_undefined = 0b11,							// 3 
};

struct time_ctrl_reg_t
{
    uint32_t       unused              : 1;	// @0
    uint32_t       Counter32Bit        : 1;	// @1 Counter32 bit (16bit if false)
    TIMER_PRESCALE Prescale            : 2;	// @2-3 Prescale  
    uint32_t       unused1             : 1;	// @4
    uint32_t       TimerIrqEnable      : 1;	// @5 Timer irq enable
    uint32_t       unused2             : 1;	// @6
    uint32_t       TimerEnable         : 1;	// @7 Timer enable
    uint32_t       DbgKeepTimerRunning : 1;	// @8 Keep timer running in debug
    uint32_t       CounterFreeRunning  : 1;	// @9 Timer is free running
    uint32_t       unused3             : 6;	// @10-15
    uint32_t       FreeRunPreScaleDiv  : 8;	// @16-23 Free running counter pre-scaler. Freq is sys_clk/(prescale+1)  (0x3E default)
    uint32_t       reserved            : 8;	// @24-31 reserved
};

constexpr uint32_t Timer_Load      = 0xB400u;
constexpr uint32_t Timer_Value     = 0xB404u;
constexpr uint32_t Timer_Control   = 0xB408u;
constexpr uint32_t Timer_Clear     = 0xB40Cu;
constexpr uint32_t Timer_RawIRQ    = 0xB410u;
constexpr uint32_t Timer_MaskedIRQ = 0xB414u;
constexpr uint32_t Timer_Reload    = 0xB418u;

// This stuff is RPi4
//    // Enable the interrupt in the interrupt controller (GIC)
//    // Raspberry Pi 3B: Virtual timer IRQ is IRQ 27 (in GIC distributor)
//    constexpr uint32_t VIRTUAL_TIMER_IRQ = 27;
//    uintptr_t GIC_DIST_BASE = Mmio::Base + 0x41000; // GIC Distributor base address
//    uintptr_t GIC_DIST_ENABLE_SET = GIC_DIST_BASE + 0x100;
//
//    // Enable IRQ 27 (virtual timer)
//    volatile uint32_t* irq_enable_reg = reinterpret_cast<volatile uint32_t*>(GIC_DIST_ENABLE_SET + (VIRTUAL_TIMER_IRQ / 32) * 4);
//    *irq_enable_reg = (1u << (VIRTUAL_TIMER_IRQ % 32));
//    // This is platform-specific and may require additional setup outside this function.
//
//    // Use the GIC CPU registers to enable the virtual timer interrupt to go to the current core
//    uintptr_t GIC_CPU_BASE = Mmio::Base + 0x42000; // GIC CPU interface base address
//    uintptr_t GIC_CPU_CTRL = GIC_CPU_BASE + 0x00;
//    volatile uint32_t* cpu_ctrl = reinterpret_cast<volatile uint32_t*>(GIC_CPU_CTRL);
//    *cpu_ctrl |= 1; // Enable the GIC CPU interface

*/

// Enhanced interrupt handler that supports both scheduled and periodic timers
Interrupts::Spark HandleArmVirtualTimerInterrupt()
{
    auto const coreId = Cpu::mpidr_el1->CoreId;
    auto& coreData = CoreData[coreId];

    auto current_time = Cpu::GetPerformanceCounter();

    // Check for scheduled timers that should fire
    for (size_t i = 0; i < MAX_SCHEDULED_TIMERS; ++i)
    {
        auto& timer = coreData.scheduled_timers[i];
        if (timer && timer.trigger_time_ticks <= current_time)
        {
            // This timer should fire
            auto const spark = coreData.scheduled_timers[i].Spark.exchange({}, std::memory_order_acquire);

            // Call the callback (do this after marking inactive to avoid issues if callback reschedules)
            Scheduler::AddSpark(spark);

            // TODO: The scheduler should use the timer to handle scheduling on its own
            //Uart::Raw::Putc('$');
            Scheduler::YieldToSparks();
        }
    }
    
    // Set up the timer for the next scheduled event
    //Uart::Raw::Putc('^');
    SetupTimerForNext();

    return {};
}

}
// namespace Timer
