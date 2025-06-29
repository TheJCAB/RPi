#include "Timer.h"

#include "Mmio.h"
#include "Uart.h"

namespace Timer
{

uint64_t GetPerformanceFrequency()
{
    uint64_t freq = 0;
    asm volatile ("mrs %0, cntfrq_el0" : "=r"(freq));
    return freq;
}

uint64_t const PerformanceFrequency = GetPerformanceFrequency();

uint64_t GetPerformanceCounter()
{
    uint64_t counter = 0;
    asm volatile ("mrs %0, cntvct_el0" : "=r"(counter));
    return counter;
}

void Delay(uint64_t us)
{
    uint64_t const freq = GetPerformanceFrequency();
    uint64_t start = GetPerformanceCounter();
    uint64_t end = start + (us * freq / 1'000'000u);
    while (GetPerformanceCounter() < end) {
        //asm volatile ("wfe"); // This is bad unless we know there will be some event.
        asm volatile ("yield");
    }
}

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

struct Basic_IRQs
{
    uint32_t ARM_Timer      : 1; // @0
    uint32_t ARM_Mailbox    : 1; // @1
    uint32_t ARM_Doorbell_0 : 1; // @2
    uint32_t ARM_Doorbell_1 : 1; // @3
    uint32_t GPU_0_Halted   : 1; // @4
    uint32_t GPU_1_Halted   : 1; // @5
    uint32_t Access_Error_1 : 1; // @6
    uint32_t Access_Error_0 : 1; // @7
    uint32_t unused         : 24; // @8-31
};

constexpr uint32_t IRQ_basic_pending  = 0xB200u;
constexpr uint32_t IRQ_pending_1      = 0xB204u;
constexpr uint32_t IRQ_pending_2      = 0xB208u;
constexpr uint32_t FIQ_control        = 0xB20Cu;
constexpr uint32_t Enable_IRQs_1      = 0xB210u;
constexpr uint32_t Enable_IRQs_2      = 0xB214u;
constexpr uint32_t Enable_Basic_IRQs  = 0xB218u;
constexpr uint32_t Disable_IRQs_1     = 0xB21Cu;
constexpr uint32_t Disable_IRQs_2     = 0xB220u;
constexpr uint32_t Disable_Basic_IRQs = 0xB224u;

void SetPeriodicInterrupt(uint64_t us)
{
    Basic_IRQs armTimerBasicIrq{ .ARM_Timer = 1 };
    *(uint32_t volatile*)(MMIO_BASE + Enable_Basic_IRQs) = *(uint32_t*)&armTimerBasicIrq;

    // Calculate timer interval in counter ticks
    //uint64_t interval = us * GetPerformanceFrequency() / 1'000'000u;

    *(uint32_t volatile*)(MMIO_BASE + Timer_Load) = 0x400;
    time_ctrl_reg_t control
    {
        .unused              = 0,
        .Counter32Bit        = 1,
        .Prescale            = Clkdiv256,
        .unused1             = 0,
        .TimerIrqEnable      = 1,
        .unused2             = 0,
        .TimerEnable         = 1,
        .DbgKeepTimerRunning = 0,
        .CounterFreeRunning  = 0,
        .unused3             = 0,
        .FreeRunPreScaleDiv  = 0x3E, // Default pre-scaler value
        .reserved            = 0
    };
    *(uint32_t volatile*)(MMIO_BASE + Timer_Control) = *(uint32_t*)&control;
}

// This should be called from the IRQ handler for the virtual timer
void HandlePeriodicInterrupt()
{
    if (*(uint32_t volatile*)(MMIO_BASE + Timer_MaskedIRQ) & 0x1) {
        // Clear the interrupt
        *(uint32_t volatile*)(MMIO_BASE + Timer_Clear) = 0x1;
    } else {
        // If the interrupt is not set, we can ignore it
        return;
    }

    // Acknowledge the interrupt by resetting the timer interval
    *(uint32_t volatile*)(MMIO_BASE + Timer_Load) = 0x400;
    if (auto lockedStream = Uart::LockedStream(true)) {
        lockedStream.Puts("Periodic interrupt handled\n");
    }
}

}
// namespace Timer
