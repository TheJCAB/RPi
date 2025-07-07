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
*/


// Calculate timer interval in counter ticks
uint64_t interval;

void SetPeriodicInterrupt(uint64_t us)
{
    interval = us * GetPerformanceFrequency() / 1'000'000u;

    // Set the timer interval
    asm volatile ("msr cntv_tval_el0, %0" :: "r"(interval));

    // Enable the timer and unmask interrupt
    uint64_t ctl = 1; // Enable = 1, IMASK = 0, ISTATUS = don't care
    asm volatile ("msr cntv_ctl_el0, %0" :: "r"(ctl));

    *reinterpret_cast<volatile uint32_t*>(0x8000'0040u) = 0x08; // Enable the virtual timer interrupt for core 3

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
}

// This should be called from the IRQ handler for the virtual timer
void HandlePeriodicInterrupt()
{
    // Acknowledge the interrupt by resetting the timer interval
    //uint64_t interval;
    //asm volatile ("mrs %0, cntv_tval_el0" : "=r"(interval));
    asm volatile ("msr cntv_tval_el0, %0" :: "r"(interval));

    if (auto lockedStream = Uart::LockedStream(true)) {
        lockedStream.Puts("Periodic interrupt handled\n");
    }
}

}
// namespace Timer
