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

    asm volatile("msr daifclr, #2"); // Clear IRQ mask in DAIF

    *reinterpret_cast<volatile uint32_t*>(0x8000'0040u) = 0x08; // Enable the virtual timer interrupt

// This stuff is RPi4
//    // Enable the interrupt in the interrupt controller (GIC)
//    // Raspberry Pi 3B: Virtual timer IRQ is IRQ 27 (in GIC distributor)
//    constexpr uint32_t VIRTUAL_TIMER_IRQ = 27;
//    uintptr_t GIC_DIST_BASE = MMIO_BASE + 0x41000; // GIC Distributor base address
//    uintptr_t GIC_DIST_ENABLE_SET = GIC_DIST_BASE + 0x100;
//
//    // Enable IRQ 27 (virtual timer)
//    volatile uint32_t* irq_enable_reg = reinterpret_cast<volatile uint32_t*>(GIC_DIST_ENABLE_SET + (VIRTUAL_TIMER_IRQ / 32) * 4);
//    *irq_enable_reg = (1u << (VIRTUAL_TIMER_IRQ % 32));
//    // This is platform-specific and may require additional setup outside this function.
//
//    // Use the GIC CPU registers to enable the virtual timer interrupt to go to the current core
//    uintptr_t GIC_CPU_BASE = MMIO_BASE + 0x42000; // GIC CPU interface base address
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
