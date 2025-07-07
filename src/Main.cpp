#include <stdint.h>
#include <stddef.h>

//#include <span>
//#include <array>
//#include <format>

#include <atomic>

#include "Cpu.h"
#include "Mmio.h"
#include "Uart.h"
#include "Mailbox.h"
#include "Framebuffer.h"
#include "Exception.h"
#include "Processor.h"
#include "Mmu.h"
#include "Timer.h"
#include "Run.h"
#include "UsbDevices.h"
#include "PCIe.h"

#include "emb-stdio.h"

extern "C"
{

extern uint64_t _bss_start;
extern uint64_t _bss_end;

extern uint64_t _data_start;
extern uint64_t _data_end;

extern void (*_init_array_start[])();
extern void (*_init_array_end[])();

void _start();
void el2_to_el1_return();

void InitCore()
{
    uint64_t el = 0;
    asm volatile ("mrs %0, CurrentEL" : "=r"(el));

    if (((el >> 2) & 0b11) == 2)
    {
        Exception::InitEL2();
        el2_to_el1_return();
    }

    Mmu::Init();

    asm volatile("msr daifclr,#3"); // Clear the IRQ and FIQ mask bits to enable them
}

volatile bool Core1Ready = false;
volatile bool Core2Ready = false;
volatile bool Core3Ready = false;

void Core1()
{
    Uart::Raw::NoMmuPuts("Core 1 starting\n");

    InitCore();

    Uart::Puts("Core 1 says hello\n");
    Core1Ready = true; // Signal that core 1 is ready

    asm volatile ("dmb ish"); // Release barrier
    asm volatile ("sev");

    while (true)
    {
        asm volatile ("wfe" ::: "memory"); // Wait for event
    }
}

void Core2()
{
    InitCore();

    Uart::Puts("Core 2 says hello\n");
    Core2Ready = true; // Signal that core 2 is ready

    asm volatile ("dmb ish"); // Release barrier
    asm volatile ("sev");

    while (true)
    {
        //{
        //    Uart::LockedStream stream;
        //    stream.Puts("Core ");
        //    stream.PutDec(2u);
        //    stream.Puts(" is running\n");
        //}
        //Timer::Delay(1000);
        asm volatile ("wfe" ::: "memory"); // Wait for event
    }
}

void Core3()
{
    InitCore();

    Uart::Puts("Core 3 says hello\n");
    Core3Ready = true; // Signal that core 3 is ready

    asm volatile ("dmb ish"); // Release barrier
    asm volatile ("sev");

    while (true)
    {
        asm volatile ("wfe" ::: "memory"); // Wait for event
    }
}

inline uint32_t AtomicAdd(uint32_t volatile& value, uint32_t increment)
{
    uint32_t old_value;
    uint32_t new_value;
    uint32_t status;
    asm volatile (
        "1:     ldxr %w0, [%3]\n"        // Load exclusive
        "       add %w1, %w0, %w4\n"     // Add increment
        "       stxr %w2, %w1, [%3]\n"   // Store exclusive  
        "       cbnz %w2, 1b\n"          // Retry if store failed
        : "=&r"(old_value), "=&r"(new_value), "=&r"(status)
        : "r"(&value), "r"(increment)
        : "memory"
    );
    return old_value;
}

void Core0()
{
    // Clear the BSS.
    for (auto p = &_bss_start; p < &_bss_end; ++p)
    {
        *p = 0; // Clear BSS
    }

    bool const isRpi4 = Cpu::IsRpi4();
    if (isRpi4)
    {
        Mmio::Base    = 0x4'7E00'0000u; // RPi4 MMIO base address
        Mmio::QA7Base = 0x4'C000'0000u; // RPi4 QA7 base address
    }

    Mailbox::Send(0, 0x80); // UART 1 and USB enabled

    Uart::Init();

    Uart::Puts("\r\n\nHello!\n");

    if (isRpi4)
    {
        Uart::Puts("Detected Raspberry Pi 4\n");
    }
    else
    {
        Uart::Puts("Detected Raspberry Pi 3\n");
    }

    Uart::Puts("Init array start: ");
    Uart::PutHex(reinterpret_cast<uintptr_t>(_init_array_start));
    Uart::Puts("\n");
    Uart::Puts("Init array end: ");
    Uart::PutHex(reinterpret_cast<uintptr_t>(_init_array_end));
    Uart::Puts("\n");

    for (auto ctor = _init_array_start; ctor < _init_array_end; ++ctor) {
        if (ctor != nullptr)
        {
            (*ctor)();
        }
    }

    uint64_t lr = 0;
    asm volatile ("mov %0, lr" : "=r"(lr));
    uint64_t fp = 0;
    asm volatile ("mov %0, fp" : "=r"(fp));
    uint64_t pc = 0;
    asm volatile ("adr %0, ." : "=r"(pc));
    Uart::Puts("LR: ");
    Uart::PutHex(lr);
    Uart::Puts("\nFR: ");
    Uart::PutHex(fp);
    Uart::Puts("\nPC: ");
    Uart::PutHex(pc);
    Uart::Puts("\n");

    Uart::Puts("Initial performance Frequency: ");
    Uart::PutDec(Timer::PerformanceFrequency);
    Uart::Puts("\n");
    Uart::Puts("Current performance Frequency: ");
    Uart::PutDec(Timer::GetPerformanceFrequency());
    Uart::Puts("\n");

    uint64_t el = 0;
    asm volatile ("mrs %0, CurrentEL" : "=r"(el));
    Uart::Puts("CurrentEL: ");
    Uart::PutDec((el >> 2) & 0b11);
    Uart::Puts("\n");

    uint64_t mmfr0 = 0;
    asm volatile ("mrs %0, id_aa64mmfr0_el1" : "=r" (mmfr0));
    Uart::Puts("ID_AA64MMFR0_EL1: ");
    Uart::PutHex(mmfr0);
    Uart::Puts("\n");

    if (((el >> 2) & 0b11) == 2)
    {
        Exception::InitEL2();

        el2_to_el1_return();

        Uart::Puts("Running EL1\n");

        asm volatile ("mrs %0, CurrentEL" : "=r"(el));
        Uart::Puts("CurrentEL: ");
        Uart::PutDec((el >> 2) & 0b11);
        Uart::Puts("\n");

        Uart::Puts("Performance Frequency: ");
        Uart::PutDec(Timer::GetPerformanceFrequency());
        Uart::Puts("\n");
    }

    Mmu::Init();

    Uart::Puts("MMU enabled\n");

    Uart::Puts("Performance Frequency: ");
    Uart::PutDec(Timer::GetPerformanceFrequency());
    Uart::Puts("\n");

    Exception::Init();

    uint64_t sctlr = 0;
    asm volatile ("mrs %0, sctlr_el1" : "=r"(sctlr));
    //sctlr |= (1 << 1); // Set A (Alignment check enable) bit
    sctlr |= (1 << 3); // Set SA (Stack Alignment Check Enable) bit
    asm volatile ("msr sctlr_el1, %0" :: "r"(sctlr));
    asm volatile ("isb"); // Ensure changes take effect

    uint64_t daif = 0;
    asm volatile ("mrs %0, daif" : "=r"(daif));
    daif &= ~(1 << 7); // Clear the SError (S) mask bit to enable synchronous exceptions
    asm volatile ("msr daif, %0" :: "r"(daif));

    uint64_t cpacr = 0;
    asm volatile ("mrs %0, cpacr_el1" : "=r"(cpacr));
    cpacr |= (3 << 20); // Set CPACR_EL1.FP
    asm volatile ("msr cpacr_el1, %0" :: "r"(cpacr));

    Uart::Puts("Waiting...\n");
    Timer::Delay(1000'000);
    
    //asm volatile ("svc #42"); // Trigger a software interrupt to test exception handling
    //asm volatile ("hvc #42"); // Trigger a software interrupt to test exception handling

    asm volatile("msr daifclr,#3"); // Clear the IRQ and FIQ mask bits to enable them

    Uart::useMutex = true;

    Uart::Puts("Spinning up the cores...\n");

    ((void* volatile*)(0xD8 + GpuMemBase))[1] = (void*)_start;
    asm volatile ("dmb ish;sev");
    while (!Core1Ready) asm volatile ("dmb ish;sev" ::: "memory");
    Uart::Puts("Core 1 is going\n");
    
    ((void* volatile*)(0xD8 + GpuMemBase))[2] = (void*)_start;
    asm volatile ("dmb ish;sev");
    while (!Core2Ready) asm volatile ("wfe;dmb ish;sev" ::: "memory");
    Uart::Puts("Core 2 is going\n");
    
    ((void* volatile*)(0xD8 + GpuMemBase))[3] = (void*)_start;
    asm volatile ("dmb ish;sev");
    while (!Core3Ready) asm volatile ("wfe;dmb ish;sev" ::: "memory");
    Uart::Puts("Core 3 is going\n");

    if (isRpi4)
    {
        PCIe::examples::demonstrate_enumeration();
    }
    else
    {
        //Timer::SetPeriodicInterrupt(1000'000); // Set a periodic interrupt every second
        //Timer::Delay(3'000'000);

        UsbInitialise();
        Timer::Delay(10'000);
        Uart::Init();

        Uart::Puts("Waiting...\n");
        Timer::Delay(1000'000);

        UsbCheckForChange();
        Uart::Init();

        /* Display the USB tree */
        printf2("\n");
        UsbShowTree(UsbGetRootHub(), 1, '+');
        printf2("\n");
    }

    Uart::Puts("Waiting...\n");
    Timer::Delay(1000'000);

    uint32_t const w = 1280;
    uint32_t const h =  720;

    Framebuffer::Init(w, h);

    // STL              Some via LLVM's libc++
    // Timers           Delay() and GetPerformanceCounter()
    // Remote boot for development :-)      Done!
    // UART input       Done
    // Exceptions       Done
    // VSync/flip       According to documentation, this is not doable from ARM, outside of the HW rendering.
    // Interrupts       Done
    // Multicore        Done
    // Storage
    // USB              Done
    // Networking?

    Run();
}

}
// extern "C"
