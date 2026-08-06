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
#include "SdCard.h"
#include "Exception.h"
#include "Interrupts.h"
#include "Processor.h"
#include "Mmu.h"
#include "Scheduler.h"
#include "Debugger.h"
#include "Syscall.h"
#include "Timer.h"
#include "Run.h"
#include "UsbDevices.h"
#include "PCIe.h"
#include "Usb.h"
#include "Async.h"

#include "TimerExample.h"

#include "emb-stdio.h"

void parse_dtb(void* dtb);

void InitGlobalHeap();

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
    // Enable the Floating point and SIMD unit for EL0 and EL1
    Cpu::cpacr_el1.modify([](auto& reg){ reg.FPEN = 3; });
    // Enable Stack alignment checks. For Hygiene.
    Cpu::sctlr_el1.modify([](auto& reg){ reg.SA = true; });
    Cpu::InstructionSynchronizationBarrier();

    el2_to_el1_return();
    Mmu::Init();
    Exception::Init();
    Interrupts::Init();
    Scheduler::Init();
}

volatile bool Core1Ready = false;
volatile bool Core2Ready = false;
volatile bool Core3Ready = false;

void Core1()
{
    {
        // Without MMU, we need to use physical addresses to access peripherals.
        BootLib::PL011Uart uart{ BootLib::Mmio::GetPeripheralsPhysicalBase() + BootLib::PL011Uart::Uart0RegistersOffset };
        uart.Puts("Core 1 starting\n");
    }

    InitCore();

    BootLib::PL011Uart uart{ Mmio::Base + BootLib::PL011Uart::Uart0RegistersOffset };
    uart.Puts("Core 1 says hello\n");

#if 0
    uint8_t* buffer = Mmu::AllocatePages<uint8_t>(2);
    uart.Puts("Buffer is ");
    uart.PutHex(reinterpret_cast<uintptr_t>(buffer));
    uart.Puts("\n");

    uart.Puts("Core 1: Writing to Buffer[0]\n");
    buffer[0] = 0x42; // Write something to the buffer
    uart.Puts("Core 1: Writing to Buffer[4095]\n");
    buffer[4095] = 0x56; // Write something to the buffer
    uart.Puts("Core 1: Writing to Buffer[8191]\n");
    buffer[8191] = 0xBA; // Write something to the buffer

    Processor::InvalidateDataCache(buffer, 8192); // Invalidate the data cache for the buffer

    if (buffer[0] != 0x42)
    {
        uart.Puts("Core 1: Buffer[0] is not 0x42\n");
    }
    else
    {
        uart.Puts("Core 1: Buffer[0] is 0x42\n");
    }

    if (buffer[4095] != 0x56)
    {
        uart.Puts("Core 1: Buffer[4095] is not 0x56\n");
    }
    else
    {
        uart.Puts("Core 1: Buffer[4095] is 0x56\n");
    }

    if (buffer[8191] != 0xBA)
    {
        uart.Puts("Core 1: Buffer[8191] is not 0xBA\n");
    }
    else
    {
        uart.Puts("Core 1: Buffer[8191] is 0xBA\n");
    }

    uart.Puts("Core 1: PhysBuffer[0] = ");
    uart.PutHex(reinterpret_cast<uint8_t*>(0x7'2000'0000)[0]);
    uart.Puts("\n");
    uart.Puts("Core 1: PhysBuffer[4095] = ");
    uart.PutHex(reinterpret_cast<uint8_t*>(0x7'2000'0000)[4095]);
    uart.Puts("\n");
    uart.Puts("Core 1: PhysBuffer[8191] = ");
    uart.PutHex(reinterpret_cast<uint8_t*>(0x7'2000'0000)[0x3FFF]);
    uart.Puts("\n");

#endif // 0

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
    {
        // Without MMU, we need to use physical addresses to access peripherals.
        BootLib::PL011Uart uart{ BootLib::Mmio::GetPeripheralsPhysicalBase() + BootLib::PL011Uart::Uart0RegistersOffset };
        uart.Puts("Core 2 starting\n");
    }

    InitCore();

    BootLib::PL011Uart uart{ Mmio::Base + BootLib::PL011Uart::Uart0RegistersOffset };
    uart.Puts("Core 2 says hello\n");
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
        //Cpu::DelayInMicroseconds(1000);
        asm volatile ("wfe" ::: "memory"); // Wait for event
    }
}

void Core3()
{
    {
        // Without MMU, we need to use physical addresses to access peripherals.
        BootLib::PL011Uart uart{ BootLib::Mmio::GetPeripheralsPhysicalBase() + BootLib::PL011Uart::Uart0RegistersOffset };
        uart.Puts("Core 3 starting\n");
    }

    InitCore();

    BootLib::PL011Uart uart{ Mmio::Base + BootLib::PL011Uart::Uart0RegistersOffset };
    uart.Puts("Core 3 says hello\n");
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

void Core0(void* dtb)
{
    {
        // Without MMU, we need to use physical addresses to access peripherals.
        // It is handy to have a UART for log-debugging.
        BootLib::PL011Uart uart{ BootLib::Mmio::GetPeripheralsPhysicalBase() + BootLib::PL011Uart::Uart0RegistersOffset };
        uart.Puts("Core 0 starting\n");

        // Clear the BSS soonest.
        for (auto p = &_bss_start; p < &_bss_end; ++p)
        {
            *p = 0; // Clear BSS
        }

        if (Cpu::IsRpi4())
        {
            uart.Puts("Detected Raspberry Pi 4\n");
        }
        else
        {
            uart.Puts("Detected Raspberry Pi 3\n");
        }

        uart.Puts("Performance Frequency: ");
        uart.PutDec(Cpu::GetPerformanceFrequency());
        uart.Puts("\n");

        // None of this seems to work. The exception vector doesn't get invoked.
        //Exception::InitEL2();
        //// Configure HCR_EL2 to route exceptions to EL2
        //asm volatile("mrs x0, hcr_el2");
        //asm volatile("orr x0, x0, #0x8");  // Set AMO bit to route SError to EL2
        //asm volatile("msr hcr_el2, x0");
        //asm volatile("isb");
        //Cpu::daifclr = 4; // Clear the SError mask bit to enable synchronous exceptions
        //asm volatile ("svc #0");
        //*(volatile int32_t*)(0x1234567890123456) = 0; // Clear the mailbox status register

        uart.Puts("\r\n\nHello!\n");

        uart.Puts("CurrentEL (should be 2): ");
        uart.PutDec(Cpu::CurrentEL->EL);
        uart.Puts("\n");

        // Enable the Floating point and SIMD unit for EL0 and EL1
        Cpu::cpacr_el1.modify([](auto& reg){ reg.FPEN = 3; });
        // Enable Stack alignment checks. For Hygiene.
        Cpu::sctlr_el1.modify([](auto& reg){ reg.SA = true; });
        Cpu::InstructionSynchronizationBarrier();

        el2_to_el1_return();

        uart.Puts("Lowered to EL1\n");

        uart.Puts("CurrentEL: ");
        uart.PutDec(Cpu::CurrentEL->EL);
        uart.Puts("\n");

        // Any further I/O operations will be done once the MMU is active.
        if (Cpu::IsRpi4())
        {
            Mmio::Base    = Mmio::Rpi4Base;
            Mmio::QA7Base = Mmio::Rpi4QA7Base;
        }

        // Initialize the MMU, and so all addresses will be virtual after this.
        Mmu::Init();
    }

    Uart::Puts("MMU enabled\n");

    // Call all global initializers.
    for (auto ctor = _init_array_start; ctor < _init_array_end; ++ctor) {
        if (ctor != nullptr)
        {
            (*ctor)();
        }
    }

    Uart::Puts("Performance Frequency from the global: ");
    Uart::PutDec(Cpu::PerformanceFrequency);
    Uart::Puts("\n");

    Uart::Puts("\n\n\n");

    Exception::Init();
    Interrupts::Init();

    InitGlobalHeap();

    parse_dtb(dtb);
    
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

    Uart::Puts("\n\n\n");

    Mailbox::TagMessage<Mailbox::Tag::GET_BOARD_MAC_ADDRESS, 2> macAddressTag{{ 0, 0 }};
    Mailbox::TagMessage<Mailbox::Tag::GET_VC_MEMORY, 2> armMemoryTag{{ 0, 0 }};
    Mailbox::TagMessage<Mailbox::Tag::GET_ARM_MEMORY, 2> vcMemoryTag{{ 0, 0 }};
    if (Mailbox::SendTags(armMemoryTag, vcMemoryTag, macAddressTag))
    {
        Uart::Puts("ARM Memory: ");
        Uart::PutHex(armMemoryTag.args[0]);
        Uart::Puts(" ");
        Uart::PutHex(armMemoryTag.args[1]);
        Uart::Puts("\n");
        Uart::Puts("VC Memory: ");
        Uart::PutHex(vcMemoryTag.args[0]);
        Uart::Puts(" ");
        Uart::PutHex(vcMemoryTag.args[1]);
        Uart::Puts("\n");
        Uart::Puts("MAC address: ");
        Uart::PutHex(macAddressTag.args[0]);
        Uart::Puts(" ");
        Uart::PutHex(macAddressTag.args[1]);
        Uart::Puts("\n");
    }
    else
    {
        Uart::Puts("Failed to get VC Memory info.\n");
    }

    Uart::Puts("\n\n\n");

    Mailbox::TagMessage<Mailbox::Tag::GET_CLOCK_RATE, 2> clockRateTag{{ 0, 0 }};
    Mailbox::TagMessage<Mailbox::Tag::GET_MEASURED_CLOCK_RATE, 2> measuredClockRateTag{{ 0, 0 }};
    Mailbox::TagMessage<Mailbox::Tag::GET_MAX_CLOCK_RATE, 2> maxClockRateTag{{ 0, 0 }};
    Mailbox::TagMessage<Mailbox::Tag::GET_POWER_STATE, 2> powerStateTag{{ 0, 0 }};

    // Get various clock rates
    uint32_t clockIds[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}; // Common clock IDs
    for (uint32_t clockId : clockIds)
    {
        clockRateTag.args[0] = clockId;
        clockRateTag.args[1] = 0;
        maxClockRateTag.args[0] = clockId;
        maxClockRateTag.args[1] = 0;
        measuredClockRateTag.args[0] = clockId;
        measuredClockRateTag.args[1] = 0;
        if (Mailbox::SendTags(clockRateTag, measuredClockRateTag, maxClockRateTag))
        {
            printf("Clock %2u rate: %10u Hz max: %10u Hz measured: %10u Hz\n", clockId, clockRateTag.args[1], maxClockRateTag.args[1], measuredClockRateTag.args[1]);
        }
        else
        {
            Uart::Puts("Clock ");
            Uart::PutDec(clockId);
            Uart::Puts(" not available.\n");
        }
    }

    Uart::Puts("\n\n\n");

    // Get various power states
    uint32_t powerIds[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}; // Common power domain IDs
    for (uint32_t powerId : powerIds)
    {
        powerStateTag.args[0] = powerId;
        powerStateTag.args[1] = 0;
        if (Mailbox::SendTags(powerStateTag))
        {
            printf("Power domain %2u state: %u\n", powerId, powerStateTag.args[1]);
        }
    }

    Uart::Puts("\n\n\n");

    Scheduler::Init();
    printf("Scheduler initialized. Main thread ThreadInfo: 0x%0X\n", reinterpret_cast<uintptr_t>(&Scheduler::GetCurrentThreadInfo()));

    auto const debuggerThread = Debugger::Init();

    Debugger::DebuggerThread = debuggerThread;

    if (Debugger::DebuggerThread == nullptr)
    {
        Uart::Puts("Debugger thread not initialized.\n");
    }
    else
    {
        Uart::Puts("Debugger thread initialized.\n");
    }

    Scheduler::AddSpark(Scheduler::MakeUserModeSpark([](uintptr_t){ Uart::Raw::Puts("Core 0 Spark running\n"); return Scheduler::Spark{}; }, 0));
    Uart::Puts("Core 0 Spark is scheduled\n");

    // A naked syscall.
    asm volatile ("svc #1");

    // Cooperative context switching via syscall...
    auto& mainThread = Scheduler::GetCurrentThreadInfo();

    auto& newThread = Scheduler::CreateThread([](uintptr_t mainThread){
        Uart::Puts("Core 0 New Thread running\n");
        Syscall::YieldToThread(*reinterpret_cast<Scheduler::ThreadInfo*>(mainThread));
        Uart::Puts("Core 0 New Thread finished\n");
        Syscall::YieldToThread(*reinterpret_cast<Scheduler::ThreadInfo*>(mainThread));
    }, reinterpret_cast<uintptr_t>(&mainThread));

    Syscall::YieldToThread(newThread);
    Uart::Puts("Core 0 Back to Main Thread\n");
    Syscall::YieldToThread(newThread);
    Uart::Puts("Core 0 Back to Main Thread again\n");

    Uart::Puts("\n\n\n");

    Async::task<void> asyncTask = []() -> Async::task<void> {
        for (int i = 0; i < 20; ++i)
        {
            Uart::Puts("Async task is running\n");
            co_await Async::DelayInMilliseconds(1000);
        }
        Uart::Puts("Async task is done\n");
        co_return;
    }();

    Uart::Puts("\n\n\n");

    Mailbox::Send(0, 0x80); // UART 1 and USB enabled?

    SdCard sdCard{ Mmio::Base + SdCard::RegistersOffset };
    if (sdCard.Init())
    {
        Uart::Puts("SD Card initialized successfully.\n");

        uint8_t buffer[512];
        if (sdCard.ReadBlock(0, buffer, 1))
        {
            Uart::Puts("Read block 0 successfully.\n");
            Uart::Puts("Data: ");
            for (size_t i = 0; i < sizeof(buffer) / 16; ++i)
            {
                for (size_t j = 0; j < 16; ++j)
                {
                    Uart::PutHex(buffer[i * 16 + j]);
                    Uart::Putc(' ');
                    if (j == 7)
                    {
                        Uart::Puts("- ");
                    }
                }
                Uart::Puts("\n");
            }
            Uart::Puts("\n");
        }
        else
        {
            Uart::Puts("Failed to read block 0.\n");
        }
        if (sdCard.ReadBlock(0x800, buffer, 1))
        {
            Uart::Puts("Read block 0x800 successfully.\n");
            Uart::Puts("Data: ");
            for (size_t i = 0; i < sizeof(buffer) / 16; ++i)
            {
                for (size_t j = 0; j < 16; ++j)
                {
                    Uart::PutHex(buffer[i * 16 + j]);
                    Uart::Putc(' ');
                    if (j == 7)
                    {
                        Uart::Puts("- ");
                    }
                }
                Uart::Puts("\n");
            }
            Uart::Puts("\n");
        }
        else
        {
            Uart::Puts("Failed to read block 0x800.\n");
        }
    }
    else
    {
        Uart::Puts("Failed to initialize SD Card.\n");
        Cpu::Halt();
    }

    Uart::Puts("\n\n\n");

    if (Cpu::IsRpi4())
    {
        PCIe::examples::demonstrate_enumeration();

        auto xhci = Usb::CreateXhciController();
        xhci->initialize();
        
    }
    else
    {
        auto usbInitTask = UsbInitialize();

        (void)WaitOnTask(std::move(usbInitTask));

        (void)WaitOnTask(UsbCheckForChange());

        /* Display the USB tree */
        printf("\n");
        UsbShowTree(UsbGetRootHub(), 1, '+');
        printf("\n");
    }

    uint32_t const w = 1280;
    uint32_t const h =  720;

    Framebuffer::Init(w, h);

    // STL              Some via LLVM's libc++
    // Timers           Delay() and GetPerformanceCounter()
    // Remote boot for development :-)      Done!
    // UART input       Done
    // Exceptions       Done
    // VSync/flip       According to documentation, Vsync is not doable from ARM, outside of the HW rendering. Flip works and doesn't appear to tear. Probably busy waits?
    // Interrupts       Done
    // Multicore        Done
    // Storage          SD Blocks
    // USB              Done Pi3
    // Networking?

    Run();
}

}
// extern "C"
