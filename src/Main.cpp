#include <BootLib/DeviceTree.h>

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
#include "UsbDriver.h"
#include "PCIe.h"
#include "Usb.h"
#include "Async.h"

#include "TimerExample.h"

#include "emb-stdio.h"

#include <stdint.h>
#include <stddef.h>

#include <atomic>
#include <bit>
#include <print>

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
    // But make sure to allow unaligned SIMD accesses.
    Cpu::sctlr_el1.modify([](auto& reg){ reg.SA = true; reg.A = false; });
    Cpu::InstructionSynchronizationBarrier();

    if (Cpu::CurrentEL->EL > 1)
    {
        el2_to_el1_return();
    }

    Mmu::EnableCachesAndMMU();
    Mmu::DumpMMUState();

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
        Puts(uart, "Core 1 starting\n");
    }

    InitCore();

    BootLib::PL011Uart uart{ Mmio::Base + BootLib::PL011Uart::Uart0RegistersOffset };
    Puts(uart, "Core 1 says hello\n");

#if 0
    uint8_t* buffer = Mmu::AllocatePages<uint8_t>(2);
    Puts(uart, "Buffer is ");
    PutHex(uart, reinterpret_cast<uintptr_t>(buffer));
    Puts(uart, "\n");

    Puts(uart, "Core 1: Writing to Buffer[0]\n");
    buffer[0] = 0x42; // Write something to the buffer
    Puts(uart, "Core 1: Writing to Buffer[4095]\n");
    buffer[4095] = 0x56; // Write something to the buffer
    Puts(uart, "Core 1: Writing to Buffer[8191]\n");
    buffer[8191] = 0xBA; // Write something to the buffer

    Processor::InvalidateDataCache(buffer, 8192); // Invalidate the data cache for the buffer

    if (buffer[0] != 0x42)
    {
        Puts(uart, "Core 1: Buffer[0] is not 0x42\n");
    }
    else
    {
        Puts(uart, "Core 1: Buffer[0] is 0x42\n");
    }

    if (buffer[4095] != 0x56)
    {
        Puts(uart, "Core 1: Buffer[4095] is not 0x56\n");
    }
    else
    {
        Puts(uart, "Core 1: Buffer[4095] is 0x56\n");
    }

    if (buffer[8191] != 0xBA)
    {
        Puts(uart, "Core 1: Buffer[8191] is not 0xBA\n");
    }
    else
    {
        Puts(uart, "Core 1: Buffer[8191] is 0xBA\n");
    }

    Puts(uart, "Core 1: PhysBuffer[0] = ");
    PutHex(uart, reinterpret_cast<uint8_t*>(0x7'2000'0000)[0]);
    Puts(uart, "\n");
    Puts(uart, "Core 1: PhysBuffer[4095] = ");
    PutHex(uart, reinterpret_cast<uint8_t*>(0x7'2000'0000)[4095]);
    Puts(uart, "\n");
    Puts(uart, "Core 1: PhysBuffer[8191] = ");
    PutHex(uart, reinterpret_cast<uint8_t*>(0x7'2000'0000)[0x3FFF]);
    Puts(uart, "\n");

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
        Puts(uart, "Core 2 starting\n");
    }

    InitCore();

    BootLib::PL011Uart uart{ Mmio::Base + BootLib::PL011Uart::Uart0RegistersOffset };
    Puts(uart, "Core 2 says hello\n");
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
        Puts(uart, "Core 3 starting\n");
    }

    InitCore();

    BootLib::PL011Uart uart{ Mmio::Base + BootLib::PL011Uart::Uart0RegistersOffset };
    Puts(uart, "Core 3 says hello\n");
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

void Core0(uintptr_t dtb)
{
    // Clear the BSS soonest.
    for (auto p = &_bss_start; p < &_bss_end; ++p)
    {
        *p = 0; // Clear BSS
    }

    bool const isQemu = reinterpret_cast<uintptr_t>(&Mmio::Base) >= 0x4000'0000u;

    {
        // Without MMU, we need to use physical addresses to access peripherals.
        // It is handy to have a UART for log-debugging.
        // TODO: Basic machine detection and less hardcoding would be grand here.
        BootLib::PL011Uart uart = [&]()
            {
                if (!isQemu)
                {
                    // Some Raspberry Pi.
                    return BootLib::PL011Uart{ BootLib::Mmio::GetPeripheralsPhysicalBase() + BootLib::PL011Uart::Uart0RegistersOffset };
                }
                else
                {
                    // QEMU's "virt" machine.
                    return BootLib::PL011Uart{ 0x900'0000u };
                }
            }();

        auto out = uart.Out();

        Puts(out, "Core ");
        PutDec(out, BootLib::Cpu::mpidr_el1->CoreId);
        Puts(out, " starting\n");

        Puts(out, "CurrentEL (may be 2): ");
        PutDec(out, Cpu::CurrentEL->EL);
        Puts(out, "\n");

        // Enable the Floating point and SIMD unit for EL0 and EL1
        Cpu::cpacr_el1.modify([](auto& reg){ reg.FPEN = 3; });
        // Enable Stack alignment checks. For Hygiene. But allow unaligned accesses.
        Cpu::sctlr_el1.modify([](auto& reg){ reg.SA = true; reg.A = false; });
        Cpu::InstructionSynchronizationBarrier();

        if (Cpu::CurrentEL->EL == 2)
        {
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

            el2_to_el1_return();

            Puts(out, "Lowered to EL1\n");

            Puts(out, "CurrentEL: ");
            PutDec(out, Cpu::CurrentEL->EL);
            Puts(out, "\n");
        }

        Puts(out, "cpacr_el1 = ");
        PutHex(out, std::bit_cast<uint64_t>(Cpu::cpacr_el1.get()));
        Puts(out, "\n");

        Puts(out, "sctlr_el1 = ");
        PutHex(out, std::bit_cast<uint64_t>(Cpu::sctlr_el1.get()));
        Puts(out, "\n");

        Uart::Init(&uart);
        Uart::Puts("This is a test\n");

        Init_EmbStdio([](char Ch, uintptr_t) { Uart::Puts({ &Ch, 1 }); });

        if (BootLib::Cpu::IsRpi4())
        {
            Puts(out, "Detected Raspberry Pi 4\n");
        }
        else
        {
            Puts(out, "Detected Raspberry Pi 3\n");
        }

        Puts(out, "Performance Frequency: ");
        PutDec(out, BootLib::Cpu::GetPerformanceFrequency());
        Puts(out, "\n");

        Puts(out, "\r\n\nHello!\n");

        Exception::Init();

        Puts(out, "DTB pointer: ");
        PutHex(out, dtb);
        Puts(out, "\n");

        BootLib::DeviceTree::ParseDeviceTree(dtb, out);

        printf("Device tree complete.\n");

        for (uintptr_t addr = 0xA00'0000u; addr < 0xA004000u; addr += 0x200u)
        {
            printf("virtio-mmio at 0x%08llX: 0x%08X 0x%08X 0x%08X 0x%08X\n",
                addr,
                reinterpret_cast<uint32_t const volatile*>(addr)[0],
                reinterpret_cast<uint32_t const volatile*>(addr)[1],
                reinterpret_cast<uint32_t const volatile*>(addr)[2],
                reinterpret_cast<uint32_t const volatile*>(addr)[3]
            );
        }

        uint64_t midr = 0;
        asm volatile ("mrs %0, midr_el1" : "=r"(midr));

        printf("midr_el1: 0x%016llX\n", midr);

        // Any further I/O operations will be done once the MMU is active.
        if (BootLib::Cpu::IsRpi4())
        {
            Mmio::Base    = Mmio::Rpi4Base;
            Mmio::QA7Base = Mmio::Rpi4QA7Base;
        }

        // Initialize the MMU, and so all addresses will be virtual after this.
        printf("Initializing page tables...\n");
        Mmu::InitPageTables();

        printf("Page tables initialized... Initializing MMU...\n");
        Mmu::EnableCachesAndMMU();

        if (BootLib::Cpu::IsRpi4())
        {
            // The MMIO address changes on Raspberry Pi 4, so we need to recreate the UART before it gets used again.
            uart.~PL011Uart();
            new(&uart) Uart::PL011Uart{ Mmio::Base + Uart::PL011Uart::Uart0RegistersOffset };
        }

        Mmu::DumpMMUState();
        Puts(out, "MMU enabled\n");
        
        Puts(out, "Initializing heap...\n");
        InitGlobalHeap();
        Puts(out, "Heap initialized.\n");
        
        // Call all global initializers.
        Puts(out, "Calling global initializers...\n");
        for (auto ctor = _init_array_start; ctor < _init_array_end; ++ctor) {
            if (ctor != nullptr)
            {
                (*ctor)();
            }
        }
        Puts(out, "Global initializers complete.\n");
    }

    // TODO: We should get addresses from the MM now.
    Uart::Init(new Uart::PL011Uart{ 
        isQemu ? 0x900'0000u
               : Mmio::Base + Uart::PL011Uart::Uart0RegistersOffset
    });

    Uart::Puts("Performance Frequency from the global: ");
    Uart::PutDec(Cpu::PerformanceFrequency);
    Uart::Puts("\n");

    Uart::Puts("\n\n\n");

//    Exception::Init();
    Interrupts::Init();

    //Uart::useMutex = true;

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

    Mailbox::TagMessage<Mailbox::Tag::GET_BOARD_MODEL, 1> modelTag{{ 0 }};
    Mailbox::TagMessage<Mailbox::Tag::GET_BOARD_REVISION, 1> revisionTag{{ 0 }};
    Mailbox::TagMessage<Mailbox::Tag::GET_BOARD_MAC_ADDRESS, 2> macAddressTag{{ 0, 0 }};
    Mailbox::TagMessage<Mailbox::Tag::GET_VC_MEMORY, 2> vcMemoryTag{{ 0, 0 }};
    Mailbox::TagMessage<Mailbox::Tag::GET_ARM_MEMORY, 2> armMemoryTag{{ 0, 0 }};
    if (Mailbox::SendTags(modelTag, revisionTag, armMemoryTag, vcMemoryTag, macAddressTag))
    {
        Uart::Puts("Board model: ");
        Uart::PutHex(modelTag.args[0]);
        Uart::Puts("\n");
        Uart::Puts("Board revision: ");
        Uart::PutHex(revisionTag.args[0]);
        Uart::Puts("\n");
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
            co_await Async::Delay(1000ms);
        }
        Uart::Puts("Async task is done\n");
        co_return;
    }();

    Uart::Puts("\n\n\n");
    
    Mailbox::Send(0, 0x80); // UART 1 and USB enabled?
    Uart::Puts("UART1 and USB enabled\n\n");

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
                    Uart::Puts(" ");
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
                    Uart::Puts(" ");
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

    std::shared_ptr<UsbDriver> usbDriver;

    if (BootLib::Cpu::IsRpi4())
    {
        //PCIe::examples::demonstrate_enumeration();
        PCIe::Bcm2711Driver pcie{};

        for (auto& device : pcie.devices_)
        {
            Uart::Puts("Device found: ");
            Uart::PutHex(device.Address.Bus);
            Uart::Puts(":");
            Uart::PutHex(device.Address.Device);
            Uart::Puts(".");
            Uart::PutHex(device.Address.Function);
            Uart::Puts("\n Vendor ID: ");
            Uart::PutHex(device.VendorId);
            Uart::Puts("\n Device ID: ");
            Uart::PutHex(device.DeviceId);
            Uart::Puts("\n Class Code: ");
            Uart::PutHex(device.ClassCode);
            Uart::Puts("\n");

            if (device.ClassCode == 0x0c0330) // USB xHCI controller
            {
                auto usbInitTask = Usb::Xhci::UsbInitializeXhci(pcie, device.Address);
                usbDriver = WaitOnTask(std::move(usbInitTask));
            }
        }
    }
    else
    {
        auto usbInitTask = UsbInitializeDesignWare();

        usbDriver = WaitOnTask(std::move(usbInitTask));
    }

    if (usbDriver)
    {
        (void)WaitOnTask(usbDriver->UsbCheckForChange());
        printf("\n");
        usbDriver->UsbShowTree();
        printf("\n");
    }

    std::println("Initializing framebuffer...");

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

    Run(*usbDriver);
}

}
// extern "C"
