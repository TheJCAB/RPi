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
#include "Processor.h"
#include "Mmu.h"
#include "Timer.h"
#include "Run.h"
#include "UsbDevices.h"
#include "PCIe.h"

#include "emb-stdio.h"

void parse_dtb(void* dtb);

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
    {
        // Without MMU, we need to use physical addresses to access peripherals.
        BootLib::PL011Uart uart{ BootLib::Mmio::GetPeripheralsPhysicalBase() + BootLib::PL011Uart::Uart0RegistersOffset };
        uart.Puts("Core 1 starting\n");
    }

    InitCore();

    BootLib::PL011Uart uart{ Mmio::Base + BootLib::PL011Uart::Uart0RegistersOffset };
    uart.Puts("Core 1 says hello\n");

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

void Core0(void* dtb, void* p1, void* p2, void* p3)
{
    {
        Cpu::cpacr_el1.modify([](auto& reg){
            reg.FPEN = 3; // Enable the Floating point and SIMD unit for EL0 and EL1
        });

        // Without MMU, we need to use physical addresses to access peripherals.
        BootLib::PL011Uart uart{ BootLib::Mmio::GetPeripheralsPhysicalBase() + BootLib::PL011Uart::Uart0RegistersOffset };
        uart.Puts("Core 0 starting\n");

        // Clear the BSS.
        for (auto p = &_bss_start; p < &_bss_end; ++p)
        {
            *p = 0; // Clear BSS
        }

        // Call all global initializers.
        for (auto ctor = _init_array_start; ctor < _init_array_end; ++ctor) {
            if (ctor != nullptr)
            {
                (*ctor)();
            }
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
        uart.PutDec(Cpu::PerformanceFrequency);
        uart.Puts("\n");

        uart.Puts("Reading DTB at ");
        uart.PutHex(reinterpret_cast<uintptr_t>(dtb));
        uart.Puts("\n");

        uart.Puts("p1 ");
        uart.PutHex(reinterpret_cast<uintptr_t>(p1));
        uart.Puts("\n");

        uart.Puts("p2 ");
        uart.PutHex(reinterpret_cast<uintptr_t>(p2));
        uart.Puts("\n");

        uart.Puts("p3 ");
        uart.PutHex(reinterpret_cast<uintptr_t>(p3));
        uart.Puts("\n");

        Exception::InitEL2();
        // Configure HCR_EL2 to route exceptions to EL2
        asm volatile("mrs x0, hcr_el2");
        asm volatile("orr x0, x0, #0x8");  // Set AMO bit to route SError to EL2
        asm volatile("msr hcr_el2, x0");
        asm volatile("isb");
        Cpu::daifclr = 4; // Clear the SError mask bit to enable synchronous exceptions
        //asm volatile ("svc #0");
        //*(volatile int32_t*)(0x1234567890123456) = 0; // Clear the mailbox status register
    }

    bool const isRpi4 = Cpu::IsRpi4();
    if (isRpi4)
    {
        Mmio::Base    = 0x4'7E00'0000u; // RPi4 MMIO base address
        Mmio::QA7Base = 0x4'C000'0000u; // RPi4 QA7 base address
    }

    Mailbox::Send(0, 0x80); // UART 1 and USB enabled

    Uart::Puts("\r\n\nHello!\n");

    uint64_t el = Cpu::CurrentEL->EL;
    //asm volatile ("mrs %0, CurrentEL" : "=r"(el));
    Uart::Puts("CurrentEL: ");
    Uart::PutDec(el);
    Uart::Puts("\n");

    if (el == 2)
    {
        el2_to_el1_return();

        Uart::Puts("Lowered to EL1\n");

        Uart::Puts("CurrentEL: ");
        Uart::PutDec(Cpu::CurrentEL->EL);
        Uart::Puts("\n");

    }

    Mmu::Init();

    Uart::Puts("MMU enabled\n");

    Uart::Puts("Performance Frequency: ");
    Uart::PutDec(Cpu::PerformanceFrequency);
    Uart::Puts("\n");

    Exception::Init();

    Cpu::sctlr_el1.modify([](auto& reg){
        //reg.A  = true; // Set A (Alignment check enable) bit
        reg.SA = true; // Set SA (Stack Alignment Check Enable) bit
    });

    asm volatile ("isb"); // Ensure changes take effect

    // Enable SError, IRQ and FIQ.
    // Note: SError means synchronous exceptions, all caused by the executing code,
    // but not necessarily means errors. It includes system calls, memory faults, etc...
    Cpu::daifclr = 7;

    parse_dtb(dtb);

    Uart::Puts("Waiting...\n");
    Cpu::DelayInMicroseconds(1000'000);
    
    //asm volatile ("svc #42"); // Trigger a software interrupt to test exception handling

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
                    if (j == 8)
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
                    if (j == 8)
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
    }

    if (isRpi4)
    {
        PCIe::examples::demonstrate_enumeration();
    }
    else
    {
        Timer::SetPeriodicVirtualTimerInterrupt(1000'000); // Set a periodic interrupt every second
        //Cpu::DelayInMicroseconds(3'000'000);

        UsbInitialise();
        Cpu::DelayInMicroseconds(10'000);
        Uart::Init();

        Uart::Puts("Waiting...\n");
        Cpu::DelayInMicroseconds(1000'000);

        UsbCheckForChange();
        Uart::Init();

        /* Display the USB tree */
        printf2("\n");
        UsbShowTree(UsbGetRootHub(), 1, '+');
        printf2("\n");
    }

    Uart::Puts("Waiting...\n");
    Cpu::DelayInMicroseconds(1000'000);

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
