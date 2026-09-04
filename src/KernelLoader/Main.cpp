#include <stdint.h>
#include <stddef.h>

#include <BootLib/Cpu.h>
#include <BootLib/Mmio.h>
#include <BootLib/Gpio.h>
#include <BootLib/Uart.h>

extern "C" uint64_t _start;
extern "C" uint64_t _end;

extern "C" [[noreturn]] void KernelMain(void* p0, void* p1, void* dtb, void* p3);

extern "C" [[noreturn]] void GetNewKernel(BootLib::PL011Uart uart0, uint8_t* destination, void* p0, void* p1, void* dtb, void* p3)
{
    uart0.Puts("\n\nLet's load a kernel over UART!\n\n");

    // We send three threes to start the protocol.
    uart0.Puts("\03\03\03");

    // The server sends "OK" to signal readiness.
    while (uart0.Getc() != 'O') {}
    while (uart0.Getc() != 'K') {}

    // 32-bit size in little endian format.
    uint8_t const s0 = uart0.Getc();
    uint8_t const s1 = uart0.Getc();
    uint8_t const s2 = uart0.Getc();
    uint8_t const s3 = uart0.Getc();
    uint32_t const size = (
        static_cast<uint32_t>(s0) | 
        static_cast<uint32_t>(s1) << 8 |
        static_cast<uint32_t>(s2) << 16 |
        static_cast<uint32_t>(s3) << 24
    );

    // We acknowledge the size.
    uart0.Puts("OK");

    uart0.Puts("Size: ");
    uart0.PutDec(size);
    uart0.Puts("\n");

    uint8_t* dest = reinterpret_cast<uint8_t*>(&_start);
    for (uint32_t i = 0; i < size; ++i)
    {
        destination[i] = uart0.Getc();
    }

    reinterpret_cast<decltype(KernelMain)*>(destination)(p0, p1, dtb, p3);
    BootLib::Cpu::Halt(); // Should never reach here.
}

extern "C" uint64_t _bss_start;
extern "C" uint64_t _bss_end;

extern "C" [[noreturn]] void KernelMain(void* p0, void* p1, void* dtb, void* p3)
{
    // Clear the BSS soonest.
    for (auto p = &_bss_start; p < &_bss_end; ++p)
    {
        *p = 0;
    }


    auto const peripheralsBase = BootLib::Mmio::GetPeripheralsPhysicalBase();

    BootLib::Uart::MiniUart ::Disable(peripheralsBase + BootLib::Uart::MiniUart ::UartRegistersOffset );
    BootLib::Uart::PL011Uart::Disable(peripheralsBase + BootLib::Uart::PL011Uart::Uart0RegistersOffset);

    BootLib::Gpio gpio{ peripheralsBase + BootLib::Gpio::RegistersOffset };

    gpio.SetUart0_14_15();

    BootLib::Uart::PL011Uart uart0{ peripheralsBase + BootLib::Uart::PL011Uart::Uart0RegistersOffset };

    uart0.Puts("\n\nRelocating...\n");

    uint32_t const offsetInBytes = 0x4000; // Relocate to 16K bytes before the start address;

    uint64_t* newLocation = &_start - offsetInBytes / 8;
    uintptr_t newAddress = reinterpret_cast<uintptr_t>(newLocation);

    // Copy the data to the new location.
    for (auto p = &_start; p < &_end; ++p)
    {
        *newLocation++ = *p;
    }

    reinterpret_cast<decltype(GetNewKernel)*>(reinterpret_cast<uintptr_t>(&GetNewKernel) - offsetInBytes)(uart0, reinterpret_cast<uint8_t*>(&_start), p0, p1, dtb, p3);
    BootLib::Cpu::Halt(); // Should never reach here.
}

