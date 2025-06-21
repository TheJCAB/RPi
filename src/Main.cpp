#include <stdint.h>
#include <stddef.h>

#include "Mmio.h"
#include "Uart.h"
#include "Framebuffer.h"
#include "Mmu.h"
#include "Run.h"

uintptr_t MMIO_BASE = 0x3F00'0000u;

extern "C" uint64_t _bss_start;
extern "C" uint64_t _bss_end;

extern "C" uint64_t _data_start;
extern "C" uint64_t _data_end;

extern "C"
{

extern "C" void el2_to_el1_return();

void KernelMain()
{
    uint64_t core_id = 0; // _ReadStatusReg(MPIDR_EL1);
    asm volatile ("mrs %0, mpidr_el1" : "=r"(core_id));

    if ((core_id & 3) != 0) {
        // Only core 0 should initialize the framebuffer
        for (;;) {}
    }

    for (auto p = &_bss_start; p < &_bss_end; ++p)
    {
        *p = 0; // Clear BSS
    }

    Uart::Init();

    Uart::Puts("\r\n\nHello!\n");

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
        uint64_t spsr = 0;
        asm volatile ("mrs %0, spsr_el2" : "=r"(spsr));
        Uart::Puts("SPSR_EL2: ");
        Uart::PutBin(spsr);
        Uart::Puts("\n");

        el2_to_el1_return();

        Uart::Puts("Running EL1\n");

        asm volatile ("mrs %0, CurrentEL" : "=r"(el));
        Uart::Puts("CurrentEL: ");
        Uart::PutDec((el >> 2) & 0b11);
        Uart::Puts("\n");

        Mmu::Init();

        MMIO_BASE = 0xFF00'0000u; // Update MMIO base to the new aperture.
        Framebuffer::GpuMemBase = 0x4000'0000u; // Update the GPU memory base to the new aperture.

        Uart::Puts("MMU enabled\n");
    }

    uint64_t spsr = 0;
    asm volatile ("mrs %0, spsr_el1" : "=r"(spsr));
    Uart::Puts("SPSR_EL1: ");
    Uart::PutBin(spsr);
    Uart::Puts("\n");

    uint32_t const w = 1280;
    uint32_t const h =  720;

    Framebuffer::Init(w, h);

    Run();
}

}
// extern "C"
