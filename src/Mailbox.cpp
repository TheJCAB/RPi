#include "Mailbox.h"

#include "Processor.h"
#include "Uart.h"
#include "Mmio.h"

uintptr_t GpuMemBase = 0;

namespace Mailbox
{
    
// Mailbox registers (base address for RPi 3B)
#define MAILBOX_BASE    (MMIO_BASE + 0xB880)
#define MAILBOX_READ    ((uint32_t volatile*)(MAILBOX_BASE + 0x00))
#define MAILBOX_RSTATUS ((uint32_t volatile*)(MAILBOX_BASE + 0x18))
#define MAILBOX_WRITE   ((uint32_t volatile*)(MAILBOX_BASE + 0x20))
#define MAILBOX_WSTATUS ((uint32_t volatile*)(MAILBOX_BASE + 0x38))
#define MAILBOX_FULL    0x80000000
#define MAILBOX_EMPTY   0x40000000

// Mailbox call function
void Send(uint8_t ch, uint32_t data)
{
    if (data & 0xFu)
    {
        //Uart::Puts("Oh, noes! Mailbox data must keep the bottom 4 bits empty for the channel.\n");
        Processor::Halt();
    }

    uint32_t const r = (data & ~0xFu) | (ch & 0xFu);
    while (!(*MAILBOX_RSTATUS & MAILBOX_EMPTY))
    {
        asm volatile ("dmb ish" ::: "memory");
        auto const read = *MAILBOX_READ;
        //Uart::Puts("Mailbox not empty. Read status: ");
        //Uart::PutHex(read);
        //Uart::Puts("\n");
    }

    // Wait until mailbox is not full
    asm volatile ("dmb ish" ::: "memory");
    while (*MAILBOX_WSTATUS & MAILBOX_FULL)
    {
        //Uart::Puts("Mailbox full, waiting...\n");
        asm volatile ("dmb ish" ::: "memory");
    }
    asm volatile ("dmb ish" ::: "memory");
    *MAILBOX_WRITE = r;
    asm volatile ("dmb ish" ::: "memory");
    // Wait for response
    while (true) {
        asm volatile ("dmb ish" ::: "memory");
        while (*MAILBOX_RSTATUS & MAILBOX_EMPTY)
        {
            asm volatile ("dmb ish" ::: "memory");
        }
        asm volatile ("dmb ish" ::: "memory");
        auto const read = *MAILBOX_READ;
        //Uart::Puts("Mailbox read status: ");
        //Uart::PutHex(read);
        //Uart::Puts("\n");
        if ((read & 0xF) == ch)
        {
            if ((read & 0xFFFF'FFF0) != data)
            {
                //Uart::Puts("Mailbox call failed.\n");
                Processor::Halt();
            }
            asm volatile ("dmb ish" ::: "memory");
            return;
        }
    }
}

// Mailbox call function
void Send(uint8_t ch, void const volatile* data)
{
    if (reinterpret_cast<uintptr_t>(data) < GpuMemBase ||
        reinterpret_cast<uintptr_t>(data) >= GpuMemBase + 0x4000'0000u)
    {
        Uart::Puts("Oh, noes! Mailbox data must be in the GPU memory range.\n");
        Processor::Halt();
    }

    asm volatile ("dsb osh" ::: "memory");
    return Send(ch, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(data)));
}

bool SendTags(std::span<uint32_t volatile> data)
{
    if (data.size() <= 2)
    {
        Uart::Puts("Oh, noes! Mailbox tag buffers must have more than 2 elements.\n");
        Processor::Halt();
    }

    data[0] = data.size() * 4;
    data[1] = 0;

    Send(8, data.data());

    return data[1] == 0x80000000; // Check if the response is successful
}

}
// namespace Mailbox
