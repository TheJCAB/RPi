#include "Framebuffer.h"

#include "Mmio.h"

#include <stddef.h>

namespace Framebuffer
{

// Mailbox registers (base address for RPi 3B)
#define MAILBOX_BASE    (MMIO_BASE + 0xB880)
#define MAILBOX_READ    ((uint32_t volatile*)(MAILBOX_BASE + 0x00))
#define MAILBOX_RSTATUS ((uint32_t volatile*)(MAILBOX_BASE + 0x18))
#define MAILBOX_WRITE   ((uint32_t volatile*)(MAILBOX_BASE + 0x20))
#define MAILBOX_WSTATUS ((uint32_t volatile*)(MAILBOX_BASE + 0x38))
#define MAILBOX_FULL    0x80000000
#define MAILBOX_EMPTY   0x40000000

// Mailbox property buffer (must be 16-byte aligned)
alignas(64) uint32_t volatile mbox_l[1024];

#define mbox ((uint32_t volatile*)((uintptr_t)mbox_l /*| 0xC000'0000*/))

// Mailbox call function
int mailbox_call(unsigned char ch)
{
    uint32_t const r = (static_cast<uint32_t>(reinterpret_cast<uintptr_t>(mbox)) & ~0xFu) | (ch & 0xFu) | 0xC000'0000u;
    while (!(*MAILBOX_RSTATUS & MAILBOX_EMPTY))
    {
        asm volatile ("dmb ish" ::: "memory");
        *MAILBOX_READ;
    }

    // Wait until mailbox is not full
    asm volatile ("dmb ish" ::: "memory");
    while (*MAILBOX_WSTATUS & MAILBOX_FULL)
    {
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
        if (*MAILBOX_READ == r)
        {
            asm volatile ("dmb ish" ::: "memory");
            return mbox[1] == 0x80000000;
        }
    }
}

void Panic(Color565 color, int divisions, int which, int repeat)
{
    uint64_t const value =
        (static_cast<uint64_t>(reinterpret_cast<uint16_t const&>(color)) << 48) |
        (static_cast<uint64_t>(reinterpret_cast<uint16_t const&>(color)) << 32) |
        (static_cast<uint64_t>(reinterpret_cast<uint16_t const&>(color)) << 16) |
        static_cast<uint64_t>(reinterpret_cast<uint16_t const&>(color));

    auto const vSize  = 16 * 1024 * 1024 / 8;
    auto const vStart = (uint64_t volatile*)(uintptr_t)(0x3F000000) - vSize;
    auto const size   = vSize / divisions;
    auto const start  = vStart + size * which;
    for (; repeat > 0; --repeat)
    {
        for (int i = 0; i < size; ++i)
        {
            start[i] = value; // Clear all video memory to the given color.
        }
    }
}

// Framebuffer info
volatile unsigned int fb_width = 1280, fb_height = 720, fb_depth = 16;
volatile unsigned int fb_pitch, fb_addr;

void Init(uint32_t width, uint32_t height)
{
    fb_width  = width;
    fb_height = height;

    int i = 0;
    mbox[i++] = 0; // Size
    mbox[i++] = 0; // Request

    mbox[i++] = 0x48003; mbox[i++] = 8; int m1 = i; mbox[i++] = 0; mbox[i++] = fb_width; mbox[i++] = fb_height; // Set phys size
    mbox[i++] = 0x48004; mbox[i++] = 8; int m2 = i; mbox[i++] = 0; mbox[i++] = fb_width; mbox[i++] = fb_height; // Set virt size
    mbox[i++] = 0x48005; mbox[i++] = 4; int m3 = i; mbox[i++] = 0; mbox[i++] = fb_depth; // Set depth
    mbox[i++] = 0x48006; mbox[i++] = 4; int m4 = i; mbox[i++] = 0; mbox[i++] = 0; // Set pixel order
    mbox[i++] = 0x40001; mbox[i++] = 8; int m5 = i; mbox[i++] = 0; int addr = i; mbox[i++] = 16; mbox[i++] = 0; // Allocate buffer
    mbox[i++] = 0x40008; mbox[i++] = 4; int m6 = i; mbox[i++] = 0; int pitch = i; mbox[i++] = 0; // Get pitch
    mbox[i++] = 0; // End tag

    // Pad to 16-byte alignment
    while (i & 3)
    {
        mbox[i++] = 0;
    }

    mbox[0] = 1024 * 4;

    asm volatile (
        "dc cvau, %0\n"
        "dsb ish\n"
        "isb\n"
        :
        : "r"(mbox)
        : "memory"
    );
    for (size_t off = 0; off < i * 4; off += 64) {
        asm volatile (
            "dc cvau, %0\n"
            :
            : "r"(((char*)mbox) + off)
            : "memory"
        );
    }
    asm volatile ("dsb ish; isb" ::: "memory");

    if (mailbox_call(8)) {
        fb_addr  = mbox[addr] & 0x3FFF'FFFF; // Convert to ARM address
        fb_pitch = mbox[pitch];
        // Framebuffer is now accessible at fb_addr
    }
    else{
        // Error?
        while (true) {
            //int const k = (255 * 4 + 2) * 4 + 3;
            if (mbox[ 0] != 1024 * 4  ) Panic(Color565{ 0x0F, 0x1F, 0x0F }, 4096, 4091, 1000); // Panic with white color
            if (mbox[ 1] != 0x80000000) Panic(Color565{ 0x00, 0x3F, 0x1F }, 4096, 4091, 1000); // Panic with white color
            if (mbox[ 1] != 0x00000000) Panic(Color565{ 0x00, 0x00, 0x00 }, 4096, 4091, 1000); // Panic with white color
            if (mbox[m1] != 0x80000000) Panic(Color565{ 0x1F, 0x3F, 0x1F }, 4096, 4091, 1000); // Panic with white color
            if (mbox[m1] != 0x00000000) Panic(Color565{ 0x00, 0x00, 0x00 }, 4096, 4091, 1000); // Panic with white color
            if (mbox[m2] != 0x80000000) Panic(Color565{ 0x00, 0x00, 0x1F }, 4096, 4091, 1000); // Panic with red color
            if (mbox[m2] != 0x00000000) Panic(Color565{ 0x00, 0x00, 0x00 }, 4096, 4091, 1000); // Panic with red color
            if (mbox[m3] != 0x80000000) Panic(Color565{ 0x00, 0x3F, 0x00 }, 4096, 4091, 1000); // Panic with green color
            if (mbox[m3] != 0x00000000) Panic(Color565{ 0x00, 0x00, 0x00 }, 4096, 4091, 1000); // Panic with green color
            if (mbox[m4] != 0x80000000) Panic(Color565{ 0x1F, 0x00, 0x00 }, 4096, 4091, 1000); // Panic with blue color
            if (mbox[m4] != 0x00000000) Panic(Color565{ 0x00, 0x00, 0x00 }, 4096, 4091, 1000); // Panic with blue color
            if (mbox[m5] != 0x80000000) Panic(Color565{ 0x1F, 0x3F, 0x00 }, 4096, 4091, 1000); // Panic with blue color
            if (mbox[m5] != 0x00000000) Panic(Color565{ 0x00, 0x00, 0x00 }, 4096, 4091, 1000); // Panic with blue color
            if (mbox[m6] != 0x80000000) Panic(Color565{ 0x1F, 0x00, 0x1F }, 4096, 4091, 1000); // Panic with blue color
            if (mbox[m6] != 0x00000000) Panic(Color565{ 0x00, 0x00, 0x00 }, 4096, 4091, 1000); // Panic with blue color
            //Panic(Color565{ 0x1F, 0x3F, 0x1F }, 256 * 4 * 4, (255 * 4 + 2) * 4 + 0); // Panic with white color
            //Panic(Color565{ 0x00, 0x00, 0x1F }, 256 * 4 * 4, (255 * 4 + 2) * 4 + 1); // Panic with red color
            //Panic(Color565{ 0x00, 0x3F, 0x00 }, 256 * 4 * 4, (255 * 4 + 2) * 4 + 2); // Panic with green color
            Panic(Color565{ 0x1F, 0x00, 0x00 }, 256 * 4 * 4, (255 * 4 + 2) * 4 + 3); // Panic with blue color
            Panic(Color565{ 0x1F, 0x3F, 0x1F }); // Panic with white color
            Panic(Color565{ 0x00, 0x00, 0x1F }); // Panic with red color
            //Panic(Color565{ 0x00, 0x3F, 0x00 }); // Panic with green color
            //Panic(Color565{ 0x1F, 0x00, 0x00 }); // Panic with blue color
            //Panic(Color565{ 0x00, 0x00, 0x00 }); // Panic with black color
        }
    }
}

void WritePixel(uint32_t x, uint32_t y, Color565 color)
{
    if (x < fb_width && y < fb_height) {
        *((uint16_t volatile*)((uintptr_t)fb_addr + (y * fb_pitch) + (x * 2))) = reinterpret_cast<uint16_t const&>(color);
    }
}

}
// namespace Framebuffer
