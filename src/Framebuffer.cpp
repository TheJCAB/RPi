#include "Framebuffer.h"

#include "Mmio.h"
#include "Mailbox.h"

#include "Uart.h"

#include <stddef.h>

extern uintptr_t GpuMemBase;

namespace Framebuffer
{
    
// Mailbox property buffer (must be 16-byte aligned)
alignas(64) uint32_t volatile mbox_l[64];

#define mbox ((uint32_t volatile*)((uintptr_t)mbox_l | GpuMemBase))

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
uint32_t  fb_depth = 16;
uintptr_t fb_addr;
uint32_t  fb_pitch;


uint32_t Width;
uint32_t Height;

void Init(uint32_t width, uint32_t height)
{
    Uart::Puts("Framebuffer initialization started...\n");
    Uart::Puts("GPU memory base: "); Uart::PutHex(GpuMemBase); Uart::Puts("\n");

    size_t i = 0;
    mbox[i++] = 0; // Size
    mbox[i++] = 0; // Request

    mbox[i++] = 0x48003; mbox[i++] = 8; size_t m1 = i; mbox[i++] = 0; mbox[i++] = width; size_t h = i; mbox[i++] = height; // Set phys size
    mbox[i++] = 0x48004; mbox[i++] = 8; size_t m2 = i; mbox[i++] = 0; size_t w = i; mbox[i++] = width; size_t ph = i; mbox[i++] = height * 2; // Set virt size
    mbox[i++] = 0x48005; mbox[i++] = 4; size_t m3 = i; mbox[i++] = 0; mbox[i++] = fb_depth; // Set depth
    mbox[i++] = 0x48006; mbox[i++] = 4; size_t m4 = i; mbox[i++] = 0; mbox[i++] = 0; // Set pixel order
    mbox[i++] = 0x40001; mbox[i++] = 8; size_t m5 = i; mbox[i++] = 0; size_t addr = i; mbox[i++] = 16; size_t size = i; mbox[i++] = 0; // Allocate buffer
    mbox[i++] = 0x40008; mbox[i++] = 4; size_t m6 = i; mbox[i++] = 0; size_t pitch = i; mbox[i++] = 0; // Get pitch
    mbox[i++] = 0; // End tag

    // Pad to 16-byte alignment
    while (i & 3)
    {
        mbox[i++] = 0;
    }

    if (Mailbox::SendTags(std::span{ mbox, i }))
    {
        fb_addr  = (mbox[addr] & 0x3FFF'FFFF) + GpuMemBase; // Convert to ARM address
        fb_pitch = mbox[pitch];
        Width = mbox[w];
        Height = mbox[h];

        // Framebuffer is now accessible at fb_addr
        Uart::Puts("Framebuffer address: "); Uart::PutHex(fb_addr); Uart::Puts("\n");
        Uart::Puts("Framebuffer size: "); Uart::PutHex(mbox[size]); Uart::Puts("\n");
        Uart::Puts("Framebuffer pitch: "); Uart::PutDec(fb_pitch); Uart::Puts("\n");
        Uart::Puts("Framebuffer width: "); Uart::PutDec(Width); Uart::Puts("\n");
        Uart::Puts("Framebuffer height: "); Uart::PutDec(Height); Uart::Puts("\n");
        Uart::Puts("Framebuffer physical height: "); Uart::PutDec(mbox[ph]); Uart::Puts("\n");
    }
    else
    {
        Uart::Puts("Framebuffer initialization failed.\n");
        Uart::Puts("Mbox size: "); Uart::PutDec(mbox[0]); Uart::Puts("\n");
        Uart::Puts("Mbox status: "); Uart::PutHex(mbox[1]); Uart::Puts("\n");
        Uart::Puts("Mbox m1: "); Uart::PutHex(mbox[m1]); Uart::Puts("\n");
        Uart::Puts("Mbox m2: "); Uart::PutHex(mbox[m2]); Uart::Puts("\n");
        Uart::Puts("Mbox m3: "); Uart::PutHex(mbox[m3]); Uart::Puts("\n");
        Uart::Puts("Mbox m4: "); Uart::PutHex(mbox[m4]); Uart::Puts("\n");
        Uart::Puts("Mbox m5: "); Uart::PutHex(mbox[m5]); Uart::Puts("\n");
        Uart::Puts("Mbox m6: "); Uart::PutHex(mbox[m6]); Uart::Puts("\n");
        Uart::Puts("Framebuffer address: "); Uart::PutHex(fb_addr); Uart::Puts("\n");
        Uart::Puts("Framebuffer pitch: "); Uart::PutDec(fb_pitch); Uart::Puts("\n");

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

uint32_t yOffset = 0;

void Flip()
{
    size_t i = 0;
    mbox[i++] = 0; // Size
    mbox[i++] = 0; // Request

    mbox[i++] = 0x48009; mbox[i++] = 8; mbox[i++] = 0; mbox[i++] = 0; mbox[i++] = yOffset; // Set phys size
    mbox[i++] = 0; // End tag

    // Pad to 16-byte alignment
    while (i & 3)
    {
        mbox[i++] = 0;
    }

    if (Mailbox::SendTags(std::span{ mbox, i }))
    {
        if (yOffset == 0)
        {
            yOffset = Height;
        }
        else
        {
            yOffset = 0;
        }
    }
    else
    {
        Uart::Puts("Framebuffer flip failed.\n");
        Uart::Puts("Mbox size: "); Uart::PutDec(mbox[0]); Uart::Puts("\n");
        Uart::Puts("Mbox status: "); Uart::PutHex(mbox[1]); Uart::Puts("\n");
        Uart::Puts("Mbox status2: "); Uart::PutHex(mbox[4]); Uart::Puts("\n");
        Uart::Puts("Mbox x: "); Uart::PutHex(mbox[5]); Uart::Puts("\n");
        Uart::Puts("Mbox y: "); Uart::PutHex(mbox[6]); Uart::Puts("\n");
    }
}

void WritePixel(uint32_t x, uint32_t y, Color565 color)
{
    if (x < Width && y < Height)
    {
        *((uint16_t volatile*)(fb_addr + ((y + yOffset) * fb_pitch) + (x * 2))) = reinterpret_cast<uint16_t const&>(color);
    }
}

void WriteSpan(uint32_t x, uint32_t y, uint32_t w, Color565 color)
{
    if (x + w <= Width && y < Height)
    {
        uint16_t const color16 = reinterpret_cast<uint16_t const&>(color);
        uint64_t const color64 =
            (static_cast<uint64_t>(color16) << 48) |
            (static_cast<uint64_t>(color16) << 32) |
            (static_cast<uint64_t>(color16) << 16) |
            (static_cast<uint64_t>(color16)      );
        auto const spanPtr16 = reinterpret_cast<uint16_t*>(fb_addr + ((y + yOffset) * fb_pitch));
        auto const spanPtr64 = reinterpret_cast<uint64_t*>(spanPtr16);
        auto const x2 = x + w;
        while (x % 4 != 0 && x < x2)
        {
            spanPtr16[x] = color16;
            ++x;
        }

        while (x + 4 <= x2)
        {
            spanPtr64[x / 4] = color64;
            x += 4;
        }

        while (x < x2)
        {
            spanPtr16[x] = color16;
            ++x;
        }
    }
}

void WriteRectangle(uint32_t x, uint32_t y, uint32_t w, uint32_t h, Color565 color)
{
    if (x + w <= Width && y + h <= Height)
    {
        for (uint32_t i = 0; i < h; ++i)
        {
            WriteSpan(x, y + i, w, color);
        }
    }
}

}
// namespace Framebuffer
