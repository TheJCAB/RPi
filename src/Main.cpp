#include <stdint.h>
#include <stddef.h>

#include "Mmio.h"
#include "Uart.h"
#include "Framebuffer.h"

extern "C"
{

uint64_t StackBuffer[0x10000/8];

void KernelMain()
{
    uint64_t core_id = 0; // _ReadStatusReg(MPIDR_EL1);
    asm volatile ("mrs %0, mpidr_el1" : "=r"(core_id));

    if ((core_id & 3) != 0) {
        // Only core 0 should initialize the framebuffer
        for (;;) {}
    }

    Uart::Init();

    Uart::Puts("\r\n\nHello!\n");
    
    uint32_t const w = 128;
    uint32_t const h =  72;

    Framebuffer::Init(w, h);

    for (uint32_t x = 0; x < w; ++x) {
        for (uint32_t y = 0; y < h; ++y) {
            // Set pixel color (example: green)
            if (x < 8)
            {
                Framebuffer::WritePixel(x, y, Blue);
            }
            else if (y < 8)
            {
                Framebuffer::WritePixel(x, y, Cyan);
            }
            else if (x >= w - 8)
            {
                Framebuffer::WritePixel(x, y, Red);
            }
            else if (y >= h - 8)
            {
                Framebuffer::WritePixel(x, y, Yellow);
            }
            else
            {
                Framebuffer::WritePixel(x, y, Green);
            }
        }
    }

    for (;;) {}
}

}
// extern "C"
