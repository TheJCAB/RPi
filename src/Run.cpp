#include "Run.h"
#include "Uart.h"
#include "Framebuffer.h"

void Run()
{
    for (;;)
    {
        for (int i = 0; i < 2; ++i)
        {
            for (uint32_t x = 0; x < Framebuffer::Width; ++x)
            {
                for (uint32_t y = 0; y < Framebuffer::Width; ++y)
                {
                    // Set pixel color (example: green)
                    if (x < 8)
                    {
                        Framebuffer::WritePixel(x, y, Blue);
                    }
                    else if (y < 8)
                    {
                        Framebuffer::WritePixel(x, y, Cyan);
                    }
                    else if (x >= Framebuffer::Width - 8)
                    {
                        Framebuffer::WritePixel(x, y, Red);
                    }
                    else if (y >= Framebuffer::Width - 8)
                    {
                        Framebuffer::WritePixel(x, y, Yellow);
                    }
                    else
                    {
                        Framebuffer::WritePixel(x, y, i == 0 ? Green : Magenta);
                    }
                }
            }
        }
    }
}
