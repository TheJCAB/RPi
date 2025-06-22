#include "Run.h"
#include "Uart.h"
#include "Timer.h"
#include "Framebuffer.h"

void Run()
{
    Color565 background = Magenta;
    for (;;)
    {
        for (int i = 0; i < 2; ++i)
        {
            if (auto c = Uart::TryGetc())
            {
                //Uart::Puts("Got character: ");
                //Uart::PutHex((uint8_t)c);
                //Uart::Puts("\n");

                if (c == 0x1 && Uart::Getc() == ' ')
                {
                    //Uart::Putc('1');
                    background = Blue;
                }
                else if (c == 0x2 && Uart::Getc() == ' ')
                {
                    //Uart::Putc('2');
                    background = Magenta;
                }
                else
                {
                    //Uart::Putc(' ');
                    //Uart::PutHex((uint8_t)c);
                }
            }
            Timer::Delay(16'000); // Delay for 16 ms (60 FPS)
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
                        Framebuffer::WritePixel(x, y, i == 0 ? Green : background);
                    }
                }
            }
        }
    }
}
