#include "Run.h"
#include "Uart.h"
#include "Timer.h"
#include "Framebuffer.h"
#include "Keyboard.h"

#include "emb-stdio.h"

void Run()
{
    Keyboard::Init();

    Color565 background = Magenta;
    for (;;)
    {
        for (int i = 0; i < 2; ++i)
        {
            if (Keyboard::IsKeyPressed(' '))
            {
                background = Blue;
            }
            else
            {
                background = Magenta;
            }

            Timer::Delay(33'333); // Delay for 33.333 ms (30 FPS)
            for (uint32_t x = 0; x < Framebuffer::Width; ++x)
            {
                for (uint32_t y = 0; y < Framebuffer::Height; ++y)
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
            Framebuffer::Flip();
            //Uart::Puts("Framebuffer flipped.\n");
        }
    }
}
