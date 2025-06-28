#include "Run.h"
#include "Uart.h"
#include "Timer.h"
#include "Framebuffer.h"
#include "HidUsbDevices.h"

#include "emb-stdio.h"

void Run(uint8_t firstKbd)
{
    HIDEnableInterruptINSimple(firstKbd, 0);

    Color565 background = Magenta;
    for (;;)
    {
        for (int i = 0; i < 2; ++i)
        {
            if (firstKbd) {
                uint16_t const USB_HID_REPORT_TYPE_INPUT = 1;
                uint8_t buf[8];
                auto const status = HIDReadInterruptReport(firstKbd, 0, buf, sizeof(buf), nullptr);
                //auto const status = HIDReadReport(firstKbd, 0, USB_HID_REPORT_TYPE_INPUT << 8 | 1, &buf[0], 8);
                if (status == RESULT::Ok)
                {
                    //GotoXY(x, y);
                    printf("HID KBD REPORT: %08b %02X %02X %02X %02X %02X %02X\n",
                        buf[0],         buf[2], buf[3],
                        buf[4], buf[5], buf[6], buf[7]);

                    //HIDSetIdle(firstKbd, 0);
                }
                else printf("Status error: %d\n", status);
            }

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
            Timer::Delay(33'333); // Delay for 33.333 ms (30 FPS)
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
