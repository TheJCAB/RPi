#pragma once

#include "Uart.h"

namespace Processor
{

[[noreturn]] inline void Halt()
{
    Uart::Puts("The processor has been halted.\n");
    for (;;)
    {
        asm volatile ("wfe"); // Wait for event
    }
}

}
// namespace Processor
