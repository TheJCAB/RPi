#pragma once

#include <BootLib/Cpu.h>

#include "Uart.h"

namespace Cpu
{

using namespace BootLib::Cpu;

extern uint64_t const PerformanceFrequency;

[[noreturn]] void Panic(char const* message);

}
// namespace Cpu
