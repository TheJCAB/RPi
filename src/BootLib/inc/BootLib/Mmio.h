#pragma once

#include <BootLib/RegisterProxy.h>

#include <stdint.h>

// This file defines the MMIO (Memory-Mapped I/O) memory regions for the Raspberry Pi.
namespace BootLib::Mmio
{

// We handle the MMIO base dynamically so we can support multiple versions of Raspberry Pi.

// 0x2000'0000u for Raspberry Pi 1, 0x3F00'0000u for Raspberry Pi 2/3, 0x7E00'0000u or 0x4'7E00'0000u for Raspberry Pi 4
uintptr_t GetPeripheralsPhysicalBase();

}
// namespace BootLib::Mmio
