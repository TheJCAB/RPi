#pragma once

#include <BootLib/RegisterProxy.h>

#include <stdint.h>

// This file defines the MMIO (Memory-Mapped I/O) memory regions for the Raspberry Pi.
namespace BootLib::Mmio
{

// MMIO bases for all Raspberry Pi boards (up to 4 so far).

constexpr uintptr_t Rpi1Base    = 0x2000'0000u;
constexpr uintptr_t Rpi1QA7Base = 0x4000'0000u;

constexpr uintptr_t Rpi2Base    = 0x3F00'0000u;
constexpr uintptr_t Rpi2QA7Base = 0x4000'0000u;

constexpr uintptr_t Rpi3Base    = 0x3F00'0000u;
constexpr uintptr_t Rpi3QA7Base = 0x4000'0000u;

constexpr uintptr_t Rpi4BaseLo    = 0xFE00'0000u;
constexpr uintptr_t Rpi4QA7BaseLo = 0xFF80'0000u;

constexpr uintptr_t Rpi4BaseHi    = 0x4'7E00'0000ull;
constexpr uintptr_t Rpi4QA7BaseHi = 0x4'C000'0000ull;

// TODO: We should figure out a way to detect the high-peripherals setting automatically.
// (from the devicetree? Or reading the config file ourselves?)
constexpr uintptr_t Rpi4Base    = Rpi4BaseLo;
constexpr uintptr_t Rpi4QA7Base = Rpi4QA7BaseLo;

// We handle the MMIO base dynamically so we can support multiple versions of Raspberry Pi.

uintptr_t GetPeripheralsPhysicalBase();

}
// namespace BootLib::Mmio
