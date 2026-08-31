#pragma once

#include <stdint.h>
#include <stddef.h>

namespace BootLib::Uart { class PL011Uart; }

namespace BootLib::DeviceTree
{

struct MemoryRange
{
    uintptr_t base;
    size_t    size;
};

struct Cpu
{
    uint8_t id;
};

extern MemoryRange memoryRanges[16];
extern size_t      memoryRangeCount;

extern Cpu      cpus[16];
extern size_t   cpuCount;

void ParseDeviceTree(void* dtb, Uart::PL011Uart* log);

}
// namespace BootLib::DeviceTree
