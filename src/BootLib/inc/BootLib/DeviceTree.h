#pragma once

#include <BootLib/StreamOut.h>

#include <stdint.h>
#include <stddef.h>

#include <string_view>

namespace BootLib::DeviceTree
{

enum class MemoryType : uint8_t
{
    Invalid,
    Normal,
    Device,
};

struct MemoryRange
{
    uintptr_t  base;
    size_t     size;
    MemoryType type;
};

enum class CpuWakeupMethod : uint8_t
{
    Invalid,
    Psci,
    RPi,
};

struct Cpu
{
    uint8_t id;
};

struct Device
{
    std::string_view name;
    std::string_view compatible;
};

extern MemoryRange memoryRanges[16];
extern size_t      memoryRangeCount;

extern Cpu      cpus[16];
extern size_t   cpuCount;

void ParseDeviceTree(uintptr_t dtb, Stream::Out const& log);

}
// namespace BootLib::DeviceTree
