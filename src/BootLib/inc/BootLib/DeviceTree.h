#pragma once

#include <BootLib/ArrayVector.h>
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

constexpr std::string_view GetName(MemoryType const type)
{
    switch (type)
    {
    case MemoryType::Invalid: return "Invalid";
    case MemoryType::Normal:  return "Normal";
    case MemoryType::Device:  return "Device";
    default:                  return "Unknown";
    }
}

struct MemoryRange
{
    uintptr_t  base;
    size_t     size;
    MemoryType type;
};

enum class Model : uint8_t
{
    Invalid,
    RaspberryPi3B,
    RaspberryPi4B,
    QemuVirtual,
};

constexpr std::string_view GetName(Model const model)
{
    switch (model)
    {
    case Model::Invalid:       return "Invalid";
    case Model::RaspberryPi3B: return "RaspberryPi3B";
    case Model::RaspberryPi4B: return "RaspberryPi4B";
    case Model::QemuVirtual:   return "QemuVirtual";
    default:                   return "Unknown";
    }
}

enum class CpuWakeupMethod : uint8_t
{
    Invalid,
    PsciHvc,
    PsciSmc,
    RPi,
};

constexpr std::string_view GetName(CpuWakeupMethod const method)
{
    switch (method)
    {
    case CpuWakeupMethod::Invalid: return "Invalid";
    case CpuWakeupMethod::PsciHvc: return "PsciHvc";
    case CpuWakeupMethod::PsciSmc: return "PsciSmc";
    case CpuWakeupMethod::RPi:     return "RPi";
    default:                       return "Unknown";
    }
}

struct Cpu
{
    uint8_t id;
};

struct DeviceMemoryRange
{
    MemoryRange range;
    uintptr_t   deviceAddress;
    uintptr_t   dmaAddress;
};

struct Device
{
    std::string_view name;
    std::string_view compatible; // Note: nul-separated list
    uint32_t         phandle;

    ArrayVector<DeviceMemoryRange, 4> mmio;
};

extern Model           model;
extern CpuWakeupMethod cpuWakeupMethod;

extern ArrayVector<MemoryRange,   16> memoryRanges;
extern ArrayVector<Cpu        ,   16> cpus;
extern ArrayVector<Device     , 1024> devices;

void ParseDeviceTree(uintptr_t dtb, Stream::Out const& log);

inline void Dump(Stream::Out const& log)
{
    using namespace Stream;

    Puts(log, "DeviceTree Info:\n");
    Puts(log, "  Model: "); Puts(log, GetName(model)); Puts(log, "\n");
    Puts(log, "  CPU Wakeup Method: "); Puts(log, GetName(cpuWakeupMethod)); Puts(log, "\n");

    Puts(log, "  CPUs ("); PutDec(log, cpus.size()); Puts(log, "):\n");
    for (auto& cpu : cpus)
    {
        Puts(log, "    ID: "); PutDec(log, cpu.id); Puts(log, "\n");
    }

    Puts(log, "  Memory Ranges ("); PutDec(log, memoryRanges.size()); Puts(log, "):\n");
    for (auto& range : memoryRanges)
    {
        Puts(log, "    Base: "); PutHex(log, range.base);
        Puts(log, ", Size: "); PutHex(log, range.size);
        Puts(log, ", Type: "); Puts(log, GetName(range.type));
        Puts(log, "\n");
    }

    Puts(log, "  Devices ("); PutDec(log, devices.size()); Puts(log, "):\n");
    for (auto& device : devices)
    {
        Puts(log, "    Name: "); Puts(log, device.name); Puts(log, ", phandle: "); PutHex(log, device.phandle); Puts(log, "\n");
        for (auto& mmio : device.mmio)
        {
            Puts(log, "        MMIO Base: "); PutHex(log, mmio.range.base);
            Puts(log, ", Size: "); PutHex(log, mmio.range.size);
            Puts(log, ", Bus: "); PutHex(log, mmio.deviceAddress);
            Puts(log, ", DMA: "); PutHex(log, mmio.dmaAddress);
            Puts(log, ", Type: "); Puts(log, GetName(mmio.range.type));
            Puts(log, "\n");
        }
    }
}

}
// namespace BootLib::DeviceTree
