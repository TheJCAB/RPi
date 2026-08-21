#pragma once

#include <stdint.h>
#include <stddef.h>

#include <memory>
#include <span>
#include <vector>

#include "UsbSpec.h"

namespace PCIe {
    struct Bcm2711Driver;
    struct DeviceAddress;
}

namespace Usb
{

struct DeviceInfo
{
    uint32_t SlotId = 0;
    uint32_t Port = 0;
    uint32_t RootHubPort = 0;
    uint32_t Speed = 0;
    DeviceDescriptor Descriptor{};
    ConfigurationDescriptor Configuration{};
    std::vector<UsbInterfaceDescriptor> Interfaces{};
    std::vector<std::vector<UsbEndpointDescriptor>> Endpoints{};
    bool HasConfiguration = false;
};

enum class Status {
    Success,
    Error,
    Timeout,
    NotFound
};


}
// namespace Usb