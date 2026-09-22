
#include "PCIe.h"

#include "Cpu.h"
#include "Mmio.h"
#include "Uart.h"
#include "Timer.h"
#include "Mailbox.h"
#include "emb-stdio.h"

#include <fmt/format.h>

#include <stdint.h>
#include <type_traits>
#include <utility>

namespace PCIe
{

struct GenericDriver : public Driver
{
    GenericDriver(PhysicalAddress mmioBase, PhysicalAddress memBase, uintptr_t pciBaseAddress, size_t memSize);

    Configuration ConfigureDevice(DeviceAddress addr) noexcept override;

    std::generator<ExtendedCapability> EnumerateExtendedCapabilities() const noexcept override;
    
    std::generator<DeviceInfo> EnumerateDevices() const noexcept override
    {
        for (auto& device : devices_)
        {
            co_yield device;
        }
    }

    union Registers;

    std::atomic<PCIeError> initError_ = PCIeError::DRIVER_NOT_INITIALIZED;

    BridgeRegisters& bridgeRegisters;

    ConfigHeader1*               rootHeader_       = nullptr; // Pointer to the root device's configuration header if present
    PcieCapabilities*            pcieCapabilities_ = nullptr; // Pointer to the root device's PCIe capabilities structure if present
    PowerManagementCapabilities* pmCapabilities_   = nullptr; // Pointer to the root device's Power Management capabilities structure if present

    std::shared_mutex       driver_mutex_;
    DeviceInfo              rootDeviceInfo_{};
    std::vector<DeviceInfo> devices_;
};

GenericDriver::GenericDriver(PhysicalAddress mmioBase, PhysicalAddress memBase, uintptr_t pciBaseAddress, size_t memSize)
    : Driver(pciBaseAddress, memBase)
    , bridgeRegisters{ *reinterpret_cast<BridgeRegisters*>(mmioBase) }
{
    std::lock_guard<std::shared_mutex> lock(driver_mutex_);

    fmt::println("PCIe registers base is 0x{:X}...", reinterpret_cast<std::uint64_t>(&bridgeRegisters));
    fmt::println("Initializing PCIe driver...");

    // Device on 0:0:0 should be a bridge.
    std::uint16_t const vid = bridgeRegisters.BridgeConfig.Common.VendorId;
    //if (vid != 0x14e4) // Broadcom vendor ID
    //{
    //    fmt::println("PCIe bridge not found (VID={:X})", vid);
    //    initError_.store(PCIeError::DEVICE_NOT_FOUND);
    //    return;
    //}
    fmt::println("PCIe bridge found (VID={:X})", vid);

    // The HW is enabled. Now we must set up the root bridge.

    // Bus 0 has only one device (the root complex).
    rootDeviceInfo_.Address   = {};
    auto rootConfig = this->ConfigureDevice(rootDeviceInfo_.Address);
    rootDeviceInfo_.VendorId  = rootConfig.GetVendorId();
    rootDeviceInfo_.DeviceId  = rootConfig.GetDeviceId();
    rootDeviceInfo_.ClassCode = rootConfig.GetClassCode();

    // Verify we can access the root complex
    if (rootDeviceInfo_.VendorId == 0 || rootDeviceInfo_.VendorId == 0xFFFF)
    {
        initError_.store(PCIeError::DEVICE_NOT_FOUND);
        return;
    }

    rootHeader_ = &rootConfig.Header1();

    // Pretty print root complex information
    fmt::println("PCIe Device 00:00.00");
    fmt::println("  Vendor ID: 0x{:04X}", rootDeviceInfo_.VendorId);
    fmt::println("  Device ID: 0x{:04X}", rootDeviceInfo_.DeviceId);
    fmt::println("  Class:     0x{:06X} ({})", rootDeviceInfo_.ClassCode, utils::GetClassCode_to_string(rootDeviceInfo_.ClassCode).data());

    // Get and display additional information if available
    {
        std::uint16_t command = rootHeader_->Common.Command;
        std::uint16_t status  = rootHeader_->Common.Status;
        fmt::println("  Command:   0x{:04X}", command);
        fmt::println("  Status:    0x{:04X}", status);
    }

    {
        uint8_t  const primaryBus           = rootHeader_->PrimaryBus;
        uint8_t  const secondaryBus         = rootHeader_->SecondaryBus;
        uint8_t  const subordinateBus       = rootHeader_->SubordinateBus;
        uint8_t  const legacyLatencyTimer   = rootHeader_->SecondaryLatencyTimer;
        uint8_t  const ioBase               = rootHeader_->IOBaseLo  + (static_cast<uint32_t>(rootHeader_->IOBaseHi ) << 8);
        uint8_t  const ioLimit              = rootHeader_->IOLimitLo + (static_cast<uint32_t>(rootHeader_->IOLimitHi) << 8);
        uint16_t const secondaryStatus      = rootHeader_->SecondaryStatus;
        uint32_t const memoryBase           = static_cast<uint32_t>(rootHeader_->NPMemBase  & 0xFFF0u) << 16;
        uint32_t const memoryLimit          = static_cast<uint32_t>(rootHeader_->NPMemLimit & 0xFFF0u) << 16;
        uint32_t const prefetchableMemBase  = (static_cast<uint64_t>(rootHeader_->PMemBaseLo  & 0xFFFFu) << 16) + (static_cast<uint64_t>(rootHeader_->PMemBaseHi ) << 32);
        uint32_t const prefetchableMemLimit = (static_cast<uint64_t>(rootHeader_->PMemLimitLo & 0xFFFFu) << 16) + (static_cast<uint64_t>(rootHeader_->PMemLimitHi) << 32);
        uint8_t  const interruptLine        = rootHeader_->Common.InterruptLine;
        uint8_t  const interruptPin         = rootHeader_->Common.InterruptPin;
        uint16_t const bridgeControl        = rootHeader_->BridgeControl;

        fmt::println("Bridge command               : 0x{:X}", rootHeader_->Common.Command.get());
        fmt::println("Bridge status                : 0x{:X}", rootHeader_->Common.Status.get());
        fmt::println("Primary Bus Number           : 0x{:X}", primaryBus);
        fmt::println("Secondary Bus Number         : 0x{:X}", secondaryBus);
        fmt::println("Subordinate Bus Number       : 0x{:X}", subordinateBus);
        fmt::println("Legacy Latency Timer         : 0x{:X}", legacyLatencyTimer);
        fmt::println("I/O Base                     : 0x{:X}", ioBase);
        fmt::println("I/O Limit                    : 0x{:X}", ioLimit);
        fmt::println("Secondary Status             : 0x{:X}", secondaryStatus);
        fmt::println("Memory Base                  : 0x{:X}", memoryBase);
        fmt::println("Memory Limit                 : 0x{:X}", memoryLimit);
        fmt::println("Prefetchable Memory Base     : 0x{:X}", prefetchableMemBase);
        fmt::println("Prefetchable Memory Limit    : 0x{:X}", prefetchableMemLimit);
        fmt::println("Interrupt Line               : 0x{:X}", interruptLine);
        fmt::println("Interrupt Pin                : 0x{:X}", interruptPin);
        fmt::println("Bridge Control               : 0x{:X}", bridgeControl);
    }

    rootHeader_->Common.Command |= 6; // Enable memory space (bit 1) and bus mastering (bit 2)

    rootHeader_->Common.CacheLineSize  = 64 / 4; // 16??
    rootHeader_->SecondaryBus   = 1; // Device numbers.
    rootHeader_->SubordinateBus = 1;
    rootHeader_->NPMemBase      = 0x0010; // static_cast<std::uint16_t>(pciBaseAddress  >> 16) & 0xFFF0;
    rootHeader_->NPMemLimit     = 0x0000; // static_cast<std::uint16_t>((pciBaseAddress + memSize - 1) >> 16) & 0xFFF0;
    rootHeader_->BridgeControl  = 0; // !Parity

    fmt::println("  BARs:");
    for (auto&& bar : rootConfig.EnumerateBars())
    {
        PrintBar(bar);
    }

    for (auto&& cap : rootConfig.EnumerateCapabilities())
    {
        PrintCapability(cap, rootHeader_->Common);
        if      (cap.Id == CapabilityId::Pcie           ) pcieCapabilities_ = &cap.GetStruct<PcieCapabilities           >(rootHeader_->Common);
        else if (cap.Id == CapabilityId::PowerManagement) pmCapabilities_   = &cap.GetStruct<PowerManagementCapabilities>(rootHeader_->Common);
    }

    for (auto&& cap : EnumerateExtendedCapabilities())
    {
        PrintExtendedCapability(cap, rootHeader_->Common);
    }

    {
        uint8_t  const primaryBus           = rootHeader_->PrimaryBus;
        uint8_t  const secondaryBus         = rootHeader_->SecondaryBus;
        uint8_t  const subordinateBus       = rootHeader_->SubordinateBus;
        uint8_t  const legacyLatencyTimer   = rootHeader_->SecondaryLatencyTimer;
        uint8_t  const ioBase               = rootHeader_->IOBaseLo  + (static_cast<uint32_t>(rootHeader_->IOBaseHi ) << 8);
        uint8_t  const ioLimit              = rootHeader_->IOLimitLo + (static_cast<uint32_t>(rootHeader_->IOLimitHi) << 8);
        uint16_t const secondaryStatus      = rootHeader_->SecondaryStatus;
        uint32_t const memoryBase           = static_cast<uint32_t>(rootHeader_->NPMemBase  & 0xFFF0u) << 16;
        uint32_t const memoryLimit          = static_cast<uint32_t>(rootHeader_->NPMemLimit & 0xFFF0u) << 16;
        uint32_t const prefetchableMemBase  = (static_cast<uint64_t>(rootHeader_->PMemBaseLo  & 0xFFFFu) << 16) + (static_cast<uint64_t>(rootHeader_->PMemBaseHi ) << 32);
        uint32_t const prefetchableMemLimit = (static_cast<uint64_t>(rootHeader_->PMemLimitLo & 0xFFFFu) << 16) + (static_cast<uint64_t>(rootHeader_->PMemLimitHi) << 32);
        uint8_t  const interruptLine        = rootHeader_->Common.InterruptLine;
        uint8_t  const interruptPin         = rootHeader_->Common.InterruptPin;
        uint16_t const bridgeControl        = rootHeader_->BridgeControl;

        fmt::println("Bridge command               : 0x{:X}", rootHeader_->Common.Command.get());
        fmt::println("Bridge status                : 0x{:X}", rootHeader_->Common.Status.get());
        fmt::println("Primary Bus Number           : 0x{:X}", primaryBus);
        fmt::println("Secondary Bus Number         : 0x{:X}", secondaryBus);
        fmt::println("Subordinate Bus Number       : 0x{:X}", subordinateBus);
        fmt::println("Legacy Latency Timer         : 0x{:X}", legacyLatencyTimer);
        fmt::println("I/O Base                     : 0x{:X}", ioBase);
        fmt::println("I/O Limit                    : 0x{:X}", ioLimit);
        fmt::println("Secondary Status             : 0x{:X}", secondaryStatus);
        fmt::println("Memory Base                  : 0x{:X}", memoryBase);
        fmt::println("Memory Limit                 : 0x{:X}", memoryLimit);
        fmt::println("Prefetchable Memory Base     : 0x{:X}", prefetchableMemBase);
        fmt::println("Prefetchable Memory Limit    : 0x{:X}", prefetchableMemLimit);
        fmt::println("Interrupt Line               : 0x{:X}", interruptLine);
        fmt::println("Interrupt Pin                : 0x{:X}", interruptPin);
        fmt::println("Bridge Control               : 0x{:X}", bridgeControl);
    }

    if (pcieCapabilities_)
    {
        pcieCapabilities_->RootControl = 0x0010; // CRS Software Visibility Enable
    }

    rootHeader_->Common.Command = 0x106; // !IO, Memory, Master, SERR, !Parity

    {
        uint8_t  const primaryBus           = rootHeader_->PrimaryBus;
        uint8_t  const secondaryBus         = rootHeader_->SecondaryBus;
        uint8_t  const subordinateBus       = rootHeader_->SubordinateBus;
        uint8_t  const legacyLatencyTimer   = rootHeader_->SecondaryLatencyTimer;
        uint8_t  const ioBase               = rootHeader_->IOBaseLo  + (static_cast<uint32_t>(rootHeader_->IOBaseHi ) << 8);
        uint8_t  const ioLimit              = rootHeader_->IOLimitLo + (static_cast<uint32_t>(rootHeader_->IOLimitHi) << 8);
        uint16_t const secondaryStatus      = rootHeader_->SecondaryStatus;
        uint32_t const memoryBase           = static_cast<uint32_t>(rootHeader_->NPMemBase  & 0xFFF0u) << 16;
        uint32_t const memoryLimit          = static_cast<uint32_t>(rootHeader_->NPMemLimit & 0xFFF0u) << 16;
        uint32_t const prefetchableMemBase  = (static_cast<uint64_t>(rootHeader_->PMemBaseLo  & 0xFFFFu) << 16) + (static_cast<uint64_t>(rootHeader_->PMemBaseHi ) << 32);
        uint32_t const prefetchableMemLimit = (static_cast<uint64_t>(rootHeader_->PMemLimitLo & 0xFFFFu) << 16) + (static_cast<uint64_t>(rootHeader_->PMemLimitHi) << 32);
        uint8_t  const interruptLine        = rootHeader_->Common.InterruptLine;
        uint8_t  const interruptPin         = rootHeader_->Common.InterruptPin;
        uint16_t const bridgeControl        = rootHeader_->BridgeControl;

        fmt::println("Bridge command               : 0x{:X}", rootHeader_->Common.Command.get());
        fmt::println("Bridge status                : 0x{:X}", rootHeader_->Common.Status.get());
        fmt::println("Primary Bus Number           : 0x{:X}", primaryBus);
        fmt::println("Secondary Bus Number         : 0x{:X}", secondaryBus);
        fmt::println("Subordinate Bus Number       : 0x{:X}", subordinateBus);
        fmt::println("Legacy Latency Timer         : 0x{:X}", legacyLatencyTimer);
        fmt::println("I/O Base                     : 0x{:X}", ioBase);
        fmt::println("I/O Limit                    : 0x{:X}", ioLimit);
        fmt::println("Secondary Status             : 0x{:X}", secondaryStatus);
        fmt::println("Memory Base                  : 0x{:X}", memoryBase);
        fmt::println("Memory Limit                 : 0x{:X}", memoryLimit);
        fmt::println("Prefetchable Memory Base     : 0x{:X}", prefetchableMemBase);
        fmt::println("Prefetchable Memory Limit    : 0x{:X}", prefetchableMemLimit);
        fmt::println("Interrupt Line               : 0x{:X}", interruptLine);
        fmt::println("Interrupt Pin                : 0x{:X}", interruptPin);
        fmt::println("Bridge Control               : 0x{:X}", bridgeControl);
    }

    // Real PCIe device enumeration for Raspberry Pi 4
    
    // Scan all possible device locations on the PCIe bus
    // On Rpi4, we typically see devices on bus 1 (downstream from the root complex)
    for (BusNumber bus = 0; bus <= 1; ++bus)
    {
        // Not using constants::MAX_DEVICES_PER_BUS because there's only one device per bus on Rpi4
        // Unless you're using a PCIe expansion board, which we're not.
        // Accessing the VID of devices that are not present can result in delays of seconds (timeout request in the bus).
        uint32_t deviceCount = 4;
        for (DeviceNumber device = 0; device < deviceCount; ++device) {
            if (bus == 0 && device == 0)
            {
                continue;
            }
            for (FunctionNumber function = 0; function < constants::MAX_FUNCTIONS_PER_DEVICE; ++function) {
                DeviceAddress const addr{ .Function = function, .Device = device, .Bus = bus };
                Configuration config = this->ConfigureDevice(addr);

                fmt::println("Scanning device at {:02}:{:02}.{}", bus, device, function);
                
                // Read vendor ID to check if device exists
                auto vendor = config.GetVendorId();
                if (vendor == 0 || vendor == 0xFFFF) {
                    // No device at this location, skip to next device if function 0 doesn't exist
                    if (function == 0) {
                        break;  // No function 0 means no device at this slot
                    }
                    continue;
                }

                auto GetClassCode = config.GetClassCode();

                // Device exists, add it to the list
                devices_.push_back(DeviceInfo{
                    .Address   = addr,
                    .VendorId  = vendor,
                    .DeviceId  = config.GetDeviceId(),
                    .ClassCode = GetClassCode
                });

                // Check if this is a multi-function device
                if (function == 0) {
                    uint8_t header_type = config.Common().HeaderType;
                    if (!(header_type & 0x80)) {
                        // Not a multi-function device, skip other functions
                        break;
                    }
                }

                // For bridges, we might need to scan secondary buses
                if (utils::is_bridge_device(GetClassCode)) {
                    // This is a bridge device - in a full implementation, we would
                    // read the secondary bus number and scan it recursively
                    // For now, we'll just note that it's a bridge
                }
            }
        }
    }

    initError_.store(PCIeError::SUCCESS);
}

Configuration GenericDriver::ConfigureDevice(DeviceAddress addr) noexcept
{
    uintptr_t offset = addr.Bus << 20 | addr.Device << 15 | addr.Function << 12; // ECAM
    return Configuration(addr, *reinterpret_cast<CommonConfigHeader*>(reinterpret_cast<uintptr_t>(&bridgeRegisters.BridgeConfig.Common) + offset));
}

std::generator<ExtendedCapability> GenericDriver::EnumerateExtendedCapabilities() const noexcept
{
    // Check if device supports extended capabilities
    ExtendedCapabilityHeader header = bridgeRegisters.FirstExtendedCapabilityHeader;
    if (header.Id == ExtendedCapabilityId::Invalid)
    {
        co_return;
    }

    uint16_t offset = bridgeRegisters.FirstExtendedCapabilityHeader.GetOffset() / sizeof(ExtendedCapabilityHeader);

    // Simulate some common capabilities
    while (offset > 0 && header.Id != ExtendedCapabilityId::Invalid && header.Id != ExtendedCapabilityId{0xFFFF})
    {
        ExtendedCapability const cap{
            .Id             = header.Id,
            .Version        = header.Version,
            .OffsetInDwords = offset,
        };

        fmt::println("Extended Capability ID: {:04X}.{:1X} at offset {:04X}", std::to_underlying(cap.Id), cap.Version, offset);

        // Add to capabilities list
        co_yield cap;

        offset = header.NextPtrInDwords;
        header = reinterpret_cast<ExtendedCapabilityEntry*>(reinterpret_cast<ExtendedCapabilityHeader*>(&bridgeRegisters) + offset)->Header.get();
    }

    //return capabilities;
}

std::shared_ptr<Driver> CreateGenericDriver(PhysicalAddress mmioBase, PhysicalAddress memBase, uintptr_t pciBaseAddress, size_t memSize)
{
    return std::make_shared<GenericDriver>(mmioBase, memBase, pciBaseAddress, memSize);
}

}
// namespace PCIe
