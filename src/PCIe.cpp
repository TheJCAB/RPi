/*
 * PCIe Driver Implementation for Raspberry Pi 4
 * 
 * This implementation provides a comprehensive PCIe driver interface specifically
 * designed for the Raspberry Pi 4's PCIe controller (BCM2711).
 * 
 * Key Features:
 * - Real PCIe device enumeration scanning buses 0 and 1
 * - Hardware-aware register access simulation based on Rpi4 specifications
 * - Support for common Rpi4 PCIe devices (USB 3.0 controller, etc.)
 * - Memory-mapped I/O with proper ARM64 memory barriers
 * - Interrupt handling (MSI/MSI-X) support
 * - DMA buffer management
 * 
 * Hardware Details:
 * - PCIe controller base: 0xFD500000 (BCM2711)
 * - Configuration space: ECAM at 0x600000000 (24GB mark)
 * - Typical devices: VL805 USB 3.0 controller on bus 1
 * - Root complex: Broadcom BCM2711 on bus 0, device 0
 * 
 * Note: This implementation includes simulation for development/testing.
 * For actual hardware access, the memory mapping functions would need to
 * be implemented with proper /dev/mem access or kernel driver integration.
 */

#include "PCIe.h"

#include "Cpu.h"
#include "Mmio.h"
#include "Uart.h"
#include "Timer.h"
#include "Mailbox.h"
#include "emb-stdio.h"

#include <cstring>
#include <string>
#include <memory>
#include <fmt/format.h>
#include <type_traits>
#include <utility>

namespace PCIe
{

Driver::~Driver() = default;

union XhciConfig
{
    CommonConfigHeader Common;
    ConfigHeader0      Header0;

    Mmio::Register<uint8_t          const, 0x60> SBRN;
    Mmio::Register<uint8_t               , 0x61> FLADJ;
    Mmio::Register<uint8_t          const, 0x62> DBES_L_LD;
};

void PrintBar(BarInfo const& bar)
{
    fmt::println("    BAR{}: 0x{:016X} (size: 0x{:X}, {}{}{})",
        bar.bar_number,
        static_cast<unsigned long long>(bar.physical_address),
        static_cast<unsigned int>(bar.size),
        bar.is_memory_space ? "Memory" : "I/O",
        bar.registers.Count > 1 ? ", 64-bit" : "",
        bar.is_prefetchable ? ", Prefetchable" : "");
}

void PrintExtendedCapability(ExtendedCapability const& cap, CommonConfigHeader const& config)
{
    fmt::print("    ID: 0x{:04X},{:1X} ", std::to_underlying(cap.Id), cap.Version);
}

void PrintCapability(Capability const& cap, CommonConfigHeader const& config)
{
    fmt::print("    ID: 0x{:02X} ", std::to_underlying(cap.Id));
    switch (cap.Id)
    {
    case CapabilityId::PowerManagement:
    {
        fmt::println("Power management capabilities");
        auto& capStruct = cap.GetStruct<PowerManagementCapabilities>(config);
        fmt::println("      Capabilities: 0x{:04X}", capStruct.Capabilities.get());
        fmt::println("      ControlStatus: 0x{:04X}", capStruct.ControlStatus.get());
        break;
    }
    case CapabilityId::Msi:
    {
        fmt::println("Msi capabilities");
        auto& capStruct = cap.GetStruct<MsiCapabilities>(config);
        fmt::println("      Capabilities:  0x{:04X}", capStruct.Capabilities.get());
        break;
    }
    case CapabilityId::Pcie:
    {
        fmt::println("PCIe capabilities");
        auto& capStruct = cap.GetStruct<PcieCapabilities>(config);
        fmt::println("      Capabilities:              0x{:04X}", capStruct.Capabilities       .get());
        fmt::println("      Device Capabilities:   0x{:08X}"    , capStruct.DeviceCapabilities .get());
        fmt::println("      Device Control:            0x{:04X}", capStruct.DeviceControl      .get());
        fmt::println("      Device Status:             0x{:04X}", capStruct.DeviceStatus       .get());
        fmt::println("      Link Capabilities:     0x{:08X}"    , capStruct.LinkCapabilities   .get());
        fmt::println("      Link Control:              0x{:04X}", capStruct.LinkControl        .get());
        fmt::println("      Link Status:               0x{:04X}", capStruct.LinkStatus         .get());
        fmt::println("      Slot Capabilities:     0x{:08X}"    , capStruct.SlotCapabilities   .get());
        fmt::println("      Slot Control:              0x{:04X}", capStruct.SlotControl        .get());
        fmt::println("      Slot Status:               0x{:04X}", capStruct.SlotStatus         .get());
        fmt::println("      Root Control:              0x{:04X}", capStruct.RootControl        .get());
        fmt::println("      Root Capabilities:         0x{:04X}", capStruct.RootCapabilities   .get());
        fmt::println("      Root Status:           0x{:08X}"    , capStruct.RootStatus         .get());
        fmt::println("      Device Capabilities 2: 0x{:08X}"    , capStruct.DeviceCapabilities2.get());
        fmt::println("      Device Control 2:          0x{:04X}", capStruct.DeviceControl2     .get());
        fmt::println("      Device Status 2:           0x{:04X}", capStruct.DeviceStatus2      .get());
        fmt::println("      Link Capabilities 2:   0x{:08X}"    , capStruct.LinkCapabilities2  .get());
        fmt::println("      Link Control 2:            0x{:04X}", capStruct.LinkControl2       .get());
        fmt::println("      Link Status 2:             0x{:04X}", capStruct.LinkStatus2        .get());
        fmt::println("      Slot Capabilities 2:   0x{:08X}"    , capStruct.SlotCapabilities2  .get());
        fmt::println("      Slot Control 2:            0x{:04X}", capStruct.SlotControl2       .get());
        fmt::println("      Slot Status 2:             0x{:04X}", capStruct.SlotStatus2        .get());
        break;
    }
    default:
        fmt::println("Unknown capability");
        break;
    }
}

namespace
{
    // Simple malloc-based allocator for DMA (not real DMA!)
    void* simple_alloc(std::size_t size, std::size_t alignment) {
        // Align size to alignment boundary
        std::size_t aligned_size = (size + alignment - 1) & ~(alignment - 1);
        
        // Allocate extra space for alignment
        std::size_t total_size = aligned_size + alignment - 1 + sizeof(void*);
        void* raw_ptr = new char[total_size];
        
        if (!raw_ptr) return nullptr;
        
        // Find aligned position
        uintptr_t aligned_addr = (reinterpret_cast<uintptr_t>(raw_ptr) + sizeof(void*) + alignment - 1) & ~(alignment - 1);
        void* aligned_ptr = reinterpret_cast<void*>(aligned_addr);
        
        // Store original pointer before aligned pointer
        void** original_ptr = reinterpret_cast<void**>(aligned_ptr) - 1;
        *original_ptr = raw_ptr;
        
        return aligned_ptr;
    }
    
    void simple_free(void* ptr) {
        if (!ptr) return;
        
        // Get original pointer stored before aligned pointer
        void** original_ptr = reinterpret_cast<void**>(ptr) - 1;
        void* raw_ptr = *original_ptr;
        
        delete[] static_cast<char*>(raw_ptr);
    }
}

// TODO: multicoreing?
//static std::mutex access_mutex_;

// Configuration class implementation

Configuration::~Configuration()
{
    if (address_ == InvalidDeviceAddress)
    {
        return;
    }
    else if (address_ == DeviceAddress{})
    {
//        root_access_mutex_.unlock();
    }
    else
    {
//        device_access_mutex_.unlock();
    }
}


size_t Configuration::MaxBars() const
{
    return Common().HeaderType == 0 ? 6 : 2;
}

std::generator<BarInfo> Configuration::EnumerateBars() const
{
    auto const maxBars = MaxBars();
    for (std::uint8_t bar_num = 0; bar_num < maxBars; ++bar_num)
    {
        BarInfo bar = GetBar(bar_num);
        if (bar.registers.Count > 1) {
            // If it's a 64-bit BAR, we need to skip the next BAR
            ++bar_num;
        }
        if (bar.size > 0)
        {
            co_yield bar;
        }
    }
}

BarInfo Configuration::GetBar(std::uint8_t bar_number) const
{
    if (bar_number >= MaxBars()) {
        return {};
    }
    
    auto& bars = Header0().BAR; // TODO: Fix accessing Header1 BARs via Header0.

    uint32_t bar_low = bars[bar_number];
    if (bar_low == 0 || bar_low == 0xFFFF'FFFFu) {
        return {};
    }

    // Get the size.
    bars[bar_number] = 0xFFFF'FFFFu;
    uint32_t bar_mask = bars[bar_number];
    //printf("BAR%u: low=0x%08X, mask=0x%08X\n", bar_number, bar_low, bar_mask);
    bars[bar_number] = bar_low; // Restore original value

    BarInfo barInfo{};
    barInfo.bar_number = bar_number;
    barInfo.flags = bar_low & 0xF;
    barInfo.is_memory_space = (bar_low & 0x1) == 0;

    if (barInfo.is_memory_space)
    {
        bool const is_64bit = ((bar_low >> 1) & 0x3) == 0x2;

        barInfo.is_prefetchable  = (bar_low & 0x8) != 0;
        barInfo.physical_address = bar_low & 0xFFFFFFF0;
        barInfo.size             = ~(bar_mask & ~0xFu) + 1;

        if (is_64bit)
        {
            // For 64-bit BARs, we need to read the next 32 bits
            uint32_t bar_high = bars[bar_number+1];
            barInfo.physical_address |= static_cast<PhysicalAddress>(bar_high) << 32;

            bars[bar_number+1] = 0xFFFF'FFFFu;
            uint32_t bar_high_mask = bars[bar_number+1];
            bars[bar_number+1] = bar_high; // Restore original value
            //printf("BAR%u high: high=0x%08X, mask=0x%08X\n", bar_number + 1, bar_high, bar_high_mask);
            barInfo.size = ~((static_cast<PhysicalAddress>(bar_high_mask) << 32) + (bar_mask & ~0xFu)) + 1;
            barInfo.registers = bars.SubSpan(bar_number, 2);
        }
        else
        {
            barInfo.registers = bars.SubSpan(bar_number, 1);
        }
    }
    else
    {
        // I/O space
        barInfo.physical_address = bar_low & 0xFFFFFFFC;
        barInfo.is_prefetchable = false;
        barInfo.size = ~(bar_mask & ~0x3u) + 1;
        barInfo.registers = bars.SubSpan(bar_number, 1);
    }

    return barInfo;
}

std::generator<Capability> Configuration::EnumerateCapabilities() const
{
    // Check if device supports capabilities
    std::uint16_t status_result = header_->Status;
    if (status_result == 0xFFFFu || !(status_result & 0x10))
    {
        co_return;
    }

    auto* asBytes = reinterpret_cast<std::byte*>(header_);

    // Simulate some common capabilities
    uint8_t offset = header_->CapabilitiesPtr;
    while (offset != 0x00u)
    {
        Capability cap{};
        auto& entry = *reinterpret_cast<CapabilityEntry*>(asBytes + offset);
        cap.Id = entry.Id;
        cap.Offset = offset;

        fmt::println("Capability ID: {:02X} at offset {:02X}", std::to_underlying(cap.Id), offset);

        // Add to capabilities list
        co_yield cap; //capabilities.push_back(cap);

        offset = entry.NextPtr;
    }

    //return capabilities;
}

std::optional<Capability> Configuration::FindCapability(CapabilityId cap_id) const
{
    for (const auto& cap : EnumerateCapabilities())
    {
        if (cap.Id == cap_id) {
            return cap;
        }
    }
    
    return std::nullopt;
}


std::generator<ExtendedCapability> Configuration::EnumerateExtendedCapabilities() const noexcept
{
    // Check if device supports extended capabilities
    ExtendedCapabilityEntry& entry = GetFirstExtendedCapability();
    ExtendedCapabilityHeader header = entry;
    if (header.Id == ExtendedCapabilityId::Invalid || header.Id == ExtendedCapabilityId{0xFFFF})
    {
        co_return;
    }

    uintptr_t const base = reinterpret_cast<uintptr_t>(header_);

    uint16_t offset = reinterpret_cast<uintptr_t>(&entry) - base;

    // Simulate some common capabilities
    do
    {
        ExtendedCapability const cap
        {
            .Id      = header.Id,
            .Version = header.Version,
            .Offset  = offset,
        };

        fmt::println("Extended Capability ID: {:04X}.{:1X} at offset {:04X}", std::to_underlying(cap.Id), cap.Version, offset);

        // Add to capabilities list
        co_yield cap;

        offset = header.NextPtrInDwords * sizeof(ExtendedCapabilityHeader);
        header = reinterpret_cast<ExtendedCapabilityEntry*>(base + offset)->get();
    }
    while (header.NextPtrInDwords > 0 && header.Id != ExtendedCapabilityId::Invalid && header.Id == ExtendedCapabilityId{0xFFFF});
}

uintptr_t NextUnusedMapOffsetAddress = 0;

bool Driver::Initialize()
{
    fmt::println("Initializing PCIe driver...");

    auto rootConfig = this->ConfigureDevice({});
    if (!rootConfig)
    {
        return false;
    }

    auto& rootHeader = rootConfig.Common();

    // Device on 0:0:0 should be a bridge.
    VendorID const vendorId = rootHeader.VendorId;

    // Verify we can access the root complex
    if (vendorId == 0 || vendorId == 0xFFFF)
    {
        return false;
    }

    // Now we must set up the root bridge.
    DeviceInfo rootDeviceInfo_ = {};
    rootDeviceInfo_.VendorId  = rootConfig.GetVendorId();
    rootDeviceInfo_.DeviceId  = rootConfig.GetDeviceId();
    rootDeviceInfo_.ClassCode = rootConfig.GetClassCode();

    // Pretty print root complex information
    fmt::println("PCIe Device 00:00.00");
    fmt::println("  Vendor ID: 0x{:04X}", rootDeviceInfo_.VendorId);
    fmt::println("  Device ID: 0x{:04X}", rootDeviceInfo_.DeviceId);
    fmt::println("  Class:     0x{:06X} ({})", rootDeviceInfo_.ClassCode, utils::GetClassCodeDescription(rootDeviceInfo_.ClassCode).data());
    fmt::println("  Header:    0x{:02X}", rootHeader.HeaderType.get());

    // Get and display additional information if available
    {
        std::uint16_t command = rootHeader.Command;
        std::uint16_t status  = rootHeader.Status;
        fmt::println("  Command:   0x{:04X}", command);
        fmt::println("  Status:    0x{:04X}", status);
    }

    rootHeader.Command |= 6; // Enable memory space (bit 1) and bus mastering (bit 2)

    rootHeader.CacheLineSize  = 64 / 4; // 16??

    buses_.push_back(BusInfo{
        .Number        = 0,
        .SingleDevice  = rootHeader.HeaderType == 0x01,
        .BridgeAddress = InvalidDeviceAddress,
    }); // Add the root bus

    if (rootHeader.HeaderType == 0x01)
    {
        auto& header1 = rootConfig.Header1();

        BusNumber number = static_cast<BusNumber>(buses_.size());
        buses_.push_back(BusInfo{
            .Number        = number,
            .SingleDevice  = false,
            .BridgeAddress = {},
        });

        header1.SecondaryBus   = number;
        header1.SubordinateBus = number;
        header1.NPMemBase      = static_cast<std::uint16_t>(PciBaseAddress  >> 16) & 0xFFF0;        // TODO: Distinguish prefetchable from non-prefetchable memory apertures
        header1.NPMemLimit     = static_cast<std::uint16_t>((PciBaseAddress + MemSize - 1) >> 16) & 0xFFF0;
        // TODO: Handle prefetchable memory apertures separately if needed
        header1.BridgeControl  = 1; // Parity
    }

    fmt::println("  BARs:");
    for (auto&& bar : rootConfig.EnumerateBars())
    {
        PrintBar(bar);
    }

    for (auto&& cap : rootConfig.EnumerateCapabilities())
    {
        PrintCapability(cap, rootHeader);
        if      (cap.Id == CapabilityId::Pcie           ) pcieCapabilities_ = &cap.GetStruct<PcieCapabilities           >(rootHeader);
        else if (cap.Id == CapabilityId::PowerManagement) pmCapabilities_   = &cap.GetStruct<PowerManagementCapabilities>(rootHeader);
    }

    for (auto&& cap : rootConfig.EnumerateExtendedCapabilities())
    {
        PrintExtendedCapability(cap, rootHeader);
    }

    if (pcieCapabilities_)
    {
        pcieCapabilities_->RootControl = 0x0010; // CRS Software Visibility Enable
    }

    rootHeader.Command = 0x146; // !IO, Memory, Master, SERR, Parity


    // Some root bridges, like QEMU's virtual PCIe root complex, use header 0.
    // Presumably because they are not configurable.
    if (rootHeader.HeaderType == 0x01)
    {
        auto& header1 = rootConfig.Header1();
        uint8_t  const primaryBus           = header1.PrimaryBus;
        uint8_t  const secondaryBus         = header1.SecondaryBus;
        uint8_t  const subordinateBus       = header1.SubordinateBus;
        uint8_t  const legacyLatencyTimer   = header1.SecondaryLatencyTimer;
        uint8_t  const ioBase               = header1.IOBaseLo  + (static_cast<uint32_t>(header1.IOBaseHi ) << 8);
        uint8_t  const ioLimit              = header1.IOLimitLo + (static_cast<uint32_t>(header1.IOLimitHi) << 8);
        uint16_t const secondaryStatus      = header1.SecondaryStatus;
        uint32_t const memoryBase           = static_cast<uint32_t>(header1.NPMemBase  & 0xFFF0u) << 16;
        uint32_t const memoryLimit          = static_cast<uint32_t>(header1.NPMemLimit & 0xFFF0u) << 16;
        uint64_t const prefetchableMemBase  = (static_cast<uint64_t>(header1.PMemBaseLo  & 0xFFFFu) << 16) + (static_cast<uint64_t>(header1.PMemBaseHi ) << 32);
        uint64_t const prefetchableMemLimit = (static_cast<uint64_t>(header1.PMemLimitLo & 0xFFFFu) << 16) + (static_cast<uint64_t>(header1.PMemLimitHi) << 32);
        uint16_t const bridgeControl        = header1.BridgeControl;

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
        fmt::println("Bridge Control               : 0x{:X}", bridgeControl);

        if (pcieCapabilities_ != nullptr)
        {
            auto const type = (pcieCapabilities_->Capabilities >> 4) & 0xFu;
            buses_.back().SingleDevice =
                type == 4 ||    // Root Port of PCI Express Root Complex
                type == 6 ||    // Downstream Port of PCI Express Switch
                type == 8;      // PCI/PCI-X to PCI Express Bridge
        }
    }

    {
        uint8_t  const interruptLine = rootHeader.InterruptLine;
        uint8_t  const interruptPin  = rootHeader.InterruptPin;

        fmt::println("Interrupt Line               : 0x{:X}", interruptLine);
        fmt::println("Interrupt Pin                : 0x{:X}", interruptPin);
    }

    // Generic PCIe device enumeration
    
    // Scan all possible device locations on the PCIe bus
    // On Rpi4, we typically see devices on bus 1 (downstream from the root complex)
    for (auto& bus : buses_)
    {
        static constexpr uint32_t MaxDevicesPerBus = 32;
        for (DeviceNumber device = 0; device < MaxDevicesPerBus; ++device)
        {
            if (bus.Number == 0 && device == 0)
            {
                // Already handled the root complex so we skip it here.
                continue;
            }
            if (bus.SingleDevice && device > 0)
            {
                // Single-device bus.
                continue;
            }
            for (FunctionNumber function = 0; function < constants::MAX_FUNCTIONS_PER_DEVICE; ++function)
            {
                fmt::print("Scanning device at {:02}:{:02}.{}", bus.Number, device, function);

                DeviceAddress const addr{ .Function = function, .Device = device, .Bus = bus.Number };
                Configuration config = this->ConfigureDevice(addr);
                if (!config)
                {
                    fmt::println("");
                    break;
                }

                fmt::print(" VendorId: ");

                // Read vendor ID to check if device exists
                auto vendor = config.GetVendorId();
                if (vendor == 0 || vendor == 0xFFFF)
                {
                    // No device at this location, skip to next device if function 0 doesn't exist
                    fmt::println("Not found");
                    if (function == 0)
                    {
                        break;  // No function 0 means no device at this slot
                    }
                    continue;
                }

                fmt::println("{:06X}", vendor);

                auto const classCode = config.GetClassCode();
                auto const deviceId  = config.GetDeviceId();

                // Device exists, add it to the list
                devices_.push_back(DeviceInfo{
                    .Address   = addr,
                    .VendorId  = vendor,
                    .DeviceId  = deviceId,
                    .ClassCode = classCode
                });

                fmt::println("  Found - Vendor ID: 0x{:04X}, Device ID: 0x{:04X}, Class Code: 0x{:06X} {}",
                            vendor, deviceId, classCode, utils::GetClassCodeDescription(classCode));

                // Check if this is a multi-function device
                if (function == 0) {
                    uint8_t header_type = config.Common().HeaderType;
                    if (!(header_type & 0x80)) {
                        // Not a multi-function device, skip other functions
                        break;
                    }
                }

                // For bridges, we might need to scan secondary buses
                if (utils::is_bridge_device(classCode)) {
                    // This is a bridge device - in a full implementation, we would
                    // read the secondary bus number and scan it recursively
                    // For now, we'll just note that it's a bridge
                }
            }
        }
    }

    return true;
}

std::generator<DeviceInfo> Driver::EnumerateDevices() const noexcept
{
    for (auto& device : devices_)
    {
        co_yield device;
    }
}

std::span<std::byte> Driver::MapBar(BarInfo& bar)
{
    if (!bar.is_memory_space || bar.size == 0)
    {
        return {};
    }

    PcieAddress pciAddress = PciBaseAddress + NextUnusedMapOffsetAddress; // Base address for PCIe memory space
    bar.registers[0] = (static_cast<uint32_t>(pciAddress) & ~0xFu) | bar.flags;
    if (bar.registers.Count > 1)
    {
        bar.registers[1] = static_cast<uint32_t>(pciAddress >> 32);
    }

    bar.physical_address = MemBaseAddress + NextUnusedMapOffsetAddress;
    NextUnusedMapOffsetAddress += bar.size;

    // Physical and virtual addresses match by grace of page tables's initial mapping.
    // TODO: Map the physical address to virtual explicitly here?
    return { reinterpret_cast<std::byte*>(bar.physical_address), bar.size };
}

}
// namespace PCIe
