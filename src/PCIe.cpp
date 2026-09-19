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


void Configuration::SetCommand(std::uint16_t command) noexcept {
    header_.Command = command;
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
    std::uint16_t status_result = header_.Status;
    if (status_result == 0xFFFFu || !(status_result & 0x10))
    {
        co_return;
    }

    auto* asBytes = reinterpret_cast<std::byte*>(&Common());

    // Simulate some common capabilities
    uint8_t offset = Common().CapabilitiesPtr;
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

uintptr_t NextUnusedMapOffsetAddress = 0;

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

void Configuration::enable_device()
{
    // Enable memory and I/O space access, bus mastering
    header_.Command |= 0x07;
}

void Configuration::disable_device()
{
    // Disable memory space, I/O space, and bus mastering
    header_.Command &= ~0x07;
}

}
// namespace PCIe
