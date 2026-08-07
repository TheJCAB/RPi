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
#include <utility>

namespace PCIe
{

union ClassAndRevision
{
    struct
    {
        uint32_t RevisionId :  8;
        uint32_t ClassCode  : 24;
    };
    uint32_t Raw32;
};

union CommonConfigHeader
{
    Mmio::Register<uint16_t         const, 0x00> VendorId;
    Mmio::Register<uint16_t         const, 0x02> DeviceId;
    Mmio::Register<uint16_t              , 0x04> Command; 
    Mmio::Register<uint16_t         const, 0x06> Status;
    Mmio::Register<ClassAndRevision const, 0x08> Class;
    Mmio::Register<uint8_t               , 0x0C> CacheLineSize;
    Mmio::Register<uint8_t               , 0x0D> MasterLatencyTimer;
    Mmio::Register<uint8_t               , 0x0E> HeaderType;
    Mmio::Register<uint8_t               , 0x0F> BIST;
    Mmio::Register<uint8_t          const, 0x34> CapabilitiesPtr;
    Mmio::Register<uint8_t               , 0x3C> InterruptLine;
    Mmio::Register<uint8_t               , 0x3D> InterruptPin;
};

union ConfigHeader0 // Endpoint device header
{
    CommonConfigHeader Common;

    Mmio::RegisterArray<uint32_t         , 0x10, 6> BAR;
    Mmio::Register     <uint16_t    const, 0x2C>    SystemVendorId;
    Mmio::Register     <uint16_t    const, 0x2E>    SubsystemId;
    Mmio::Register     <uint8_t          , 0x3E>    MinGnt;
    Mmio::Register     <uint8_t          , 0x3F>    MaxLat;
};

union ConfigHeader1 // Bridge device header
{
    CommonConfigHeader Common;

    Mmio::RegisterArray<uint32_t         , 0x10, 2> BAR;
    Mmio::Register     <uint8_t     const, 0x18>    PrimaryBus;
    Mmio::Register     <uint8_t          , 0x19>    SecondaryBus;
    Mmio::Register     <uint8_t          , 0x1A>    SubordinateBus;
    Mmio::Register     <uint8_t     const, 0x1B>    SecondaryLatencyTimer;
    Mmio::Register     <uint8_t     const, 0x1C>    IOBaseLo;
    Mmio::Register     <uint8_t     const, 0x1D>    IOLimitLo;
    Mmio::Register     <uint16_t    const, 0x1E>    SecondaryStatus;
    Mmio::Register     <uint16_t         , 0x20>    NPMemBase;
    Mmio::Register     <uint16_t         , 0x22>    NPMemLimit;
    Mmio::Register     <uint16_t    const, 0x24>    PMemBaseLo;
    Mmio::Register     <uint16_t    const, 0x26>    PMemLimitLo;
    Mmio::Register     <uint32_t    const, 0x28>    PMemBaseHi;
    Mmio::Register     <uint32_t    const, 0x2C>    PMemLimitHi;
    Mmio::Register     <uint16_t    const, 0x30>    IOBaseHi;
    Mmio::Register     <uint16_t    const, 0x32>    IOLimitHi;
    Mmio::Register     <uint16_t         , 0x3E>    BridgeControl;
};

union XhciConfig
{
    CommonConfigHeader Common;
    ConfigHeader0      Header0;

    Mmio::Register<uint8_t          const, 0x60> SBRN;
    Mmio::Register<uint8_t               , 0x61> FLADJ;
    Mmio::Register<uint8_t          const, 0x62> DBES_L_LD;
};

enum class BurstSize : uint32_t
{
    Size128 = 0,
    Size256 = 1,
    Size512 = 2,
};

union MiscControl
{
    struct
    {
        uint32_t  SCB2_SIZE        : 5; // @0
        uint32_t                   : 5; // @5-9
        uint32_t  RCB_MPSPMODE     : 2; // @10-11
        uint32_t  SCB_ACCESS_EN    : 1; // @12
        uint32_t  CFG_READ_UR_MODE : 1; // @13
        uint32_t                   : 6; // @14-19
        BurstSize MAX_BURST_SIZE   : 2; // @20-21
        uint32_t  SCB1_SIZE        : 5; // @22-26
        uint32_t  SCB0_SIZE        : 5; // @27-31
    };
    uint32_t Raw32;
};

union Bcm2711Driver::Registers
{
    ConfigHeader1 BridgeConfig;

	Mmio::Register<uint32_t     , 0x0188> PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1;
    Mmio::Register<uint32_t     , 0x043C> ID;

    Mmio::Register<MiscControl  , 0x4008> MISC_CTRL;
    Mmio::Register<uint32_t     , 0x400C> MEM_PCI_LO;
    Mmio::Register<uint32_t     , 0x4010> MEM_PCI_HI;
    Mmio::Register<uint32_t     , 0x402C> RC_BAR1_CONFIG_LO;
    Mmio::Register<uint32_t     , 0x4030> RC_BAR1_CONFIG_HI;
    Mmio::Register<uint32_t     , 0x4034> RC_BAR2_CONFIG_LO;
    Mmio::Register<uint32_t     , 0x4038> RC_BAR2_CONFIG_HI;
    Mmio::Register<uint32_t     , 0x403C> RC_BAR3_CONFIG_LO;
    Mmio::Register<uint32_t     , 0x4044> MSI_BAR_CONFIG_LO;
    Mmio::Register<uint32_t     , 0x4048> MSI_BAR_CONFIG_HI;
    Mmio::Register<uint32_t     , 0x404C> MSI_DATA_CONFIG;
    Mmio::Register<uint32_t     , 0x4060> EOI_CTRL;
    Mmio::Register<uint32_t     , 0x4064> PCIE_CTRL;
    Mmio::Register<uint32_t     , 0x4068> STATUS;
    Mmio::Register<uint32_t     , 0x406C> REV;
    Mmio::Register<uint32_t     , 0x4070> MEM_CPU_LO;
    Mmio::Register<uint32_t     , 0x4080> MEM_CPU_HI_START;
    Mmio::Register<uint32_t     , 0x4084> MEM_CPU_HI_END;
    Mmio::Register<uint32_t     , 0x4204> DEBUG;
    Mmio::Register<uint32_t     , 0x4300> INTR2_CPU_STATUS;
    Mmio::Register<uint32_t     , 0x4304> INTR2_CPU_SET;
    Mmio::Register<uint32_t     , 0x4308> INTR2_CPU_CLEAR;
    Mmio::Register<uint32_t     , 0x430C> INTR2_CPU_MASK_STATUS;
    Mmio::Register<uint32_t     , 0x4310> INTR2_CPU_MASK_SET;
    Mmio::Register<uint32_t     , 0x4314> INTR2_CPU_MASK_CLEAR;

    ConfigHeader0& GetDeviceConfig() { return *reinterpret_cast<ConfigHeader0*>(reinterpret_cast<uintptr_t>(this) + 0x8000); }

    Mmio::Register<DeviceAddress, 0x9000>  CFG_INDEX;
    Mmio::Register<uint32_t     , 0x9210>  INIT;
};

void PrintBar(BarInfo const& bar)
{
    printf("    BAR%d: 0x%016llx (size: 0x%x, %s%s%s)\n",
        bar.bar_number,
        static_cast<unsigned long long>(bar.physical_address),
        static_cast<unsigned int>(bar.size),
        bar.is_memory_space ? "Memory" : "I/O",
        bar.is_64bit ? ", 64-bit" : "",
        bar.is_prefetchable ? ", Prefetchable" : "");
}

void PrintCapability(Capability const& cap, CommonConfigHeader const& config)
{
    printf("    ID: 0x%02x ", cap.Id);
    switch (cap.Id)
    {
    case CapabilityId::PowerManagement:
    {
        printf("Power management capabilities\n");
        auto& capStruct = cap.GetStruct<PowerManagementCapabilities>(config);
        printf("      Capabilities: 0x%04X\n", capStruct.Capabilities.get());
        printf("      ControlStatus: 0x%04X\n", capStruct.ControlStatus.get());
        break;
    }
    case CapabilityId::Msi:
    {
        printf("Msi capabilities\n");
        auto& capStruct = cap.GetStruct<MsiCapabilities>(config);
        printf("      Capabilities:  0x%04X\n", capStruct.Capabilities.get());
        break;
    }
    case CapabilityId::Pcie:
    {
        printf("PCIe capabilities\n");
        auto& capStruct = cap.GetStruct<PcieCapabilities>(config);
        printf("      Capabilities:              0x%04X\n", capStruct.Capabilities       .get());
        printf("      Device Capabilities:   0x%08X\n"    , capStruct.DeviceCapabilities .get());
        printf("      Device Control:            0x%04X\n", capStruct.DeviceControl      .get());
        printf("      Device Status:             0x%04X\n", capStruct.DeviceStatus       .get());
        printf("      Link Capabilities:     0x%08X\n"    , capStruct.LinkCapabilities   .get());
        printf("      Link Control:              0x%04X\n", capStruct.LinkControl        .get());
        printf("      Link Status:               0x%04X\n", capStruct.LinkStatus         .get());
        printf("      Slot Capabilities:     0x%08X\n"    , capStruct.SlotCapabilities   .get());
        printf("      Slot Control:              0x%04X\n", capStruct.SlotControl        .get());
        printf("      Slot Status:               0x%04X\n", capStruct.SlotStatus         .get());
        printf("      Root Control:              0x%04X\n", capStruct.RootControl        .get());
        printf("      Root Capabilities:         0x%04X\n", capStruct.RootCapabilities   .get());
        printf("      Root Status:           0x%08X\n"    , capStruct.RootStatus         .get());
        printf("      Device Capabilities 2: 0x%08X\n"    , capStruct.DeviceCapabilities2.get());
        printf("      Device Control 2:          0x%04X\n", capStruct.DeviceControl2     .get());
        printf("      Device Status 2:           0x%04X\n", capStruct.DeviceStatus2      .get());
        printf("      Link Capabilities 2:   0x%08X\n"    , capStruct.LinkCapabilities2  .get());
        printf("      Link Control 2:            0x%04X\n", capStruct.LinkControl2       .get());
        printf("      Link Status 2:             0x%04X\n", capStruct.LinkStatus2        .get());
        printf("      Slot Capabilities 2:   0x%08X\n"    , capStruct.SlotCapabilities2  .get());
        printf("      Slot Control 2:            0x%04X\n", capStruct.SlotControl2       .get());
        printf("      Slot Status 2:             0x%04X\n", capStruct.SlotStatus2        .get());
        break;
    }
    default:
        printf("Unknown capability\n");
        break;
    }
}

namespace
{
    constexpr uint32_t RPI_PCIE_BRIDGE_OFFSET = 0;
    constexpr uint32_t RPI_PCIE_DEVICE_OFFSET = 0x8000;

    // Simplified memory mapping without system calls
    // This would need to be implemented with actual hardware access in a real system
    void* g_config_base = nullptr;
   
    // Configuration space access via ECAM (Enhanced Configuration Access Mechanism)
    constexpr PCIe::PhysicalAddress CONFIG_BASE = 0x6'0000'0000ULL;  // 24GB mark
    constexpr std::size_t CONFIG_SIZE = 0x400'0000;  // 64MB for 256 buses
    
    // Memory space for devices
    constexpr PCIe::PhysicalAddress MEM_BASE = 0x6'0000'0000ULL;
    constexpr std::size_t MEM_SIZE = 0x400'0000;
    constexpr PCIe::PhysicalAddress MEM_LIMIT = MEM_BASE + MEM_SIZE - 1;

    constexpr PCIe::PcieAddress PCI_BASE  = 0x0'F800'0000u;
    constexpr PCIe::PcieAddress PCI_LIMIT = PCI_BASE + MEM_SIZE - 1;

    // Register offsets in PCIe controller
    constexpr std::uint32_t BRIDGE_ENABLE_REG = 0x9310;
    constexpr std::uint32_t BRIDGE_ENABLE_MASK = 0x1;

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
Configuration::Configuration(Bcm2711Driver& root, DeviceAddress addr) noexcept 
    : address_(addr)
    , header_{ addr == DeviceAddress{} ? root.registers.BridgeConfig.Common : root.registers.GetDeviceConfig().Common }
{
    if (addr == DeviceAddress{})
    {
        // The bridge is always at this address.

        // root_access_mutex_.lock();
        return;
    }

//    device_access_mutex_.lock();

    static constinit DeviceAddress lastAddress = DeviceAddress{};

    // Set up the configuration space, but only when the address changes.
    // TODO: This and all other device access should be properly synchronized in a real implementation.
    if (lastAddress != addr)
    {
        lastAddress = addr;
        root.registers.CFG_INDEX = addr;
    }
}

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

VendorID Configuration::vendor_id() const noexcept
{
    return Common().VendorId;
}

DeviceID Configuration::device_id() const noexcept {
    return Common().DeviceId;
}

ClassCode Configuration::class_code() const noexcept {
    return Common().Class->ClassCode;
}

void Configuration::set_command(std::uint16_t command) noexcept {
    header_.Command = command;
}

size_t Configuration::MaxBars() const
{
    return Common().HeaderType == 0 ? 6 : 2;
}

std::generator<BarInfo> Configuration::enumerate_bars() const
{
    auto const maxBars = MaxBars();
    for (std::uint8_t bar_num = 0; bar_num < maxBars; ++bar_num)
    {
        BarInfo bar = get_bar(bar_num);
        if (bar.is_64bit) {
            // If it's a 64-bit BAR, we need to skip the next BAR
            ++bar_num;
        }
        if (bar.size > 0)
        {
            co_yield bar;
        }
    }
}

BarInfo Configuration::get_bar(std::uint8_t bar_number) const
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

    if (barInfo.is_memory_space) {
        barInfo.is_64bit = ((bar_low >> 1) & 0x3) == 0x2;
        barInfo.is_prefetchable = (bar_low & 0x8) != 0;
        barInfo.physical_address = bar_low & 0xFFFFFFF0;
        barInfo.size = ~(bar_mask & ~0xFu) + 1;

        if (barInfo.is_64bit) {
            // For 64-bit BARs, we need to read the next 32 bits
            uint32_t bar_high = bars[bar_number+1];
            barInfo.physical_address |= static_cast<PhysicalAddress>(bar_high) << 32;

            bars[bar_number+1] = 0xFFFF'FFFFu;
            uint32_t bar_high_mask = bars[bar_number+1];
            bars[bar_number+1] = bar_high; // Restore original value
            //printf("BAR%u high: high=0x%08X, mask=0x%08X\n", bar_number + 1, bar_high, bar_high_mask);
            barInfo.size = ~((static_cast<PhysicalAddress>(bar_high_mask) << 32) + (bar_mask & ~0xFu)) + 1;
        }
    } else {
        // I/O space
        barInfo.physical_address = bar_low & 0xFFFFFFFC;
        barInfo.is_64bit = false;
        barInfo.is_prefetchable = false;
        barInfo.size = ~(bar_mask & ~0x3u) + 1;
    }

    return barInfo;
}

std::generator<Capability> Configuration::enumerate_capabilities() const
{
    // Check if device supports capabilities
    std::uint16_t status_result = header_.Status;
    if (status_result == 0xFFFFu || !(status_result & 0x10)) {
        //return capabilities;
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

        printf("Capability ID: %02X at offset %02X\n", cap.Id, offset);

        // Add to capabilities list
        co_yield cap; //capabilities.push_back(cap);

        offset = entry.NextPtr;
    }

    //return capabilities;
}

std::optional<Capability> Configuration::find_capability(CapabilityId cap_id) const
{
    for (const auto& cap : enumerate_capabilities())
    {
        if (cap.Id == cap_id) {
            return cap;
        }
    }
    
    return std::nullopt;
}

uintptr_t NextUnusedMapOffsetAddress = 0;

std::span<std::byte> Configuration::map_bar(BarInfo& bar)
{
    if (!bar.is_memory_space || bar.size == 0) {
        return {};
    }

    auto const maxBars = MaxBars();

    PcieAddress pciAddress = PCI_BASE + NextUnusedMapOffsetAddress; // Base address for PCIe memory space
    Header0().BAR[bar.bar_number] = (static_cast<uint32_t>(pciAddress) & ~0xFu) | bar.flags;
    if (bar.is_64bit) {
        Header0().BAR[bar.bar_number + 1] = static_cast<uint32_t>(pciAddress >> 32);
    }

    bar.physical_address = MEM_BASE + NextUnusedMapOffsetAddress;
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

// Hardware register access for Rpi4 PCIe controller
// These addresses are based on the BCM2711 datasheet
// TODO: Mapping the registers here (or wherever) via the MMU, even in low address mode.
constexpr PhysicalAddress Rpi4_PCIE_REGS_BASE_HI = 0x4'7D50'0000;

Bcm2711Driver::Bcm2711Driver()
    : registers{ *reinterpret_cast<Bcm2711Driver::Registers*>(Rpi4_PCIE_REGS_BASE_HI + Mmio::Base - Mmio::Rpi4BaseHi) }
{
    std::lock_guard<std::shared_mutex> lock(driver_mutex_);

    printf("PCIe registers base is 0x%ll0X...\n", reinterpret_cast<std::uint64_t>(&registers));
    printf("Initializing PCIe driver...\n");

    // Reset the controller.
    registers.INIT |= 0x3u; // Assert INIT and PERST
    Cpu::DelayInMicroseconds(1000);
    registers.INIT &= ~0x2u; // Deassert INIT

    // SERDES_IDDQ -- Note: Seen no explanation/documentation for this.
    registers.DEBUG &= ~0x0800'0000u;
    Cpu::DelayInMicroseconds(100);

    std::uint32_t revision = registers.REV;
    printf("Revision=%x\n", revision);

    // Clear and mask interrupts.
    registers.INTR2_CPU_CLEAR    = 0xFFFF'FFFFu;
    registers.INTR2_CPU_MASK_SET = 0xFFFF'FFFFu;

    // Take controller out of reset.
    registers.INIT &= ~0x1u;
    Cpu::DelayInMilliseconds(100);

    // Wait for link to become active.
    {
        uint32_t status = registers.STATUS;
        for (unsigned i = 0; i < 100; i++)
        {
            if ((status & 0x30) == 0x30) { // Phy linkup & DL Active
                break;
            }
            Cpu::DelayInMicroseconds(1000);
            status = registers.STATUS;
        }

        if ((status & 0x30) != 0x30)
        {
            printf("PCIe link not ready (status=%x)\n", status);
            initError_.store(PCIeError::HARDWARE_ERROR);
            return;
        }

        if ((status & 0x80) == 0)
        {
            printf("PCIe is not in rc mode (status=%x)\n", status);
            initError_.store(PCIeError::HARDWARE_ERROR);
            return;
        }

        printf("PCIe link ready (status=%x)\n", status);
    }

    // Set up the MISC_CTRL register with appropriate values.
    registers.MISC_CTRL = [](auto& reg)
        {
            printf("MISC_CTRL before = 0x%08X\n", reg.Raw32);
            
            // Ignored for now.
            // reg.SCB0_SIZE        = ;
            // reg.SCB1_SIZE        = ;
            // reg.SCB2_SIZE        = ;
            // reg.RCB_MPSPMODE     = ;

            reg.SCB_ACCESS_EN    = 1;
            reg.CFG_READ_UR_MODE = 1;
            reg.MAX_BURST_SIZE   = BurstSize::Size128;
            
            printf("MISC_CTRL after = 0x%08X\n", reg.Raw32);
            // (reg & 0x30'3480u) | 0x3480u;
        };

    // Set up the PCI address, split into two 32-bit registers.
    registers.MEM_PCI_LO = static_cast<std::uint32_t>(PCI_BASE);
    registers.MEM_PCI_HI = static_cast<std::uint32_t>(PCI_BASE >> 32);

    // Set up the CPU addresses.
    registers.MEM_CPU_LO       = static_cast<uint32_t>(((MEM_BASE & 0xFFF0'0000u) >> 16) | (MEM_LIMIT & 0xFFF0'0000u));
    registers.MEM_CPU_HI_START = static_cast<uint32_t>(MEM_BASE >> 32);
    registers.MEM_CPU_HI_END   = static_cast<uint32_t>(MEM_LIMIT >> 32);

    // Device on 0:0:0 should be a bridge.
    std::uint16_t const vid = registers.BridgeConfig.Common.VendorId;
    if (vid != 0x14e4) // Broadcom vendor ID
    {
        printf("PCIe bridge not found (VID=%x)\n", vid);
        initError_.store(PCIeError::DEVICE_NOT_FOUND);
        return;
    }

    registers.RC_BAR2_CONFIG_LO = 15; // DMA?
    registers.RC_BAR2_CONFIG_HI = 0;

    // SCB
    registers.MISC_CTRL = [](auto& reg){ reg.SCB0_SIZE = 7u; };

    // Set the class code to PCI-to-PCI bridge (0x060400) if it's not already set.
    uint32_t ccode = registers.ID;
    printf("Class code %x\n", ccode);
    if ((ccode & 0xffffff) != 0x060400)
    {
        ccode = (ccode & ~0xffffff) | 0x060400;
        printf("Changing to %x\n", ccode);
        registers.ID = ccode;
    }

    // CLKREQ_DEBUG_ENABLE -- Note: Something about "refclk" from RC being gated.
    registers.DEBUG |= 2u;

    // The HW is enabled. Now we must set up the root bridge.

    // The Rpi4 PCIe controller is typically on bus 0, and devices appear on bus 1

    // Bus 0 has only one device (the root complex).
    rootDeviceInfo_.Address   = {};
    auto rootConfig = Configuration(*this, rootDeviceInfo_.Address);
    rootDeviceInfo_.VendorId  = rootConfig.vendor_id();
    rootDeviceInfo_.DeviceId  = rootConfig.device_id();
    rootDeviceInfo_.ClassCode = rootConfig.class_code();
    
    // Verify we can access the root complex
    if (rootDeviceInfo_.VendorId == 0 || rootDeviceInfo_.VendorId == 0xFFFF)
    {
        initError_.store(PCIeError::DEVICE_NOT_FOUND);
        return;
    }

    rootHeader_ = &rootConfig.Header1();

    // Pretty print root complex information
    printf("PCIe Device 00:00.00\n");
    printf("  Vendor ID: 0x%04x\n", rootDeviceInfo_.VendorId);
    printf("  Device ID: 0x%04x\n", rootDeviceInfo_.DeviceId);
    printf("  Class:     0x%06x (%s)\n", rootDeviceInfo_.ClassCode, utils::class_code_to_string(rootDeviceInfo_.ClassCode).data());

    // Get and display additional information if available
    {
        std::uint16_t command = rootHeader_->Common.Command;
        std::uint16_t status  = rootHeader_->Common.Status;
        printf("  Command:   0x%04x\n", command);
        printf("  Status:    0x%04x\n", status);
    }

    rootHeader_->Common.Command |= 6; // Enable memory space (bit 1) and bus mastering (bit 2)

    rootHeader_->Common.CacheLineSize  = 64 / 4; // 16??
    rootHeader_->SecondaryBus   = 1; // Device numbers.
    rootHeader_->SubordinateBus = 1;
    rootHeader_->NPMemBase      = static_cast<std::uint16_t>(PCI_BASE  >> 16) & 0xFFF0;
    rootHeader_->NPMemLimit     = static_cast<std::uint16_t>(PCI_LIMIT >> 16) & 0xFFF0;
    rootHeader_->BridgeControl  = 1; // Parity

    printf("  BARs:\n");
    for (auto&& bar : rootConfig.enumerate_bars())
    {
        PrintBar(bar);
    }

    for (auto&& cap : rootConfig.enumerate_capabilities())
    {
        PrintCapability(cap, rootHeader_->Common);
        if      (cap.Id == CapabilityId::Pcie           ) pcieCapabilities_ = &cap.GetStruct<PcieCapabilities           >(rootHeader_->Common);
        else if (cap.Id == CapabilityId::PowerManagement) pmCapabilities_   = &cap.GetStruct<PowerManagementCapabilities>(rootHeader_->Common);
    }

    if (pcieCapabilities_)
    {
        pcieCapabilities_->RootControl = 0x0010; // CRS Software Visibility Enable
    }

    rootHeader_->Common.Command = 0x107; // IO, Memory, Master, SERR

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

        printf("Bridge command               : %#X\n", rootHeader_->Common.Command.get());
        printf("Bridge status                : %#X\n", rootHeader_->Common.Status.get());
        printf("Primary Bus Number           : %#X\n", primaryBus);
        printf("Secondary Bus Number         : %#X\n", secondaryBus);
        printf("Subordinate Bus Number       : %#X\n", subordinateBus);
        printf("Legacy Latency Timer         : %#X\n", legacyLatencyTimer);
        printf("I/O Base                     : %#X\n", ioBase);
        printf("I/O Limit                    : %#X\n", ioLimit);
        printf("Secondary Status             : %#X\n", secondaryStatus);
        printf("Memory Base                  : %#X\n", memoryBase);
        printf("Memory Limit                 : %#X\n", memoryLimit);
        printf("Prefetchable Memory Base     : %#llX\n", prefetchableMemBase);
        printf("Prefetchable Memory Limit    : %#llX\n", prefetchableMemLimit);
        printf("Interrupt Line               : %#X\n", interruptLine);
        printf("Interrupt Pin                : %#X\n", interruptPin);
        printf("Bridge Control               : %#X\n", bridgeControl);
    }

    // Real PCIe device enumeration for Raspberry Pi 4
    
    // Scan all possible device locations on the PCIe bus
    // On Rpi4, we typically see devices on bus 1 (downstream from the root complex)
    for (BusNumber bus = 1; bus <= 1; ++bus)
    {
        // Not using constants::MAX_DEVICES_PER_BUS because there's only one device per bus on Rpi4
        // Unless you're using a PCIe expansion board, which we're not.
        // Accessing the VID of devices that are not present can result in delays of seconds (timeout request in the bus).
        uint32_t deviceCount = 1;
        for (DeviceNumber device = 0; device < deviceCount; ++device) {
            for (FunctionNumber function = 0; function < constants::MAX_FUNCTIONS_PER_DEVICE; ++function) {
                DeviceAddress const addr{ .Function = function, .Device = device, .Bus = bus };
                Configuration config(*this, addr);

                printf("Scanning device at %02u:%02u.%01u\n", bus, device, function);
                
                // Read vendor ID to check if device exists
                auto vendor = config.vendor_id();
                if (vendor == 0 || vendor == 0xFFFF) {
                    // No device at this location, skip to next device if function 0 doesn't exist
                    if (function == 0) {
                        break;  // No function 0 means no device at this slot
                    }
                    continue;
                }

                auto class_code = config.class_code();

                // Device exists, add it to the list
                devices_.push_back(DeviceInfo{
                    .Address   = addr,
                    .VendorId  = vendor,
                    .DeviceId  = config.device_id(),
                    .ClassCode = class_code
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
                if (utils::is_bridge_device(class_code)) {
                    // This is a bridge device - in a full implementation, we would
                    // read the secondary bus number and scan it recursively
                    // For now, we'll just note that it's a bridge
                }
            }
        }
    }

    initError_.store(PCIeError::SUCCESS);
}

//std::vector<DeviceInfo> find_devices(VendorID vendor, std::optional<DeviceID> device)
//{
//    std::vector<DeviceInfo> matching_devices;
//
//    for (const auto& info : devices_)
//    {
//        if (info.VendorId != vendor) continue;
//        
//        if (device.has_value()) {
//            if (info.DeviceId != device.value()) continue;
//        }
//
//        matching_devices.push_back(info);
//    }
//    
//    return matching_devices;
//}

//std::vector<DeviceInfo> find_devices_by_class(ClassCode class_code, std::uint32_t mask)
//{
//    std::vector<DeviceInfo> matching_devices;
//
//    for (const auto& info : devices_)
//    {
//        if ((info.ClassCode & mask) == (class_code & mask)) {
//            matching_devices.push_back(info);
//        }
//    }
//    
//    return matching_devices;
//}

// Utility functions implementation
namespace utils {
    constexpr std::string_view class_code_to_string(ClassCode class_code) noexcept {
        switch (class_code >> 16) {
            case 0x00: return "Unclassified";
            case 0x01: return "Mass Storage Controller";
            case 0x02: return "Network Controller";
            case 0x03: return "Display Controller";
            case 0x04: return "Multimedia Controller";
            case 0x05: return "Memory Controller";
            case 0x06: return "Bridge Device";
            case 0x07: return "Communication Controller";
            case 0x08: return "Generic System Peripheral";
            case 0x09: return "Input Device Controller";
            case 0x0A: return "Docking Station";
            case 0x0B: return "Processor";
            case 0x0C: return "Serial Bus Controller";
            case 0x0D: return "Wireless Controller";
            case 0x0E: return "Intelligent Controller";
            case 0x0F: return "Satellite Controller";
            case 0x10: return "Encryption Controller";
            case 0x11: return "Signal Processing Controller";
            default: return "Unknown";
        }
    }
    
    constexpr bool is_bridge_device(ClassCode class_code) noexcept {
        return (class_code >> 16) == 0x06;
    }
    
    constexpr bool is_endpoint_device(ClassCode class_code) noexcept {
        return !is_bridge_device(class_code);
    }
    
    constexpr std::size_t align_up(std::size_t value, std::size_t alignment) noexcept {
        return (value + alignment - 1) & ~(alignment - 1);
    }
    
    constexpr bool is_power_of_two(std::size_t value) noexcept {
        return value != 0 && (value & (value - 1)) == 0;
    }
}


// Example usage function (for demonstration)
namespace examples {
    
    // Example: Enumerate and display all PCIe devices
    void demonstrate_enumeration()
    {
        Bcm2711Driver root{};

        
        auto init_result = root.initError_.load();
        if (init_result != PCIeError::SUCCESS) {
            // Initialization error
            printf("PCIe init error: %u\n", init_result);
            return;
        }

        printf("Enumerating PCIe devices...\n");

        printf("PCIe driver initialized successfully.\n");

        printf("Enumerating devices...\n");
        
        printf("Found %zu PCIe devices:\n", root.devices_.size());
        
        // Process each found device
        for (const auto& info : root.devices_) {
            // Read device information
            auto addr       = info.Address;
            auto vendor     = info.VendorId;
            auto device_id  = info.DeviceId;
            auto class_code = info.ClassCode;

            // Pretty print device information
            printf("PCIe Device %02x:%02x.%x\n", addr.Bus, addr.Device, addr.Function);
            printf("  Vendor ID: 0x%04x\n", vendor);
            printf("  Device ID: 0x%04x\n", device_id);
            printf("  Class:     0x%06x (%s)\n", class_code, 
                utils::class_code_to_string(class_code).data());

            // Get and display additional information if available
            Configuration configuration{ root, info.Address };
            std::uint16_t command = configuration.Common().Command;
            std::uint16_t status  = configuration.Common().Status;
            printf("  Command:   0x%04x\n", command);
            printf("  Status:    0x%04x\n", status);

            auto& common = configuration.Common();

            if (utils::is_bridge_device(class_code))
            {
            }
            else
            {
                common.Command = 0x107; // IO, Memory, Master //, SERR

                common.CacheLineSize  = 64 / 4; // ??
            }

            // Display BARs if any and find Bar0
            BarInfo bar0{};
            for (auto&& bar : configuration.enumerate_bars())
            {
                if (bar0.size == 0)
                {
                    printf("  BARs:\n");
                    bar0 = bar;
                }
                PrintBar(bar);
            }

            // Display capabilities if any
            printf("  Capabilities:\n");
            for (const auto& cap : configuration.enumerate_capabilities())
            {
                PrintCapability(cap, configuration.Common());
            }

            if (utils::is_bridge_device(class_code))
            {
            }
            else
            {
                // TODO: Don't hardcode. Use the BAR mapping.
                printf("Word0: %08X\n", *(uint32_t*)CONFIG_BASE);
                
                // Enable BAR 0 at the beginning of PCIe aperture
                printf("    Configuring BAR 0: CPU=0x%016llx Size = 0x%zX\n", bar0.physical_address, bar0.size);
                if (bar0.size == 0)
                {
                    printf("    BAR 0 size is zero. Halting...\n");
                    Cpu::Halt();
                }
                
                auto bar0Memory = configuration.map_bar(bar0);

                printf("    Configured BAR 0: CPU=0x%016llx Size = 0x%zX\n", bar0.physical_address, bar0.size);
                
                // Verify the BAR was written correctly
                //auto const rebar0 = configuration.get_bar(0);

                //printf("    BAR 0 readback: CPU=0x%016llx\n", rebar0.physical_address);

                // Add a delay to ensure the configuration takes effect
                Cpu::DelayInMicroseconds(100'000);

                asm volatile("dsb sy" : : : "memory");  // ARM64


                printf("    Command: 0x%08x  Status: 0x%08x\n", *reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x20), *reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x24));
                {
                    auto const timeoutTime = Cpu::GetPerformanceCounter() + Cpu::GetPerformanceTicksForMs(10'000);
                    while ((*reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x24) & (1u << 11)) && Cpu::GetPerformanceCounter() < timeoutTime)
                    {
                        Cpu::Yield();
                    }
                    if (*reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x24) & (1u << 11))
                    {
                        printf("XHCI didn't become ready.\n");
                        printf("    Command: 0x%08x  Status: 0x%08x\n", *reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x20), *reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x24));
                        Cpu::Halt();
                    }
                }

                // Test memory access
                printf("    Testing memory access at 0x%016llx...\n", bar0.physical_address);
                volatile uint32_t* test_ptr = reinterpret_cast<volatile uint32_t*>(bar0.physical_address);
                uint32_t test_value = *test_ptr;
                printf("    First word: 0x%08x\n", test_value);
                printf("    Command: 0x%08x  Status: 0x%08x\n", *reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x20), *reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x24));

                printf("    Testing memory access again at 0x%016llx...\n", bar0.physical_address);
                printf("    First word: 0x%08x\n", *test_ptr);
                printf("    Command: 0x%08x  Status: 0x%08x\n", *reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x20), *reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x24));

                for (int i = 0; i * 4 < 0xB4; ++i)
                {
                    printf("    PCIe [0x%03X] 0x%08X\n", i * 4, reinterpret_cast<uint32_t const*>(&configuration.Header0())[i]);
                }

                for (int i = 0; i < 64 && i * 4 < bar0.size; ++i)
                {
                    printf("    xHCI [0x%03X] 0x%08X\n", i * 4, test_ptr[i]);
                }
                for (int i = 0x420 / 4; i < 0x440 / 4; ++i)
                {
                    printf("    xHCI [0x%03X] 0x%08X\n", i * 4, reinterpret_cast<uint32_t const volatile*>(bar0.physical_address)[i]);
                }

                printf("    Resetting...\n");
                *reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x20) |= 1u << 1;
                {
                    auto const timeoutTime = Cpu::GetPerformanceCounter() + Cpu::GetPerformanceTicksForMs(10'000);
                    while ((*reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x20) & (1u << 1)) && Cpu::GetPerformanceCounter() < timeoutTime)
                    {
                        Cpu::Yield();
                    }
                }

                for (int i = 0; i * 4 < 0xB4; ++i)
                {
                    printf("    PCIe [0x%03X] 0x%08X\n", i * 4, reinterpret_cast<uint32_t const*>(&configuration.Header0())[i]);
                }

                for (int i = 0; i < 64 && i * 4 < bar0.size; ++i)
                {
                    printf("    xHCI [0x%03X] 0x%08X\n", i * 4, test_ptr[i]);
                }
                for (int i = 0x420 / 4; i < 0x440 / 4; ++i)
                {
                    printf("    xHCI [0x%03X] 0x%08X\n", i * 4, reinterpret_cast<uint32_t const volatile*>(bar0.physical_address)[i]);
                }

                //// Enable USB controller power via mailbox
                while (*reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x24) & 0x800u)
                {
                    printf("    Status: 0x%08x\n", *reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x24));

                    printf("    Enabling USB controller power...\n");
                    Mailbox::TagMessage<Mailbox::Tag::RPI4_PCIE_XHCI_USB_RESET, 1> resetTag{{ 1u << 20 }};
                    if (!Mailbox::SendTags(resetTag)) {
                        printf("    ✗ Failed to enable USB controller power\n");
                    }
                    else
                    {
                        printf("    New state: %u\n", resetTag.args[0]);
                    }

                    auto const timeoutTime = Cpu::GetPerformanceCounter() + Cpu::GetPerformanceTicksForMs(10'000);
                    while ((*reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x24) & 0x800u) &&
                           (Cpu::GetPerformanceCounter() < timeoutTime))
                    {
                        Cpu::Yield();
                    }

                    for (int i = 0; i * 4 < 0xB4; ++i)
                    {
                        printf("    PCIe [0x%03X] 0x%08X\n", i * 4, reinterpret_cast<uint32_t const*>(&configuration.Header0())[i]);
                    }

                    for (int i = 0; i < 64 && i * 4 < bar0.size; ++i)
                    {
                        printf("    xHCI [0x%03X] 0x%08X\n", i * 4, reinterpret_cast<uint32_t const volatile*>(bar0.physical_address)[i]);
                    }
                    for (int i = 0x420 / 4; i < 0x440 / 4; ++i)
                    {
                        printf("    xHCI [0x%03X] 0x%08X\n", i * 4, reinterpret_cast<uint32_t const volatile*>(bar0.physical_address)[i]);
                    }

                    Cpu::Halt();
                }

                printf("    Status: 0x%08x\n", *reinterpret_cast<volatile uint32_t*>(bar0.physical_address + 0x24));

                // Verify XHCI controller presence by reading its capability registers
                if (vendor == 0x1106 && device_id == 0x3483) { // VIA VL805 USB 3.0 controller
                    printf("    Detected VL805 USB 3.0 controller\n");
                    
                    // Wait for power stabilization
                    Cpu::DelayInMicroseconds(10000); // 10ms delay
                    
                    // XHCI capability registers start at BAR 0
                    volatile uint32_t* xhci_base = reinterpret_cast<volatile uint32_t*>(0x600000000ULL);
                    
                    // Read XHCI Capability Registers
                    uint32_t caplength_hciversion = xhci_base[0x00 / 4]; // Capability Register Length and Interface Version
                    uint32_t hcsparams1 = xhci_base[0x04 / 4];           // Structural Parameters 1
                    uint32_t hcsparams2 = xhci_base[0x08 / 4];           // Structural Parameters 2
                    uint32_t hcsparams3 = xhci_base[0x0C / 4];           // Structural Parameters 3
                    uint32_t hccparams1 = xhci_base[0x10 / 4];           // Capability Parameters 1
                    
                    uint8_t cap_length = caplength_hciversion & 0xFF;
                    uint16_t hci_version = (caplength_hciversion >> 16) & 0xFFFF;
                    
                    printf("    XHCI Capability Length: 0x%02x\n", cap_length);
                    printf("    XHCI Interface Version: 0x%04x\n", hci_version);
                    printf("    Max Device Slots: %u\n", hcsparams1 & 0xFF);
                    printf("    Max Interrupters: %u\n", (hcsparams1 >> 8) & 0x7FF);
                    printf("    Max Ports: %u\n", (hcsparams1 >> 24) & 0xFF);
                    
                    // Verify this looks like a valid XHCI controller
                    if (cap_length >= 0x20 && cap_length <= 0x40 && 
                        (hci_version == 0x0100 || hci_version == 0x0110 || hci_version == 0x0120)) {
                        printf("    ✓ XHCI controller verification successful\n");
                        
                        // Read operational registers base
                        volatile uint32_t* xhci_op_base = reinterpret_cast<volatile uint32_t*>(0x600000000ULL + cap_length);
                        uint32_t usbcmd = xhci_op_base[0x00 / 4];  // USB Command register
                        uint32_t usbsts = xhci_op_base[0x04 / 4];  // USB Status register
                        
                        printf("    USB Command: 0x%08x\n", usbcmd);
                        printf("    USB Status: 0x%08x %s\n", usbsts, 
                               (usbsts & 0x1) ? "(Controller Halted)" : "(Controller Running)");
                    } else {
                        printf("    ✗ XHCI controller verification failed - invalid capability registers\n");
                    }
                }

            }

            printf("\n");
        }
    }
}

}
// namespace PCIe
