
#include "Bcm2711Pcie.h"

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

union BcmConfigRegisters
{
    Mmio::Register<MiscControl  , 0x008> MISC_CTRL;
    Mmio::Register<uint32_t     , 0x00C> MEM_PCI_LO;
    Mmio::Register<uint32_t     , 0x010> MEM_PCI_HI;
    Mmio::Register<uint32_t     , 0x02C> RC_BAR1_CONFIG_LO;
    Mmio::Register<uint32_t     , 0x030> RC_BAR1_CONFIG_HI;
    Mmio::Register<uint32_t     , 0x034> RC_BAR2_CONFIG_LO;
    Mmio::Register<uint32_t     , 0x038> RC_BAR2_CONFIG_HI;
    Mmio::Register<uint32_t     , 0x03C> RC_BAR3_CONFIG_LO;
    Mmio::Register<uint32_t     , 0x044> MSI_BAR_CONFIG_LO;
    Mmio::Register<uint32_t     , 0x048> MSI_BAR_CONFIG_HI;
    Mmio::Register<uint32_t     , 0x04C> MSI_DATA_CONFIG;
    Mmio::Register<uint32_t     , 0x060> EOI_CTRL;
    Mmio::Register<uint32_t     , 0x064> PCIE_CTRL;
    Mmio::Register<uint32_t     , 0x068> STATUS;
    Mmio::Register<uint32_t     , 0x06C> REV;
    Mmio::Register<uint32_t     , 0x070> MEM_CPU_LO;
    Mmio::Register<uint32_t     , 0x080> MEM_CPU_HI_START;
    Mmio::Register<uint32_t     , 0x084> MEM_CPU_HI_END;
    Mmio::Register<uint32_t     , 0x204> DEBUG;
    Mmio::Register<uint32_t     , 0x300> INTR2_CPU_STATUS;
    Mmio::Register<uint32_t     , 0x304> INTR2_CPU_SET;
    Mmio::Register<uint32_t     , 0x308> INTR2_CPU_CLEAR;
    Mmio::Register<uint32_t     , 0x30C> INTR2_CPU_MASK_STATUS;
    Mmio::Register<uint32_t     , 0x310> INTR2_CPU_MASK_SET;
    Mmio::Register<uint32_t     , 0x314> INTR2_CPU_MASK_CLEAR;
};


struct Bcm2711Driver : public Driver
{
    Bcm2711Driver(PhysicalAddress mmioBase);

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
    Registers&       registers;

    DeviceAddress lastConfiguredAddress{};

    ConfigHeader1*               rootHeader_       = nullptr; // Pointer to the root device's configuration header if present
    PcieCapabilities*            pcieCapabilities_ = nullptr; // Pointer to the root device's PCIe capabilities structure if present
    PowerManagementCapabilities* pmCapabilities_   = nullptr; // Pointer to the root device's Power Management capabilities structure if present

    std::shared_mutex       driver_mutex_;
    DeviceInfo              rootDeviceInfo_{};
    std::vector<DeviceInfo> devices_;
};

union Bcm2711Driver::Registers
{
//    Mmio::Register<uint32_t     , 0x0188> PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1;
    Mmio::Register<uint32_t     , 0x043C> ID;

    Mmio::RegisterSet<BcmConfigRegisters, 0x4000> BcmConfig;

    Mmio::RegisterSet<ConfigHeader0, 0x8000> DeviceConfig;

    Mmio::Register<DeviceAddress, 0x9000>  CFG_INDEX;
    Mmio::Register<uint32_t     , 0x9210>  INIT;
};

Bcm2711Driver::Bcm2711Driver(PhysicalAddress mmioBase)
    : Driver(PCI_BASE, MEM_BASE)
    , bridgeRegisters{ *reinterpret_cast<BridgeRegisters*>(mmioBase) }
    , registers      { *reinterpret_cast<Registers*      >(mmioBase) }
{
    std::lock_guard<std::shared_mutex> lock(driver_mutex_);

    fmt::println("PCIe registers base is 0x{:X}...", reinterpret_cast<std::uint64_t>(&registers));
    fmt::println("Initializing PCIe driver...");

    // Reset the controller.
    registers.INIT |= 0x3u; // Assert INIT and PERST
    Cpu::Delay(1ms);
    registers.INIT &= ~0x2u; // Deassert INIT

    // SERDES_IDDQ -- Note: Seen no explanation/documentation for this.
    registers.BcmConfig().DEBUG &= ~0x0800'0000u;
    Cpu::Delay(100us);

    std::uint32_t revision = registers.BcmConfig().REV;
    fmt::println("Revision={:X}", revision);

    // Clear and mask interrupts.
    registers.BcmConfig().INTR2_CPU_CLEAR    = 0xFFFF'FFFFu;
    registers.BcmConfig().INTR2_CPU_MASK_SET = 0xFFFF'FFFFu;

    // Take controller out of reset.
    registers.INIT &= ~0x1u;
    Cpu::Delay(100ms);

    // Wait for link to become active.
    {
        uint32_t status = registers.BcmConfig().STATUS;
        for (unsigned i = 0; i < 100; i++)
        {
            if ((status & 0x30) == 0x30) { // Phy linkup & DL Active
                break;
            }
            Cpu::Delay(1ms);
            status = registers.BcmConfig().STATUS;
        }

        if ((status & 0x30) != 0x30)
        {
            fmt::println("PCIe link not ready (status={:X})", status);
            initError_.store(PCIeError::HARDWARE_ERROR);
            return;
        }

        if ((status & 0x80) == 0)
        {
            fmt::println("PCIe is not in rc mode (status={:X})", status);
            initError_.store(PCIeError::HARDWARE_ERROR);
            return;
        }

        fmt::println("PCIe link ready (status={:X})", status);
    }

    // Set up the MISC_CTRL register with appropriate values.
    registers.BcmConfig().MISC_CTRL = [](auto& reg)
        {
            fmt::println("MISC_CTRL before = 0x{:08X}", reg.Raw32);
            
            // Ignored for now.
            // reg.SCB0_SIZE        = ;
            // reg.SCB1_SIZE        = ;
            // reg.SCB2_SIZE        = ;
            // reg.RCB_MPSPMODE     = ;

            reg.SCB_ACCESS_EN    = 1;
            reg.CFG_READ_UR_MODE = 1;
            reg.MAX_BURST_SIZE   = BurstSize::Size128;
            
            fmt::println("MISC_CTRL after = 0x{:08X}", reg.Raw32);
            // (reg & 0x30'3480u) | 0x3480u;
        };

    // Set up the PCI address, split into two 32-bit registers.
    registers.BcmConfig().MEM_PCI_LO = static_cast<std::uint32_t>(PCI_BASE);
    registers.BcmConfig().MEM_PCI_HI = static_cast<std::uint32_t>(PCI_BASE >> 32);

    // Set up the CPU addresses.
    registers.BcmConfig().MEM_CPU_LO       = static_cast<uint32_t>(((MEM_BASE & 0xFFF0'0000u) >> 16) | (MEM_LIMIT & 0xFFF0'0000u));
    registers.BcmConfig().MEM_CPU_HI_START = static_cast<uint32_t>(MEM_BASE >> 32);
    registers.BcmConfig().MEM_CPU_HI_END   = static_cast<uint32_t>(MEM_LIMIT >> 32);

    // Device on 0:0:0 should be a bridge.
    std::uint16_t const vid = bridgeRegisters.BridgeConfig.Common.VendorId;
    if (vid != 0x14e4) // Broadcom vendor ID
    {
        fmt::println("PCIe bridge not found (VID={:X})", vid);
        initError_.store(PCIeError::DEVICE_NOT_FOUND);
        return;
    }

    registers.BcmConfig().RC_BAR2_CONFIG_LO = 18; // 33 - 15 == 8 GB
    registers.BcmConfig().RC_BAR2_CONFIG_HI = 4; // Newer Pi4B boards with more than 4 GB use 4'0000'0000 as the base address.

    // SCB
    registers.BcmConfig().MISC_CTRL = [](auto& reg){ reg.SCB0_SIZE = 18u; };

    // Set the class code to PCI-to-PCI bridge (0x060400) if it's not already set.
    uint32_t ccode = registers.ID;
    fmt::println("Class code {:X}", ccode);
    if ((ccode & 0xffffff) != 0x060400)
    {
        ccode = (ccode & ~0xffffff) | 0x060400;
        fmt::println("Changing to {:X}", ccode);
        registers.ID = ccode;
    }

    // CLKREQ_DEBUG_ENABLE -- Note: Something about "refclk" from RC being gated.
    registers.BcmConfig().DEBUG |= 2u;

    // The HW is enabled. Now we must set up the root bridge.

    // The Rpi4 PCIe controller is typically on bus 0, and devices appear on bus 1

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

    rootHeader_->Common.Command |= 6; // Enable memory space (bit 1) and bus mastering (bit 2)

    rootHeader_->Common.CacheLineSize  = 64 / 4; // 16??
    rootHeader_->SecondaryBus   = 1; // Device numbers.
    rootHeader_->SubordinateBus = 1;
    rootHeader_->NPMemBase      = static_cast<std::uint16_t>(PCI_BASE  >> 16) & 0xFFF0;
    rootHeader_->NPMemLimit     = static_cast<std::uint16_t>(PCI_LIMIT >> 16) & 0xFFF0;
    rootHeader_->BridgeControl  = 1; // Parity

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

    if (pcieCapabilities_)
    {
        pcieCapabilities_->RootControl = 0x0010; // CRS Software Visibility Enable
    }

    rootHeader_->Common.Command = 0x146; // !IO, Memory, Master, SERR, Parity

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
    for (BusNumber bus = 1; bus <= 1; ++bus)
    {
        // Not using constants::MAX_DEVICES_PER_BUS because there's only one device per bus on Rpi4
        // Unless you're using a PCIe expansion board, which we're not.
        // Accessing the VID of devices that are not present can result in delays of seconds (timeout request in the bus).
        uint32_t deviceCount = 1;
        for (DeviceNumber device = 0; device < deviceCount; ++device) {
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

Configuration Bcm2711Driver::ConfigureDevice(DeviceAddress addr) noexcept
{
    // The bridge is always at this address.
    bool const isBridge = addr == DeviceAddress{};
    if (isBridge)
    {
        return Configuration(addr, bridgeRegisters.BridgeConfig.Common);
    }

    // Set up the configuration space, but only when the address changes.
    // TODO: This and all other device access should be properly synchronized.
    if (lastConfiguredAddress != addr)
    {
        lastConfiguredAddress = addr;
        registers.CFG_INDEX = addr;
    }

    return Configuration(addr, registers.DeviceConfig().Common);
}

std::generator<ExtendedCapability> Bcm2711Driver::EnumerateExtendedCapabilities() const noexcept
{
    // Check if device supports extended capabilities
    ExtendedCapabilityHeader header = bridgeRegisters.FirstExtendedCapabilityHeader;
    if (header.Id == ExtendedCapabilityId::Invalid)
    {
        co_return;
    }

    uint16_t offset = bridgeRegisters.FirstExtendedCapabilityHeader.GetOffset() / sizeof(ExtendedCapabilityHeader);

    // Simulate some common capabilities
    while (offset > 0)
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

std::shared_ptr<Driver> CreateBcm2711Driver(PhysicalAddress mmioBase)
{
    return std::make_shared<Bcm2711Driver>(mmioBase);
}

}
// namespace PCIe
