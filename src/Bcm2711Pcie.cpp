
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

// Memory space for devices
constexpr PCIe::PhysicalAddress MEM_BASE = 0x6'0000'0000ULL;
constexpr PCIe::PcieAddress     PCI_BASE = 0x0'C000'0000u;
constexpr std::size_t           MEM_SIZE = 0x400'0000;

constexpr PCIe::PhysicalAddress MEM_LIMIT = MEM_BASE + MEM_SIZE - 1;
constexpr PCIe::PcieAddress     PCI_LIMIT = PCI_BASE + MEM_SIZE - 1;

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

    bool Initialize() override;

    Configuration ConfigureDevice(DeviceAddress addr) noexcept override;

    union Registers;

    std::atomic<PCIeError> initError_ = PCIeError::DRIVER_NOT_INITIALIZED;

    Registers&       registers;

    DeviceAddress lastConfiguredAddress{};

    ConfigHeader1&               rootHeader; // Root bridge's configuration header

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
    : Driver(PCI_BASE, MEM_BASE, MEM_SIZE)
    , rootHeader{ *reinterpret_cast<ConfigHeader1*>(mmioBase) }
    , registers { *reinterpret_cast<Registers*    >(mmioBase) }
{
    fmt::println("PCIe configuration registers base is 0x{:X}...", reinterpret_cast<std::uint64_t>(&rootHeader));

    initError_.store(PCIeError::SUCCESS);
}

bool Bcm2711Driver::Initialize()
{
    // Initialization logic here

    if (initError_.load() != PCIeError::SUCCESS)
    {
        return false;
    }

    return Driver::Initialize();
}

Configuration Bcm2711Driver::ConfigureDevice(DeviceAddress addr) noexcept
{
    // The bridge is always at this address.
    bool const isBridge = addr == DeviceAddress{};
    if (isBridge)
    {
        return Configuration(addr, rootHeader.Common);
    }

    if (addr.Bus == 0 && addr.Device > 0)
    {
        // This HW reserves bus 0 for the root bridge only.
        return Configuration{};
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

std::shared_ptr<Driver> CreateBcm2711Driver(PhysicalAddress mmioBase)
{
    auto& registers = *reinterpret_cast<Bcm2711Driver::Registers*>(mmioBase);

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
            return {};
        }

        if ((status & 0x80) == 0)
        {
            fmt::println("PCIe is not in rc mode (status={:X})", status);
            return {};
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

    registers.BcmConfig().RC_BAR2_CONFIG_LO = 18; // 33 - 15 == 8 GB
    registers.BcmConfig().RC_BAR2_CONFIG_HI = 4; // Newer Pi4B boards with more than 4 GB use 4'0000'0000 as the base address.

    // SCB
    registers.BcmConfig().MISC_CTRL = [](auto& reg){ reg.SCB0_SIZE = 18u; };

    auto& rootHeader = *reinterpret_cast<CommonConfigHeader*>(mmioBase);

    // Device on 0:0:0 should be a bridge.
    std::uint16_t const vid = rootHeader.VendorId;
    if (vid != 0x14e4) // Broadcom vendor ID
    {
        fmt::println("PCIe bridge not found (VID={:X})", vid);
        return {};
    }

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

    auto driver = std::make_shared<Bcm2711Driver>(mmioBase);
    if (!driver->Initialize())
    {
        return {};
    }
    return driver;
}

}
// namespace PCIe
