#pragma once

#include "Mmio.h"

#include <cstdint>
#include <memory>
#include <vector>
#include <optional>
#include <span>
#include <concepts>
//#include <ranges>
#include <functional>
#include <string_view>
#include <array>
#include <atomic>
#include <mutex>
#include <expected>
#include <generator>

namespace PCIe
{

// Type aliases for clarity
using BusNumber       = std::uint8_t;
using DeviceNumber    = std::uint8_t;
using FunctionNumber  = std::uint8_t;
using VendorID        = std::uint16_t;
using DeviceID        = std::uint16_t;
using ClassCode       = std::uint32_t;
using PcieAddress     = std::uint64_t;
using PhysicalAddress = std::uint64_t;
using VirtualAddress  = void*;
using RegisterOffset  = std::uint32_t;
using RegisterValue   = std::uint32_t;

// Standard configuration space offsets
union CommonConfigHeader;
union ConfigHeader0;
union ConfigHeader1;
union XhciConfig;

enum class CapabilityId : uint8_t
{
    PowerManagement = 0x01,
    Msi             = 0x05,
    Pcie            = 0x10,
};

union CapabilityEntry
{
    BootLib::Register<CapabilityId     const, 0x00> Id;
    BootLib::Register<uint8_t          const, 0x01> NextPtr;
};

union PowerManagementCapabilities
{
    BootLib::Register<CapabilityId     const, 0x00> Id;                 // 0x01
    BootLib::Register<uint8_t          const, 0x01> NextPtr;
    BootLib::Register<uint16_t         const, 0x02> Capabilities;
    BootLib::Register<uint16_t              , 0x04> ControlStatus;
    BootLib::Register<uint8_t          const, 0x07> Data;
};

union MsiCapabilities
{
    BootLib::Register<CapabilityId     const, 0x00> Id;                 // 0x05
    BootLib::Register<uint8_t          const, 0x01> NextPtr;
    BootLib::Register<uint16_t         const, 0x02> Capabilities;
};

union PcieCapabilities
{
    BootLib::Register<CapabilityId     const, 0x00> Id;                 // 0x10
    BootLib::Register<uint8_t          const, 0x01> NextPtr;
    BootLib::Register<uint16_t         const, 0x02> Capabilities;
    BootLib::Register<uint32_t         const, 0x04> DeviceCapabilities;
    BootLib::Register<uint16_t              , 0x08> DeviceControl;
    BootLib::Register<uint16_t              , 0x0A> DeviceStatus;
    BootLib::Register<uint32_t         const, 0x0C> LinkCapabilities;
    BootLib::Register<uint16_t              , 0x10> LinkControl;
    BootLib::Register<uint16_t              , 0x12> LinkStatus;
    BootLib::Register<uint32_t         const, 0x14> SlotCapabilities;
    BootLib::Register<uint16_t              , 0x18> SlotControl;
    BootLib::Register<uint16_t              , 0x1A> SlotStatus;
    BootLib::Register<uint16_t              , 0x1C> RootControl;
    BootLib::Register<uint16_t         const, 0x1E> RootCapabilities;
    BootLib::Register<uint32_t              , 0x20> RootStatus;
    BootLib::Register<uint32_t         const, 0x24> DeviceCapabilities2;
    BootLib::Register<uint16_t              , 0x28> DeviceControl2;
    BootLib::Register<uint16_t              , 0x2A> DeviceStatus2;
    BootLib::Register<uint32_t         const, 0x2C> LinkCapabilities2;
    BootLib::Register<uint16_t              , 0x30> LinkControl2;
    BootLib::Register<uint16_t              , 0x32> LinkStatus2;
    BootLib::Register<uint32_t         const, 0x34> SlotCapabilities2;
    BootLib::Register<uint16_t              , 0x38> SlotControl2;
    BootLib::Register<uint16_t              , 0x3A> SlotStatus2;
};

template < CapabilityId Id > struct CapabilityStructT { using type = CapabilityEntry; };
template <> struct CapabilityStructT<CapabilityId::PowerManagement> { using type = PowerManagementCapabilities; };
template <> struct CapabilityStructT<CapabilityId::Msi            > { using type = MsiCapabilities; };
template <> struct CapabilityStructT<CapabilityId::Pcie           > { using type = PcieCapabilities; };

template < CapabilityId Id > using CapabilityStruct = typename CapabilityStructT<Id>::type;

enum class ExtendedCapabilityId : uint16_t
{
    Invalid = 0,
};

struct ExtendedCapabilityHeader
{
    ExtendedCapabilityId Id;
    uint16_t             Version         :  4;
    uint16_t             NextPtr         :  2;
    uint16_t             NextPtrInDwords : 10;
};

static_assert(sizeof(ExtendedCapabilityHeader) == sizeof(uint32_t));

union ExtendedCapabilityEntry
{
    BootLib::Register<ExtendedCapabilityHeader const, 0x00> Header;
};

// Forward declarations
class Configuration;
class MemoryMappedRegion;
class InterruptHandler;

// PCIe-specific constants
namespace constants
{
    constexpr std::size_t CONFIG_SPACE_SIZE = 4096;
    constexpr std::size_t LEGACY_CONFIG_SPACE_SIZE = 256;
    constexpr std::uint16_t RESERVED_VENDOR_ID = 0;
    constexpr std::uint16_t INVALID_VENDOR_ID = 0xFFFF;
    constexpr std::uint8_t MAX_BUSES = 255;
    constexpr std::uint8_t MAX_DEVICES_PER_BUS = 32;
    constexpr std::uint8_t MAX_FUNCTIONS_PER_DEVICE = 8;
}

// Error types for expected returns
enum class PCIeError {
    SUCCESS,
    DEVICE_NOT_FOUND,
    INVALID_ADDRESS,
    ACCESS_DENIED,
    TIMEOUT,
    HARDWARE_ERROR,
    MEMORY_MAP_FAILED,
    INTERRUPT_SETUP_FAILED,
    INVALID_CONFIGURATION,
    DRIVER_NOT_INITIALIZED
};

// PCIe device address structure
// Matches the ECAM register address
struct DeviceAddress
{
    uint32_t reserved0 : 12;
    uint32_t Function  :  3;
    uint32_t Device    :  5;
    uint32_t Bus       :  8;
    uint32_t reserved1 :  4;

    friend constexpr bool operator== (DeviceAddress const&, DeviceAddress const&) = default;
    friend constexpr auto operator<=>(DeviceAddress const&, DeviceAddress const&) = default;
};

constexpr DeviceAddress InvalidDeviceAddress{ .reserved0 = 0xFFF, .Function = 0x7, .Device = 0x1F, .Bus = 0xFF, .reserved1 = 0xF };

// ECAM (Enhanced Configuration Access Mechanism) address
struct RegisterAddress
{
    uint32_t Offset    : 12;
    uint32_t Function  :  3;
    uint32_t Device    :  5;
    uint32_t Bus       :  8;
    uint32_t reserved1 :  4;

    friend constexpr bool operator== (RegisterAddress const&, RegisterAddress const&) = default;
    friend constexpr auto operator<=>(RegisterAddress const&, RegisterAddress const&) = default;

    constexpr operator DeviceAddress() const noexcept { return DeviceAddress{ .Function = Function, .Device = Device, .Bus = Bus }; }
};

enum class BarType : uint8_t {
    IO             = 0b0000,
    Mem32          = 0b0001,
    Mem64          = 0b0101,
    Prefetchable32 = 0b1001,
    Prefetchable64 = 0b1101,
};

// Base Address Register (BAR) information
struct BarInfo
{
    std::uint8_t                 bar_number;
    uint8_t                      flags;
    bool                         is_memory_space;  // true for memory, false for I/O
    bool                         is_prefetchable;
    PhysicalAddress              physical_address;
    std::size_t                  size;
    Mmio::RegisterSpan<uint32_t> registers;
};

// PCIe capability structure
struct Capability
{
    CapabilityId Id;
    uint8_t      Offset;

    template < typename StructT > auto& GetStruct(CommonConfigHeader&       header) const { return *reinterpret_cast<StructT*      >(reinterpret_cast<uintptr_t>(&header) + Offset); }
    template < typename StructT > auto& GetStruct(CommonConfigHeader const& header) const { return *reinterpret_cast<StructT const*>(reinterpret_cast<uintptr_t>(&header) + Offset); }
};

// PCIe extended capability structure
struct ExtendedCapability
{
    ExtendedCapabilityId Id;
    uint16_t             Version         :  4;
    uint16_t             Reserved        :  2;
    uint16_t             OffsetInDwords  : 10;

    template < typename StructT > auto& GetStruct(CommonConfigHeader&       header) const { return *reinterpret_cast<StructT*      >(reinterpret_cast<ExtendedCapabilityHeader      *>(&header) + OffsetInDwords); }
    template < typename StructT > auto& GetStruct(CommonConfigHeader const& header) const { return *reinterpret_cast<StructT const*>(reinterpret_cast<ExtendedCapabilityHeader const*>(&header) + OffsetInDwords); }
};

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

// Concepts for type safety
template<typename T>
concept PCIeRegisterType = std::integral<T> && (sizeof(T) <= 4);

// Configuration space accessor
class Configuration
{
public:
    explicit Configuration(DeviceAddress address, CommonConfigHeader& header) noexcept 
        : address_{ address }
        , header_ { header  }
    {}

    ~Configuration();
    
    Configuration(Configuration&& other) = delete;
    Configuration& operator=(Configuration&& other) = delete;

    [[nodiscard]] DeviceAddress GetAddress() const noexcept { return address_; }
    [[nodiscard]] bool          IsValid   () const noexcept { return address_ != InvalidDeviceAddress; }
    [[nodiscard]] explicit   operator bool() const noexcept { return address_ != InvalidDeviceAddress; }

    // Convenience methods for standard registers
    [[nodiscard]] VendorID  GetVendorId () const noexcept { return Common().VendorId; }
    [[nodiscard]] DeviceID  GetDeviceId () const noexcept { return Common().DeviceId; }
    [[nodiscard]] ClassCode GetClassCode() const noexcept { return Common().Class->ClassCode; }

    void SetCommand(std::uint16_t command) noexcept;

    void enable_device();
    void disable_device();

    // BAR access
    [[nodiscard]] size_t MaxBars() const;
    [[nodiscard]] std::generator<BarInfo> EnumerateBars() const;
    [[nodiscard]] BarInfo GetBar(std::uint8_t bar_number) const;

    // Capability iteration
    [[nodiscard]] std::generator<Capability> EnumerateCapabilities() const;
    [[nodiscard]] std::optional<Capability> FindCapability(CapabilityId cap_id) const;

    CommonConfigHeader& Common () const { return header_; }
    ConfigHeader0&      Header0() const { return *reinterpret_cast<ConfigHeader0*>(&header_); }
    ConfigHeader1&      Header1() const { return *reinterpret_cast<ConfigHeader1*>(&header_); }
    XhciConfig&         Xhci   () const { return *reinterpret_cast<XhciConfig*   >(&header_); }

private:
    DeviceAddress                address_ = InvalidDeviceAddress;
    CommonConfigHeader&          header_; // Base address for configuration space registers
};

// Main PCIe device information class
struct DeviceInfo
{
    PCIe::DeviceAddress Address;
    PCIe::VendorID      VendorId;
    PCIe::DeviceID      DeviceId;
    PCIe::ClassCode     ClassCode;
};

union BridgeRegisters
{
    ConfigHeader1 BridgeConfig;

    Mmio::Register<ExtendedCapabilityHeader const, 0x100> FirstExtendedCapabilityHeader;
};

// PCIe bus manager/driver
// Hardware register access for Rpi4 PCIe controller
// These addresses are based on the BCM2711 datasheet
// TODO: Mapping the registers here (or wherever) via the MMU, even in low address mode.
constexpr PhysicalAddress Rpi4_PCIE_REGS_BASE_HI = 0x4'7D50'0000;

class Driver
{
public:
    explicit Driver(uintptr_t pciBaseAddress, uintptr_t memBaseAddress)
        : PciBaseAddress(pciBaseAddress)
        , MemBaseAddress(memBaseAddress)
    {}

    virtual ~Driver() = 0;

    virtual Configuration ConfigureDevice(DeviceAddress addr) noexcept = 0;

    virtual std::generator<ExtendedCapability> EnumerateExtendedCapabilities() const noexcept = 0;

    virtual std::generator<DeviceInfo> EnumerateDevices() const noexcept = 0;

    std::span<std::byte> MapBar(BarInfo& bar);

private:
    uintptr_t PciBaseAddress;
    uintptr_t MemBaseAddress;
};

void PrintBar               (BarInfo const&);
void PrintExtendedCapability(ExtendedCapability const&, CommonConfigHeader const&);
void PrintCapability        (Capability const&        , CommonConfigHeader const&);

std::shared_ptr<Driver> CreateBcm2711Driver(PhysicalAddress mmioBase);

// Utility functions
namespace utils {
    [[nodiscard]] constexpr std::string_view GetClassCode_to_string(ClassCode GetClassCode) noexcept
    {
        switch (GetClassCode >> 16) {
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
    
    [[nodiscard]] constexpr bool is_bridge_device(ClassCode GetClassCode) noexcept
    {
        return (GetClassCode >> 16) == 0x06;
    }
}

}
// namespace PCIe
