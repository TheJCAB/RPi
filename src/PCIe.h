#pragma once

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

namespace PCIe
{

// Forward declarations
class Configuration;
class MemoryMappedRegion;
class InterruptHandler;

// Type aliases for clarity
using BusNumber       = std::uint8_t;
using DeviceNumber    = std::uint8_t;
using FunctionNumber  = std::uint8_t;
using VendorID        = std::uint16_t;
using DeviceID        = std::uint16_t;
using ClassCode       = std::uint32_t;
using PhysicalAddress = std::uint64_t;
using VirtualAddress  = void*;
using RegisterOffset  = std::uint32_t;
using RegisterValue   = std::uint32_t;

// PCIe-specific constants
namespace constants {
    constexpr std::size_t CONFIG_SPACE_SIZE = 4096;
    constexpr std::size_t LEGACY_CONFIG_SPACE_SIZE = 256;
    constexpr std::uint16_t RESERVED_VENDOR_ID = 0;
    constexpr std::uint16_t INVALID_VENDOR_ID = 0xFFFF;
    constexpr std::uint8_t MAX_BUSES = 255;
    constexpr std::uint8_t MAX_DEVICES_PER_BUS = 32;
    constexpr std::uint8_t MAX_FUNCTIONS_PER_DEVICE = 8;
    
    // Standard configuration space offsets
    constexpr RegisterOffset VENDOR_ID_OFFSET = 0x00;
    constexpr RegisterOffset DEVICE_ID_OFFSET = 0x02;
    constexpr RegisterOffset COMMAND_OFFSET = 0x04;
    constexpr RegisterOffset STATUS_OFFSET = 0x06;
    constexpr RegisterOffset CLASS_CODE_OFFSET = 0x08;
    constexpr RegisterOffset HEADER_TYPE_OFFSET = 0x0E;
    constexpr RegisterOffset BAR0_OFFSET = 0x10;
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
struct BarInfo {
    std::uint8_t bar_number;
    PhysicalAddress physical_address;
    std::size_t size;
    uint8_t flags;
    bool is_memory_space;  // true for memory, false for I/O
    bool is_64bit;
    bool is_prefetchable;
};

// PCIe capability structure
struct Capability {
    std::uint8_t id;
    std::uint8_t next_offset;
    std::span<const std::uint8_t> data;
};

// Concepts for type safety
template<typename T>
concept PCIeRegisterType = std::integral<T> && (sizeof(T) <= 4);

// Configuration space accessor
class Configuration
{
public:
    explicit Configuration(DeviceAddress addr) noexcept;
    ~Configuration();
    
    Configuration(Configuration&& other)
    {
        address_ = other.address_;
        registersBase_ = other.registersBase_;
        other.address_ = InvalidDeviceAddress; // Invalidate the moved-from object
        other.registersBase_ = 0;
    }

    Configuration& operator=(Configuration&& other)
    {
        if (this != &other) {
            address_ = other.address_;
            registersBase_ = other.registersBase_;
            other.address_ = InvalidDeviceAddress; // Invalidate the moved-from object
            other.registersBase_ = 0;
        }
        return *this;
    }

    [[nodiscard]] DeviceAddress GetAddress() const noexcept { return address_; }
    [[nodiscard]] bool          IsValid   () const noexcept { return address_ != InvalidDeviceAddress; }
    [[nodiscard]] explicit   operator bool() const noexcept { return address_ != InvalidDeviceAddress; }

    // Register access methods
    template<PCIeRegisterType T>
    [[nodiscard]] T read_register(RegisterOffset offset) const noexcept;

    template<PCIeRegisterType T>
    PCIeError write_register(RegisterOffset offset, T value) const noexcept;
    
    // Convenience methods for standard registers
    [[nodiscard]] VendorID  vendor_id () const noexcept;
    [[nodiscard]] DeviceID  device_id () const noexcept;
    [[nodiscard]] ClassCode class_code() const noexcept;
    [[nodiscard]] uint16_t  command   () const noexcept;
    [[nodiscard]] uint16_t  status    () const noexcept;

    [[nodiscard]] PCIeError set_command(std::uint16_t command) noexcept;

    [[nodiscard]] PCIeError enable_device();
    [[nodiscard]] PCIeError disable_device();

    // BAR access
    [[nodiscard]] size_t enumerate_bars(std::span<BarInfo>) const;
    [[nodiscard]] BarInfo get_bar(std::uint8_t bar_number) const;
    [[nodiscard]] std::span<uint8_t> map_bar(BarInfo& bar);

    // Capability iteration
    [[nodiscard]] std::expected<std::vector<Capability>, PCIeError> enumerate_capabilities() const;
    [[nodiscard]] std::expected<std::optional<Capability>, PCIeError> find_capability(std::uint8_t cap_id) const;

   
private:
    DeviceAddress address_ = InvalidDeviceAddress;
    uintptr_t registersBase_ = 0; // Base address for configuration space registers
};

// Main PCIe device information class
struct DeviceInfo
{
    PCIe::DeviceAddress Address;
    PCIe::VendorID      VendorId;
    PCIe::DeviceID      DeviceId;
    PCIe::ClassCode     ClassCode;
};


// PCIe bus manager/driver

// Initialization
[[nodiscard]] PCIeError initialize();
[[nodiscard]] bool is_initialized() noexcept;

// Device enumeration
[[nodiscard]] std::vector<DeviceInfo> enumerate_devices();
[[nodiscard]] std::vector<DeviceInfo> find_devices(VendorID vendor, std::optional<DeviceID> device = std::nullopt);
[[nodiscard]] std::vector<DeviceInfo> find_devices_by_class(ClassCode class_code, std::uint32_t mask = 0xFFFFFF00);

// Utility functions
namespace utils {
    [[nodiscard]] constexpr std::string_view class_code_to_string(ClassCode class_code) noexcept;
    [[nodiscard]] constexpr bool is_bridge_device(ClassCode class_code) noexcept;
    [[nodiscard]] constexpr bool is_endpoint_device(ClassCode class_code) noexcept;
    
    // Address conversion utilities
    [[nodiscard]] constexpr DeviceAddress bdf_to_address(std::uint32_t bdf) noexcept;
    [[nodiscard]] constexpr std::uint32_t address_to_bdf(DeviceAddress addr) noexcept;
    
    // Size and alignment utilities
    [[nodiscard]] constexpr std::size_t align_up(std::size_t value, std::size_t alignment) noexcept;
    [[nodiscard]] constexpr bool is_power_of_two(std::size_t value) noexcept;
}


// Example usage function (for demonstration)
namespace examples {
    
// Example: Enumerate and display all PCIe devices
void demonstrate_enumeration();

}
// namespace examples

}
// namespace PCIe
