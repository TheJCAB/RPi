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
class Device;
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

template<typename T>
concept InterruptCallable = std::invocable<T, const Device&>;

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
    
    PCIeError set_command(std::uint16_t command) noexcept;
    
    // BAR access
    [[nodiscard]] std::expected<std::vector<BarInfo>, PCIeError> enumerate_bars() const;
    [[nodiscard]] std::expected<BarInfo, PCIeError> get_bar(std::uint8_t bar_number) const;
    
    // Capability iteration
    [[nodiscard]] std::expected<std::vector<Capability>, PCIeError> enumerate_capabilities() const;
    [[nodiscard]] std::expected<std::optional<Capability>, PCIeError> find_capability(std::uint8_t cap_id) const;
    
private:
    DeviceAddress address_ = InvalidDeviceAddress;
    uintptr_t registersBase_ = 0; // Base address for configuration space registers
};

// Memory-mapped I/O region
class MemoryMappedRegion {
public:
    MemoryMappedRegion(PhysicalAddress phys_addr, std::size_t size);
    ~MemoryMappedRegion();
    
    MemoryMappedRegion(const MemoryMappedRegion&) = delete;
    MemoryMappedRegion& operator=(const MemoryMappedRegion&) = delete;
    MemoryMappedRegion(MemoryMappedRegion&&) noexcept;
    MemoryMappedRegion& operator=(MemoryMappedRegion&&) noexcept;
    
    // Register access with type safety
    template<PCIeRegisterType T>
    [[nodiscard]] T read(std::size_t offset) const noexcept;
    
    template<PCIeRegisterType T>
    void write(std::size_t offset, T value) noexcept;
    
    // Memory barriers
    void memory_barrier() const noexcept;
    void read_barrier() const noexcept;
    void write_barrier() const noexcept;
    
    [[nodiscard]] VirtualAddress virtual_address() const noexcept { return virtual_addr_; }
    [[nodiscard]] PhysicalAddress physical_address() const noexcept { return physical_addr_; }
    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] bool is_valid() const noexcept { return virtual_addr_ != nullptr; }
    
private:
    PhysicalAddress physical_addr_;
    VirtualAddress virtual_addr_;
    std::size_t size_;
};

// Interrupt management
class InterruptHandler {
public:
    using HandlerFunction = std::function<void(const Device&)>;
    
    explicit InterruptHandler(const Device& device);
    ~InterruptHandler();
    
    InterruptHandler(const InterruptHandler&) = delete;
    InterruptHandler& operator=(const InterruptHandler&) = delete;
    //InterruptHandler(InterruptHandler&&) = default;
    //InterruptHandler& operator=(InterruptHandler&&) = default;
    
    // MSI/MSI-X support
    [[nodiscard]] PCIeError enable_msi(std::uint8_t vector_count = 1);
    [[nodiscard]] PCIeError enable_msi_x(std::uint16_t vector_count);
    PCIeError disable_interrupts() noexcept;
    
    // Handler registration
    PCIeError register_handler(HandlerFunction handler);
    void unregister_handler() noexcept;
    
    [[nodiscard]] bool is_enabled() const noexcept { return enabled_.load(); }
    [[nodiscard]] bool supports_msi() const noexcept;
    [[nodiscard]] bool supports_msi_x() const noexcept;
    
private:
    const Device& device_;
    std::atomic<bool> enabled_{false};
    HandlerFunction handler_;
    std::mutex handler_mutex_;
};

// Main PCIe device class
class Device {
public:
    explicit Device(Configuration addr);
    ~Device() = default;
    
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;
    //Device(Device&&) = default;
    //Device& operator=(Device&&) = default;
    
    // Device identification
    [[nodiscard]] DeviceAddress address   () const noexcept { return address_   ; }
    [[nodiscard]] VendorID      vendor_id () const noexcept { return vendor_id_ ; }
    [[nodiscard]] DeviceID      device_id () const noexcept { return device_id_ ; }
    [[nodiscard]] ClassCode     class_code() const noexcept { return class_code_; }
    [[nodiscard]] bool is_valid() const;
    
    // Configuration space access
    [[nodiscard]] Configuration configuration() const noexcept;
    
    // Memory mapping
    [[nodiscard]] std::expected<std::unique_ptr<MemoryMappedRegion>, PCIeError> 
    map_bar(std::uint8_t bar_number);
    
    // Power management
    PCIeError enable_device();
    PCIeError disable_device();
    [[nodiscard]] bool is_enabled() const;
    
    // Interrupt management
    [[nodiscard]] std::expected<std::unique_ptr<InterruptHandler>, PCIeError> 
    create_interrupt_handler();
    
    // DMA operations (if supported)
    [[nodiscard]] std::expected<PhysicalAddress, PCIeError> 
    allocate_dma_buffer(std::size_t size, std::size_t alignment = 4096);
    PCIeError free_dma_buffer(PhysicalAddress addr, std::size_t size);
    
private:
    DeviceAddress address_;
    VendorID      vendor_id_;
    DeviceID      device_id_;
    ClassCode     class_code_;
    mutable std::mutex device_mutex_;
    std::atomic<bool> enabled_{false};
};

// PCIe bus manager/driver
class PCIeDriver {
public:
    static PCIeDriver& instance();
    
    PCIeDriver(const PCIeDriver&) = delete;
    PCIeDriver& operator=(const PCIeDriver&) = delete;
    
    // Initialization
    [[nodiscard]] PCIeError initialize();
    void shutdown() noexcept;
    [[nodiscard]] bool is_initialized() const noexcept { return initialized_.load(); }
    
    // Device enumeration
    [[nodiscard]] std::expected<std::vector<DeviceAddress>, PCIeError> enumerate_devices();
    [[nodiscard]] std::expected<std::vector<DeviceAddress>, PCIeError> 
    find_devices(VendorID vendor, std::optional<DeviceID> device = std::nullopt);
    [[nodiscard]] std::expected<std::vector<DeviceAddress>, PCIeError> 
    find_devices_by_class(ClassCode class_code, std::uint32_t mask = 0xFFFFFF00);
    
    // Device creation
    [[nodiscard]] std::expected<std::unique_ptr<Device>, PCIeError> 
    create_device(DeviceAddress addr);
    
    // Hot-plug support
    using HotplugCallback = std::function<void(DeviceAddress, bool /* added */)>;
    PCIeError register_hotplug_callback(HotplugCallback callback);
    void unregister_hotplug_callback() noexcept;
    
    // Statistics and debugging
    struct Statistics {
        std::size_t total_devices;
        std::size_t active_devices;
        std::size_t total_interrupts;
        std::size_t dma_allocations;
        std::size_t memory_mapped_regions;
    };
    
    [[nodiscard]] Statistics get_statistics() const noexcept;
    void reset_statistics() noexcept;
    
private:
    PCIeDriver() = default;
    ~PCIeDriver() = default;
    
    std::atomic<bool> initialized_{false};
    mutable std::shared_mutex driver_mutex_;
    std::vector<std::unique_ptr<Device>> active_devices_;
    HotplugCallback hotplug_callback_;
    mutable Statistics stats_{};
};

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

// RAII helper for device management
class ScopedDevice {
public:
    explicit ScopedDevice(DeviceAddress addr);
    ~ScopedDevice();
    
    ScopedDevice(const ScopedDevice&) = delete;
    ScopedDevice& operator=(const ScopedDevice&) = delete;
    ScopedDevice(ScopedDevice&&) = default;
    ScopedDevice& operator=(ScopedDevice&&) = default;
    
    [[nodiscard]] Device* operator->() noexcept { return device_.get(); }
    [[nodiscard]] const Device* operator->() const noexcept { return device_.get(); }
    [[nodiscard]] Device& operator*() noexcept { return *device_; }
    [[nodiscard]] const Device& operator*() const noexcept { return *device_; }
    
    [[nodiscard]] bool is_valid() const noexcept { return device_ != nullptr; }
    [[nodiscard]] Device* get() noexcept { return device_.get(); }
    [[nodiscard]] const Device* get() const noexcept { return device_.get(); }
    
private:
    std::unique_ptr<Device> device_;
};



// Example usage function (for demonstration)
namespace examples {
    
// Example: Enumerate and display all PCIe devices
void demonstrate_enumeration();

// Example: Find a specific device by vendor/device ID
void find_usb_controller();

}
// namespace examples

}
// namespace PCIe
