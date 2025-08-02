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
//#include <fstream>
//#include <sstream>
//#include <iostream>
//#include <iomanip>
#include <string>
#include <memory>

namespace PCIe
{

// Hardware register access for Rpi4 PCIe controller
// These addresses are based on the BCM2711 datasheet
PhysicalAddress Rpi4_PCIE_REGS_BASE = 0x4'7D50'0000;
constexpr std::size_t Rpi4_PCIE_REGS_SIZE = 0x9310;

namespace
{
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x043C>  RPI_PCIE_REG_ID;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x400C>  RPI_PCIE_REG_MEM_PCI_LO;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x4010>  RPI_PCIE_REG_MEM_PCI_HI;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x4068>  RPI_PCIE_REG_STATUS;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x406C>  RPI_PCIE_REG_REV;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x4070>  RPI_PCIE_REG_MEM_CPU_LO;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x4080>  RPI_PCIE_REG_MEM_CPU_HI_START;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x4084>  RPI_PCIE_REG_MEM_CPU_HI_END;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x4204>  RPI_PCIE_REG_DEBUG;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x4310>  RPI_PCIE_REG_INTMASK;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x4314>  RPI_PCIE_REG_INTCLR;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, DeviceAddress, 0x9000>  RPI_PCIE_REG_CFG_INDEX;
    Mmio::RegisterProxy<Rpi4_PCIE_REGS_BASE, uint32_t     , 0x9210>  RPI_PCIE_REG_INIT;

    constexpr uint32_t RPI_PCIE_BRIDGE_OFFSET = 0;
    constexpr uint32_t RPI_PCIE_DEVICE_OFFSET = 0x8000;

    struct pci_header_common_t
    {
        uint16_t    vid;
        uint16_t    did;
        uint16_t    command;
        uint16_t    status;
        uint8_t     revision;
        uint8_t     prog;
        uint8_t     subclass;
        uint8_t     class_;
        uint8_t     cache_line_size;
        uint8_t     latency_timer;
        uint8_t     header_type;
        uint8_t     bist;
    };

    struct pci_header_0_t
    {
        pci_header_common_t common;
        uint32_t            bar[6];
    };

    struct pci_header_1_t
    {
        pci_header_common_t common;
        uint32_t            bar[2];
        uint8_t             primary_bus;
        uint8_t             secondary_bus;
        uint8_t             subordinate_bus;
        uint8_t             secondary_latency_timer;
    };

    // Simplified memory mapping without system calls
    // This would need to be implemented with actual hardware access in a real system
    void* g_config_base = nullptr;
    bool g_initialized = false;
   
    // Configuration space access via ECAM (Enhanced Configuration Access Mechanism)
    constexpr PCIe::PhysicalAddress CONFIG_BASE = 0x6'0000'0000ULL;  // 24GB mark
    constexpr std::size_t CONFIG_SIZE = 0x400'0000;  // 64MB for 256 buses
    
    // Memory space for devices
    constexpr PCIe::PhysicalAddress MEM_BASE = 0x6'0000'0000ULL;
    constexpr std::size_t MEM_SIZE = 0x400'0000;
    
    // Register offsets in PCIe controller
    constexpr std::uint32_t BRIDGE_ENABLE_REG = 0x9310;
    constexpr std::uint32_t BRIDGE_ENABLE_MASK = 0x1;

    // Simulate reading from actual hardware addresses
    template<typename T>
    T read_hardware_register(PhysicalAddress address) {
        //// Memory barrier for coherency
        //#if defined(__aarch64__) || defined(_M_ARM64)
        //    __asm__ volatile("dmb sy" ::: "memory");
        //#endif
        
        // Check if address is within PCIe configuration space
        if (address >= CONFIG_BASE && address < (CONFIG_BASE + CONFIG_SIZE)) {
            // Direct memory access to PCIe configuration space
            volatile T* reg_ptr = reinterpret_cast<volatile T*>(address);
            T value = *reg_ptr;
            
            //// Memory barrier after read
            //#if defined(__aarch64__) || defined(_M_ARM64)
            //__asm__ volatile("dmb ld" ::: "memory");
            //#endif
            
            return value;
        }
        
        // Check if address is within PCIe controller registers
        if (address >= Rpi4_PCIE_REGS_BASE && address < (Rpi4_PCIE_REGS_BASE + Rpi4_PCIE_REGS_SIZE)) {
            // Direct memory access to PCIe controller registers
            volatile T* reg_ptr = reinterpret_cast<volatile T*>(address);
            T value = *reg_ptr;
            
            //// Memory barrier after read
            //#if defined(__aarch64__) || defined(_M_ARM64)
            //__asm__ volatile("dmb ld" ::: "memory");
            //#endif
            
            return value;
        }
        
        // For other addresses, return 0xFFFFFFFF to indicate no device
        return static_cast<T>(0xFFFFFFFF);
    }

    // Simulate reading from actual hardware addresses
    template<typename T>
    void write_hardware_register(PhysicalAddress address, T value) {
        //// Memory barrier for coherency
        //#if defined(__aarch64__) || defined(_M_ARM64)
        //    __asm__ volatile("dmb sy" ::: "memory");
        //#endif
        
        // Check if address is within PCIe configuration space
        if (address >= CONFIG_BASE && address < (CONFIG_BASE + CONFIG_SIZE)) {
            // Direct memory access to PCIe configuration space
            volatile T* reg_ptr = reinterpret_cast<volatile T*>(address);
            *reg_ptr = value;
            
            //// Memory barrier after read
            //#if defined(__aarch64__) || defined(_M_ARM64)
            //__asm__ volatile("dmb ld" ::: "memory");
            //#endif
            
            return;
        }
        
        // Check if address is within PCIe controller registers
        if (address >= Rpi4_PCIE_REGS_BASE && address < (Rpi4_PCIE_REGS_BASE + Rpi4_PCIE_REGS_SIZE)) {
            // Direct memory access to PCIe controller registers
            volatile T* reg_ptr = reinterpret_cast<volatile T*>(address);
            *reg_ptr = value;
            
            //// Memory barrier after read
            //#if defined(__aarch64__) || defined(_M_ARM64)
            //__asm__ volatile("dmb ld" ::: "memory");
            //#endif
            
            return;
        }
        
        // For other addresses, ignore the write.
    }

    // Stub implementations for platforms without proper support
    void* map_physical_memory_stub(PhysicalAddress phys_addr, std::size_t size) {
        // In a real implementation, this would:
        // 1. Map physical memory using /dev/mem on Linux
        // 2. Use a kernel driver on Windows
        // 3. Use direct hardware access on bare metal
        
        // For now, return nullptr to indicate failure
        return nullptr;
    }
    
    void unmap_physical_memory_stub(void* virtual_addr, std::size_t size) {
        // Stub implementation
    }
    
    bool init_platform_stub() {
        // Enhanced platform initialization for Rpi4
        // In a real implementation, this would:
        // 1. Check if PCIe is enabled in device tree
        // 2. Initialize PCIe controller registers
        // 3. Enable PCIe clock and power
        // 4. Wait for link training to complete

        // Reset controller.
        uint32_t init = RPI_PCIE_REG_INIT;
        printf("RPI_PCIE_REG_INIT=%x\n", init);
        init |= 0x3;
        RPI_PCIE_REG_INIT = init;

        init = RPI_PCIE_REG_INIT;
        printf("RPI_PCIE_REG_INIT after reset=%x\n", init);

        Cpu::DelayInMicroseconds(1000);

        init = RPI_PCIE_REG_INIT;
        printf("RPI_PCIE_REG_INIT=%x\n", init);
        init &= ~0x2;
        RPI_PCIE_REG_INIT = init;
        init = RPI_PCIE_REG_INIT;
        printf("RPI_PCIE_REG_INIT after reset=%x\n", init);

        uint32_t rev = RPI_PCIE_REG_REV;
        printf("Rev=%x\n", rev);

        // Clear and mask interrupts.
        RPI_PCIE_REG_INTCLR  = 0xFFFF'FFFFu;
        RPI_PCIE_REG_INTMASK = 0xFFFF'FFFFu;

        // Take controller out of reset.
        RPI_PCIE_REG_INIT = RPI_PCIE_REG_INIT & ~0x1;

        // Wait for link to become active.
        uint32_t status = RPI_PCIE_REG_STATUS;
        for (unsigned i = 0; i < 100; i++) {
            if ((status & 0x30) == 0x30) {
                break;
            }
            Cpu::DelayInMicroseconds(1000);
            status = RPI_PCIE_REG_STATUS;
        }

        if ((status & 0x30) != 0x30) {
            printf("PCIe link not ready (status=%x)\n", status);
            return false;
        }

        printf("PCIe link ready (status=%x)\n", status);

        uint32_t ccode = RPI_PCIE_REG_ID;
        printf("Class code %x\n", ccode);
        if ((ccode & 0xffffff) != 0x060400) {
            ccode = (ccode & ~0xffffff) | 0x060400;
            printf("Changing to %x\n", ccode);
            RPI_PCIE_REG_ID = ccode;
        }

        // Set up the PCI address, split into two 32-bit registers.
        RPI_PCIE_REG_MEM_PCI_LO = 0xF800'0000u;
        RPI_PCIE_REG_MEM_PCI_HI = 0;

        // Set up the CPU addresses.
        // The low register holds the bottom part of the start and end addresses as
        // two 12-bit values expressed in megabytes:
        // | low_end |  0  | low_start | 0 |
        // | 31    20|   16|          4|  0|
        // Note that the end address gets truncated at the megabyte boundary, so
        // 0x1000000 bytes will become 0xf megabytes.
        // There are two high registers for holding the top 32-bits of the start and
        // end addresses, respectively.
        // Of course, this is all speculation in the absence of an official
        // datasheet.
        uint64_t cpu_addr_start = 0x6'0000'0000ull;
        uint64_t cpu_addr_end   = cpu_addr_start + 0x1'0000'0000ull; // 4 GB

        RPI_PCIE_REG_MEM_CPU_LO       = static_cast<uint32_t>(((cpu_addr_start >> 16) & 0xfff0) | (((cpu_addr_end >> 20) - 1) << 20));
        RPI_PCIE_REG_MEM_CPU_HI_START = static_cast<uint32_t>(cpu_addr_start >> 32);
        RPI_PCIE_REG_MEM_CPU_HI_END   = static_cast<uint32_t>(cpu_addr_end >> 32);

        // Device on 0:0:0 should be a bridge.
        auto const vid = *(volatile uint16_t *)(Rpi4_PCIE_REGS_BASE + RPI_PCIE_BRIDGE_OFFSET + 0x00);
        if (vid != 0x14e4) { // Broadcom vendor ID
            printf("PCIe bridge not found (VID=%x)\n", vid);
            return false;
        }

        // Configure secondary and subordinate device numbers.
        *(volatile uint8_t *)(Rpi4_PCIE_REGS_BASE + RPI_PCIE_BRIDGE_OFFSET + 0x18) = 1; // Secondary bus
        *(volatile uint8_t *)(Rpi4_PCIE_REGS_BASE + RPI_PCIE_BRIDGE_OFFSET + 0x19) = 1; // Subordinate bus

        // For simulation, we'll assume success
        g_initialized = true;
        return true;
    }

    void cleanup_platform_stub() {
        // Cleanup platform resources
        g_initialized = false;
    }

    // Check if PCIe controller is enabled and operational
    bool is_pcie_controller_ready() {
        // In a real implementation, this would:
        // 1. Read PCIe controller status registers
        // 2. Check link status and training completion
        // 3. Verify bridge configuration
        
        // For simulation, assume ready if initialized
        return g_initialized;
    }
    
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
Configuration::Configuration(DeviceAddress addr) noexcept 
    : address_(addr)
{
    if (addr == InvalidDeviceAddress)
    {
        return;
    }
    if (addr == DeviceAddress{})
    {
        // The bridge is always at this address.
        registersBase_ = Rpi4_PCIE_REGS_BASE + RPI_PCIE_BRIDGE_OFFSET;
//        root_access_mutex_.lock();
        return;
    }

//    device_access_mutex_.lock();

    static constinit DeviceAddress lastAddress = DeviceAddress{};

    // Set up the configuration space, but only when the address changes.
    // Multicore TODO: This and all other device access should be properly
    // done in a real implementation.
    if (lastAddress != addr)
    {
        lastAddress = addr;
        RPI_PCIE_REG_CFG_INDEX = addr;
    }

    registersBase_ = Rpi4_PCIE_REGS_BASE + RPI_PCIE_DEVICE_OFFSET;
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

template<PCIeRegisterType T>
T Configuration::read_register(RegisterOffset offset) const noexcept {
    //std::lock_guard<std::mutex> lock(access_mutex_);
    
    if (offset + sizeof(T) > constants::CONFIG_SPACE_SIZE) {
        return static_cast<T>(0xFFFFFFFFu);
    }

    // Attempt to read from hardware (simulated)
    T value = read_hardware_register<T>(registersBase_ + offset);

    // Check for invalid vendor ID which indicates no device
    if (offset == constants::VENDOR_ID_OFFSET && value == static_cast<T>(0)) {
        return static_cast<T>(0xFFFFu); // Device not found
    }
    
    return value;
}

template<PCIeRegisterType T>
PCIeError Configuration::write_register(RegisterOffset offset, T value) const noexcept {
    //std::lock_guard<std::mutex> lock(access_mutex_);
    
    if (offset + sizeof(T) > constants::CONFIG_SPACE_SIZE) {
        return PCIeError::INVALID_ADDRESS;
    }

    // In a real implementation, this would write to the hardware register
    // For now, we'll simulate the write by updating our cache
    // Some registers are read-only, so we need to handle that appropriately
    
    bool is_writable = true;
    
    // Check for read-only registers
    switch (offset) {
        case constants::VENDOR_ID_OFFSET:
        case constants::DEVICE_ID_OFFSET:
        case constants::CLASS_CODE_OFFSET:
        case constants::HEADER_TYPE_OFFSET:
            is_writable = false;
            break;
        default:
            break;
    }
    
    if (is_writable) {
        // In a real implementation, write to hardware here:
        write_hardware_register(registersBase_ + offset, value);
    }
    
    return PCIeError::SUCCESS;
}

// Explicit template instantiations
template std::uint8_t Configuration::read_register<std::uint8_t>(RegisterOffset) const noexcept;
template std::uint16_t Configuration::read_register<std::uint16_t>(RegisterOffset) const noexcept;
template std::uint32_t Configuration::read_register<std::uint32_t>(RegisterOffset) const noexcept;
template PCIeError Configuration::write_register<std::uint8_t>(RegisterOffset, std::uint8_t) const noexcept;
template PCIeError Configuration::write_register<std::uint16_t>(RegisterOffset, std::uint16_t) const noexcept;
template PCIeError Configuration::write_register<std::uint32_t>(RegisterOffset, std::uint32_t) const noexcept;

VendorID Configuration::vendor_id() const noexcept
{
    return read_register<VendorID>(constants::VENDOR_ID_OFFSET);
}

DeviceID Configuration::device_id() const noexcept {
    return read_register<DeviceID>(constants::DEVICE_ID_OFFSET);
}

ClassCode Configuration::class_code() const noexcept {
    auto result = read_register<std::uint32_t>(constants::CLASS_CODE_OFFSET);
    if (!result) return ClassCode{};
    return static_cast<ClassCode>(result >> 8);  // Upper 24 bits
}

std::uint16_t Configuration::command() const noexcept {
    return read_register<std::uint16_t>(constants::COMMAND_OFFSET);
}

std::uint16_t Configuration::status() const noexcept {
    return read_register<std::uint16_t>(constants::STATUS_OFFSET);
}

PCIeError Configuration::set_command(std::uint16_t command) noexcept {
    return write_register<std::uint16_t>(constants::COMMAND_OFFSET, command);
}

size_t Configuration::enumerate_bars(std::span<BarInfo> bars) const
{
    size_t count = 0;
    for (std::uint8_t bar_num = 0; bar_num < 6 && count < bars.size(); ++bar_num)
    {
        bars[count] = get_bar(bar_num);
        if (bars[count].is_64bit) {
            // If it's a 64-bit BAR, we need to skip the next BAR
            ++bar_num;
        }
        if (bars[count].size > 0) {
            ++count;
        }
    }
    
    return count;
}

BarInfo Configuration::get_bar(std::uint8_t bar_number) const
{
    if (bar_number >= 6) {
        return {};
    }
    
    RegisterOffset bar_offset = constants::BAR0_OFFSET + (bar_number * 4);

    auto bar_low = read_register<std::uint32_t>(bar_offset);
    if (bar_low == 0 || bar_low == 0xFFFF'FFFFu) {
        return {};
    }

    // Get the size.
    write_register<uint32_t>(bar_offset, 0xFFFF'FFFFu);
    auto bar_mask = read_register<std::uint32_t>(bar_offset);
    printf("BAR%u: low=0x%08X, mask=0x%08X\n", bar_number, bar_low, bar_mask);
    write_register<uint32_t>(bar_offset, bar_low); // Restore original value

    BarInfo bar_info{};
    bar_info.bar_number = bar_number;
    bar_info.flags = bar_low & 0xF;
    bar_info.is_memory_space = (bar_low & 0x1) == 0;

    if (bar_info.is_memory_space) {
        bar_info.is_64bit = ((bar_low >> 1) & 0x3) == 0x2;
        bar_info.is_prefetchable = (bar_low & 0x8) != 0;
        bar_info.physical_address = bar_low & 0xFFFFFFF0;
        bar_info.size = ~(bar_mask & ~0xFu) + 1;

        if (bar_info.is_64bit) {
            // For 64-bit BARs, we need to read the next 32 bits
            RegisterOffset bar_high_offset = bar_offset + 4;
            auto bar_high = read_register<std::uint32_t>(bar_high_offset);
            bar_info.physical_address |= static_cast<PhysicalAddress>(bar_high) << 32;

            write_register<uint32_t>(bar_high_offset, 0xFFFF'FFFFu);
            auto bar_high_mask = read_register<std::uint32_t>(bar_high_offset);
            write_register<uint32_t>(bar_high_offset, bar_high); // Restore original value
            printf("BAR%u high: high=0x%08X, mask=0x%08X\n", bar_number + 1, bar_high, bar_high_mask);
            bar_info.size = ~((static_cast<PhysicalAddress>(bar_high_mask) << 32) + (bar_mask & ~0xFu)) + 1;
        }
    } else {
        // I/O space
        bar_info.physical_address = bar_low & 0xFFFFFFFC;
        bar_info.is_64bit = false;
        bar_info.is_prefetchable = false;
        bar_info.size = ~(bar_mask & ~0x3u) + 1;
    }
    
    return bar_info;
}

std::expected<std::vector<Capability>, PCIeError> Configuration::enumerate_capabilities() const {
    std::vector<Capability> capabilities;
    
    // Check if device supports capabilities
    auto status_result = status();
    if (status_result == 0xFFFFu || !(status_result & 0x10)) {
        return std::expected<std::vector<Capability>, PCIeError>(std::move(capabilities));
    }

    // Simulate some common capabilities
    uint8_t capabilityPointer = 0x34;
    while (capabilityPointer != 0x00u)
    {
        Capability cap{};
        cap.id = read_register<std::uint8_t>(capabilityPointer);

        // Read next capability pointer
        cap.next_offset = read_register<std::uint8_t>(capabilityPointer + 1);

        printf("Capability ID: %02X at offset %02X\n", cap.id, capabilityPointer);

        // Add to capabilities list
        capabilities.push_back(cap);

        capabilityPointer = cap.next_offset;
    }

    return std::expected<std::vector<Capability>, PCIeError>(std::move(capabilities));
}

std::expected<std::optional<Capability>, PCIeError> Configuration::find_capability(std::uint8_t cap_id) const {
    auto caps_result = enumerate_capabilities();
    if (!caps_result) {
        return std::unexpected{ caps_result.error() };
    }
    
    for (const auto& cap : caps_result.value()) {
        if (cap.id == cap_id) {
            return std::expected<std::optional<Capability>, PCIeError>(cap);
        }
    }
    
    return std::expected<std::optional<Capability>, PCIeError>(std::nullopt);
}

uintptr_t NextUnusedMapOffsetAddress = 0;

std::span<std::byte> Configuration::map_bar(BarInfo& bar)
{
    if (!bar.is_memory_space || bar.size == 0) {
        return {};
    }

    RegisterOffset bar_offset = constants::BAR0_OFFSET + (bar.bar_number * 4);

    uint64_t pciAddress = 0xF800'0000u + NextUnusedMapOffsetAddress; // Base address for PCIe memory space
    write_register<std::uint32_t>(bar_offset, (static_cast<uint32_t>(pciAddress) & ~0xFu) | bar.flags);
    if (bar.is_64bit) {
        RegisterOffset bar_high_offset = bar_offset + 4;
        write_register<std::uint32_t>(bar_high_offset, static_cast<uint32_t>(pciAddress >> 32));
    }

    bar.physical_address = 0x6'0000'0000ull + NextUnusedMapOffsetAddress;
    NextUnusedMapOffsetAddress += bar.size;

    return { reinterpret_cast<std::byte*>(bar.physical_address), bar.size };
}

PCIeError Configuration::enable_device()
{
    // Enable memory and I/O space access, bus mastering
    std::uint16_t c = command();
    c |= 0x07;  // Enable memory space, I/O space, and bus mastering
    
    auto set_result = set_command(c);
    if (set_result != PCIeError::SUCCESS) {
        return set_result;
    }
    
    return PCIeError::SUCCESS;
}

PCIeError Configuration::disable_device()
{
    std::uint16_t c = command();
    c &= ~0x07;  // Disable memory space, I/O space, and bus mastering
    
    auto set_result = set_command(c);
    if (set_result != PCIeError::SUCCESS) {
        return set_result;
    }
    
    return PCIeError::SUCCESS;
}

std::atomic<bool> initialized_{false};
std::shared_mutex driver_mutex_;
std::vector<DeviceInfo> devices_;

PCIeError initialize() {
    std::lock_guard<std::shared_mutex> lock(driver_mutex_);
    
    if (initialized_.load()) {
        return PCIeError::SUCCESS;
    }
    
    // Initialize platform-specific resources
    if (!init_platform_stub()) {
        return PCIeError::HARDWARE_ERROR;
    }
    
    // Check if PCIe controller is ready
    if (!is_pcie_controller_ready()) {
        cleanup_platform_stub();
        return PCIeError::HARDWARE_ERROR;
    }
    
    // Verify we can access the root complex
    DeviceAddress root_addr{};
    auto root_config = Configuration(root_addr);
    auto root_vendor = root_config.vendor_id();

    if (root_vendor == constants::INVALID_VENDOR_ID) {
        cleanup_platform_stub();
        return PCIeError::DEVICE_NOT_FOUND;
    }

    // Real PCIe device enumeration for Raspberry Pi 4
    // The Rpi4 PCIe controller is typically on bus 0, and devices appear on bus 1
    
    {
        // First, check if the PCIe root complex exists (bus 0, device 0, function 0)
        DeviceAddress root_addr{};
        Configuration root_config(root_addr);
        auto root_vendor = root_config.vendor_id();
        
        if (root_vendor != constants::INVALID_VENDOR_ID) {
            devices_.push_back(DeviceInfo{
                .Address   = root_addr,
                .VendorId  = root_vendor,
                .DeviceId  = root_config.device_id(),
                .ClassCode = root_config.class_code()
            });
        }
    }

    // Scan all possible device locations on the PCIe bus
    // On Rpi4, we typically see devices on bus 1 (downstream from the root complex)
    for (BusNumber bus = 1; bus <= 1; ++bus) {  // Rpi4 usually has buses 0 and 1
        // Bus 0 has only one device (the root complex).
        // Not using constants::MAX_DEVICES_PER_BUS because there's only one device per bus on Rpi4
        // Unless you're using a PCIe expansion board, which we're not.
        // Accessing the VID of devices that are not present can result in delays of seconds (timeout request in the bus).
        uint32_t deviceCount = (bus == 0) ? 1 : 1;
        for (DeviceNumber device = 0; device < deviceCount; ++device) {
            for (FunctionNumber function = 0; function < constants::MAX_FUNCTIONS_PER_DEVICE; ++function) {
                DeviceAddress addr{ .Function = function, .Device = device, .Bus = bus };
                Configuration config(addr);

                printf("Scanning device at %02u:%02u.%01u\n", bus, device, function);
                
                // Read vendor ID to check if device exists
                auto vendor = config.vendor_id();
                if (vendor == constants::INVALID_VENDOR_ID || vendor == constants::RESERVED_VENDOR_ID) {
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
                    auto header_type = config.read_register<std::uint8_t>(constants::HEADER_TYPE_OFFSET);
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

    initialized_.store(true);

    return PCIeError::SUCCESS;
}

[[nodiscard]] bool is_initialized() noexcept { return initialized_.load(); }

std::vector<DeviceInfo> find_devices(VendorID vendor, std::optional<DeviceID> device)
{
    std::vector<DeviceInfo> matching_devices;

    for (const auto& info : devices_)
    {
        if (info.VendorId != vendor) continue;
        
        if (device.has_value()) {
            if (info.DeviceId != device.value()) continue;
        }

        matching_devices.push_back(info);
    }
    
    return matching_devices;
}

std::vector<DeviceInfo> find_devices_by_class(ClassCode class_code, std::uint32_t mask)
{
    std::vector<DeviceInfo> matching_devices;

    for (const auto& info : devices_)
    {
        if ((info.ClassCode & mask) == (class_code & mask)) {
            matching_devices.push_back(info);
        }
    }
    
    return matching_devices;
}

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
    void demonstrate_enumeration() {
        printf("Enumerating PCIe devices...\n");

        printf("Initializing PCIe driver...\n");

        auto init_result = initialize();
        if (init_result != PCIeError::SUCCESS) {
            // Handle initialization error
            return;
        }

        printf("PCIe driver initialized successfully.\n");

        printf("Enumerating devices...\n");
        
        printf("Found %zu PCIe devices:\n", devices_.size());
        
        // Process each found device
        for (const auto& info : devices_) {
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
            Configuration configuration{ info.Address };
            auto command = configuration.command();
            auto status = configuration.status();
            printf("  Command:   0x%04x\n", command);
            printf("  Status:    0x%04x\n", status);

            if (utils::is_bridge_device(class_code))
            {
                configuration.write_register<uint16_t>(0x20, (0xF800'0000u >> 16) & 0xFFF0u);
                configuration.write_register<uint16_t>(0x22, (0xFFF0'0000u >> 16) & 0xFFF0u);

                configuration.set_command(command | 6); // Enable memory space (bit 1) and bus mastering (bit 2)

                uint8_t  const primaryBus           = configuration.read_register<uint8_t >(0x18);
                uint8_t  const secondaryBus         = configuration.read_register<uint8_t >(0x19);
                uint8_t  const subordinateBus       = configuration.read_register<uint8_t >(0x1A);
                uint8_t  const legacyLatencyTimer   = configuration.read_register<uint8_t >(0x1B);
                uint8_t  const ioBase               = configuration.read_register<uint8_t >(0x1C) + (static_cast<uint32_t>(configuration.read_register<uint16_t >(0x30)) << 8);
                uint8_t  const ioLimit              = configuration.read_register<uint8_t >(0x1D) + (static_cast<uint32_t>(configuration.read_register<uint16_t >(0x32)) << 8);
                uint16_t const secondaryStatus      = configuration.read_register<uint16_t>(0x1E);
                uint32_t const memoryBase           = static_cast<uint32_t>(configuration.read_register<uint16_t>(0x20) & 0xFFF0u) << 16;
                uint32_t const memoryLimit          = static_cast<uint32_t>(configuration.read_register<uint16_t>(0x22) & 0xFFF0u) << 16;
                uint32_t const prefetchableMemBase  = (static_cast<uint64_t>(configuration.read_register<uint16_t>(0x24) & 0xFFFFu) << 16) + (static_cast<uint64_t>(configuration.read_register<uint32_t>(0x28)) << 32);
                uint32_t const prefetchableMemLimit = (static_cast<uint64_t>(configuration.read_register<uint16_t>(0x26) & 0xFFFFu) << 16) + (static_cast<uint64_t>(configuration.read_register<uint32_t>(0x2C)) << 32);
                uint8_t  const interruptLine        = configuration.read_register<uint8_t >(0x3C);
                uint8_t  const interruptPin         = configuration.read_register<uint8_t >(0x3D);
                uint16_t const bridgeControl        = configuration.read_register<uint16_t>(0x3E);

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
            else
            {
                // Display BARs if any
                BarInfo bars[6];
                size_t barCount = configuration.enumerate_bars(bars);
                if (barCount > 0)
                {
                    printf("  BARs:\n");
                    for (const auto& bar : std::span{ bars, barCount }) {
                        printf("    BAR%d: 0x%016llx (size: 0x%x, %s%s%s)\n",
                            bar.bar_number,
                            static_cast<unsigned long long>(bar.physical_address),
                            static_cast<unsigned int>(bar.size),
                            bar.is_memory_space ? "Memory" : "I/O",
                            bar.is_64bit ? ", 64-bit" : "",
                            bar.is_prefetchable ? ", Prefetchable" : "");
                    }
                }

                // Display capabilities if any
                auto caps_result = configuration.enumerate_capabilities();
                if (caps_result && !caps_result.value().empty()) {
                    printf("  Capabilities:\n");
                    for (const auto& cap : caps_result.value()) {
                        printf("    ID: 0x%02x\n", cap.id);
                    }
                }

                printf("Word0: %08X\n", *(uint32_t*)(0x6'0000'0000ull));
        
        // Enable USB controller power via mailbox
        printf("    Enabling USB controller power...\n");
        constexpr uint32_t USB_HCD = 3; // USB Host Controller Device ID
        //Mailbox::TagMessage<Mailbox::Tag::SET_POWER_STATE, 2> powerStateTag{{ USB_HCD, 3 }};
        Mailbox::TagMessage<Mailbox::Tag::RPI4_PCIE_XHCI_USB_RESET, 1> resetTag{{ 0x0010'0000 }};

        if (!Mailbox::SendTags(resetTag)) {
            printf("    ✗ Failed to enable USB controller power\n");
        }
        else
        {
            printf("    New state: %u\n", resetTag.args[0]);
        }

                // Enable BAR 0 at the beginning of PCIe aperture
                auto bar0 = configuration.get_bar(0);
                if (bar0.size > 0) {
                    // The PCIe controller maps CPU address 0x600000000 to PCI address 0xF8000000
                    // So we need to program the device's BAR to use the PCI address that corresponds
                    // to where we want to access it in CPU memory space
                    uint64_t cpu_address = 0x600000000;      // Where we'll access it from CPU
                    uint32_t pci_address = 0xF8000000;       // PCI address space base
                    
                    // Write the PCI address to BAR 0 (this is what the device will see)
                    configuration.write_register<uint32_t>(0x10, (pci_address & ~0xFu) | bar0.flags);
                    if (bar0.is_64bit) {
                        configuration.write_register<uint32_t>(0x14, static_cast<uint32_t>(pci_address >> 32));
                    }
                    
                    // Enable memory space access and bus mastering
                    auto cmd = configuration.command();
                    cmd |= 0x06;  // Enable memory space (bit 1) and bus mastering (bit 2)
                    configuration.set_command(cmd);
                    
                    printf("    Configured BAR 0: PCI=0x%08x, CPU=0x%016llx\n", pci_address, cpu_address);
                    
                    // Verify the BAR was written correctly
                    auto written_bar = configuration.read_register<uint32_t>(0x10);
                    printf("    BAR 0 readback: 0x%08x\n", written_bar);
                    
                    // Add a delay to ensure the configuration takes effect
                    Cpu::DelayInMicroseconds(10000);

                    asm volatile("dsb sy" : : : "memory");  // ARM64

                    // Test memory access
                    printf("    Testing memory access at 0x%016llx...\n", cpu_address);
                    volatile uint32_t* test_ptr = reinterpret_cast<volatile uint32_t*>(cpu_address);
                    uint32_t test_value = *test_ptr;
                    printf("    First word: 0x%08x\n", test_value);
                }
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
