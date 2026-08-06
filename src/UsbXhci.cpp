#include "Usb.h"
#include "Uart.h"
#include "Cpu.h"
#include "Mmu.h"
#include "Interrupts.h"

#include "emb-stdio.h"

#include <mutex>
#include <atomic>
#include <cstring>

namespace Usb
{

union HcsParams2
{
    struct
    {
        uint32_t IsoSchedThreshold           :  3; // Minimum number of frames (1 ms) or microframes (128 us) to stay ahead.
        uint32_t IsoSchedThresholdIsInFrames :  1; // 1 = frames, 0 = microframes
        uint32_t EventRingSegmentTableMax    :  4;
        uint32_t Reserved                    : 13;
        uint32_t MaxScratchpadBuffersHi      :  5;
        uint32_t SaveRestoreUsesScratchpad   :  1;
        uint32_t MaxScratchpadBuffersLo      :  5;
    };
    
    uint32_t Raw32;
};

// XHCI Capability Registers offsets
constexpr uint32_t XHCI_CAP_CAPLENGTH    = 0x00;  // Capability Register Length
constexpr uint32_t XHCI_CAP_HCIVERSION   = 0x02;  // Interface Version Number
constexpr uint32_t XHCI_CAP_HCSPARAMS1   = 0x04;  // Structural Parameters 1
constexpr uint32_t XHCI_CAP_HCSPARAMS2   = 0x08;  // Structural Parameters 2
constexpr uint32_t XHCI_CAP_HCSPARAMS3   = 0x0C;  // Structural Parameters 3
constexpr uint32_t XHCI_CAP_HCCPARAMS1   = 0x10;  // Capability Parameters 1
constexpr uint32_t XHCI_CAP_DOORBELL_OFFSET = 0x14;
constexpr uint32_t XHCI_CAP_RUNTIME_OFFSET  = 0x18;

union OpConfig
{
    struct
    {
        uint32_t MaxDeviceSlotsEnabled          :  8;
        uint32_t U3EntryEnable                  :  1;
        uint32_t ConfigurationInformationEnable :  1;
        uint32_t Reserved                       : 22;
    };
    
    uint32_t Raw32;
};

// XHCI Operational Registers offsets (relative to capability length)
constexpr uint32_t XHCI_OP_USBCMD        = 0x00;  // USB Command
constexpr uint32_t XHCI_OP_USBSTS        = 0x04;  // USB Status
constexpr uint32_t XHCI_OP_PAGESIZE      = 0x08;  // Page Size
constexpr uint32_t XHCI_OP_DNCTRL        = 0x14;  // Device Notification Control
constexpr uint32_t XHCI_OP_CRCR          = 0x18;  // Command Ring Control
constexpr uint32_t XHCI_OP_DCBAAP        = 0x30;  // Device Context Base Address Array Pointer
constexpr uint32_t XHCI_OP_CONFIG        = 0x38;  // Configure

// XHCI Command Register bits
constexpr uint32_t XHCI_CMD_RUN           = (1 << 0);   // Run/Stop
constexpr uint32_t XHCI_CMD_HCRST         = (1 << 1);   // Host Controller Reset
constexpr uint32_t XHCI_CMD_INTE          = (1 << 2);   // Interrupter Enable
constexpr uint32_t XHCI_CMD_HSEE          = (1 << 3);   // Host System Error Enable

// XHCI Status Register bits
constexpr uint32_t XHCI_STS_HCH           = (1 << 0);   // HC Halted
constexpr uint32_t XHCI_STS_HSE           = (1 << 2);   // Host System Error
constexpr uint32_t XHCI_STS_EINT          = (1 << 3);   // Event Interrupt
constexpr uint32_t XHCI_STS_PCD           = (1 << 4);   // Port Change Detect
constexpr uint32_t XHCI_STS_SSS           = (1 << 8);   // Save State Status
constexpr uint32_t XHCI_STS_RSS           = (1 << 9);   // Restore State Status
constexpr uint32_t XHCI_STS_SRE           = (1 << 10);  // Save/Restore Error
constexpr uint32_t XHCI_STS_CNR           = (1 << 11);  // Controller Not Ready
constexpr uint32_t XHCI_STS_HCE           = (1 << 12);  // Host Controller Error

// XHCI Runtime Registers offsets (from runtime register space base)
constexpr uint32_t XHCI_RT_MFINDEX        = 0x00;  // Microframe Index
constexpr uint32_t XHCI_RT_IR0_IMAN       = 0x20;  // Interrupter Management
constexpr uint32_t XHCI_RT_IR0_IMOD       = 0x24;  // Interrupter Moderation
constexpr uint32_t XHCI_RT_IR0_ERSTSZ     = 0x28;  // Event Ring Segment Table Size
constexpr uint32_t XHCI_RT_IR0_ERSTBA     = 0x30;  // Event Ring Segment Table Base Address
constexpr uint32_t XHCI_RT_IR0_ERDP       = 0x38;  // Event Ring Dequeue Pointer

// TRB (Transfer Request Block) types
constexpr uint32_t TRB_TYPE_NORMAL        = 1;
constexpr uint32_t TRB_TYPE_SETUP         = 2;
constexpr uint32_t TRB_TYPE_DATA          = 3;
constexpr uint32_t TRB_TYPE_STATUS        = 4;
constexpr uint32_t TRB_TYPE_LINK          = 6;
constexpr uint32_t TRB_TYPE_EVENT_DATA    = 7;
constexpr uint32_t TRB_TYPE_NOOP_CMD      = 23;
constexpr uint32_t TRB_TYPE_ENABLE_SLOT   = 9;
constexpr uint32_t TRB_TYPE_DISABLE_SLOT  = 10;
constexpr uint32_t TRB_TYPE_ADDRESS_DEV   = 11;

// TRB completion codes
constexpr uint32_t TRB_CC_SUCCESS         = 1;
constexpr uint32_t TRB_CC_SHORT_PACKET    = 13;

// Transfer Request Block structure (16 bytes, 64-byte aligned for rings)
struct alignas(16) TRB {
    uint64_t parameter;
    uint32_t status;
    uint32_t control;
};

// Event Ring Segment Table Entry
struct alignas(16) EventRingSegment {
    uint64_t base_address;
    uint32_t size;
    uint32_t reserved;
};

// Basic USB request structure for control transfers
struct UsbRequest {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

constexpr uint32_t COMMAND_RING_SIZE = 256;
constexpr uint32_t EVENT_RING_SIZE = 256;
constexpr uint32_t TRANSFER_RING_SIZE = 256;

class UsbXhci : public UsbController
{
    uintptr_t CpuBaseAddress = 0x6'0000'0000;
    uintptr_t PciBaseAddress =   0xF800'0000;
    
    // Register access pointers
    volatile uint32_t* capability_regs = nullptr;
    volatile uint32_t* operational_regs = nullptr;
    volatile uint32_t* runtime_regs = nullptr;
    uint8_t capability_length = 0;
    uint8_t runtime_offset = 0;
    
    // Device capabilities
    uint32_t max_device_slots = 0;
    uint32_t max_interrupters = 0;
    uint32_t max_ports = 0;
    
    // DMA allocated memory regions
    TRB* command_ring = nullptr;
    TRB* event_ring = nullptr;
    EventRingSegment* event_ring_segment_table = nullptr;
    uint64_t* device_context_base_array = nullptr;
    
    // Ring management
    std::atomic<uint32_t> command_ring_enqueue{0};
    std::atomic<uint32_t> command_ring_dequeue{0};
    std::atomic<uint32_t> event_ring_dequeue{0};
    bool command_ring_cycle_state = true;
    bool event_ring_cycle_state = true;
    
    // Synchronization
    std::mutex command_mutex;
    std::atomic<bool> interrupt_pending{false};
    
    // Device slots
    struct DeviceSlot {
        uint32_t slot_id = 0;
        TRB* transfer_ring = nullptr;
        std::atomic<bool> in_use{false};
    };
    std::unique_ptr<DeviceSlot[]> device_slots;

private:
    uint32_t read_reg32(uint32_t offset) const {
        volatile uint32_t* ptr = reinterpret_cast<volatile uint32_t*>(CpuBaseAddress + offset);
        asm volatile("dsb sy" : : : "memory");  // Data synchronization barrier
        uint32_t value = *ptr;
        asm volatile("dsb sy" : : : "memory");
        return value;
    }
    
    void write_reg32(uint32_t offset, uint32_t value) const {
        volatile uint32_t* ptr = reinterpret_cast<volatile uint32_t*>(CpuBaseAddress + offset);
        asm volatile("dsb sy" : : : "memory");
        *ptr = value;
        asm volatile("dsb sy" : : : "memory");
    }
    
    uint64_t read_reg64(uint32_t offset) const {
        volatile uint64_t* ptr = reinterpret_cast<volatile uint64_t*>(CpuBaseAddress + offset);
        asm volatile("dsb sy" : : : "memory");
        uint64_t value = *ptr;
        asm volatile("dsb sy" : : : "memory");
        return value;
    }
    
    void write_reg64(uint32_t offset, uint64_t value) const {
        volatile uint64_t* ptr = reinterpret_cast<volatile uint64_t*>(CpuBaseAddress + offset);
        asm volatile("dsb sy" : : : "memory");
        *ptr = value;
        asm volatile("dsb sy" : : : "memory");
    }
    
    uint16_t read_reg16(uint32_t offset) const {
        volatile uint16_t* ptr = reinterpret_cast<volatile uint16_t*>(CpuBaseAddress + offset);
        asm volatile("dsb sy" : : : "memory");
        uint16_t value = *ptr;
        asm volatile("dsb sy" : : : "memory");
        return value;
    }
    
    uint8_t read_reg8(uint32_t offset) const {
        volatile uint8_t* ptr = reinterpret_cast<volatile uint8_t*>(CpuBaseAddress + offset);
        asm volatile("dsb sy" : : : "memory");
        uint8_t value = *ptr;
        asm volatile("dsb sy" : : : "memory");
        return value;
    }
    
    // Get physical address for DMA (assuming identity mapping for now)
    uint64_t get_physical_address(void* virtual_addr) const {
        return reinterpret_cast<uintptr_t>(virtual_addr);
    }
    
    // Ring buffer operations
    uint32_t advance_ring_pointer(uint32_t current, uint32_t ring_size) {
        return (current + 1) % ring_size;
    }
    
    void ring_doorbell(uint32_t doorbell, uint32_t target = 0) {
        uint32_t doorbell_offset = 0x000 + (doorbell * 4); // Doorbell array starts after operational registers
        // Note: This is a simplified doorbell calculation, actual implementation may vary
        write_reg32(capability_length + 0x400 + doorbell_offset, target);
    }
    
    bool setup_rings() {
        Uart::Puts("XHCI: Setting up rings...\n");
        
        // Allocate command ring (64-byte aligned)
        command_ring = Mmu::AllocateGpuMemory<TRB>((COMMAND_RING_SIZE * sizeof(TRB) + Mmu::PageSize - 1) / Mmu::PageSize);
        if (!command_ring) {
            Uart::Puts("XHCI: Failed to allocate command ring\n");
            return false;
        }
        std::memset(command_ring, 0, COMMAND_RING_SIZE * sizeof(TRB));
        
        // Allocate event ring
        event_ring = Mmu::AllocateGpuMemory<TRB>((EVENT_RING_SIZE * sizeof(TRB) + Mmu::PageSize - 1) / Mmu::PageSize);
        if (!event_ring) {
            Uart::Puts("XHCI: Failed to allocate event ring\n");
            return false;
        }
        std::memset(event_ring, 0, EVENT_RING_SIZE * sizeof(TRB));
        
        // Allocate event ring segment table
        event_ring_segment_table = Mmu::AllocateGpuMemory<EventRingSegment>(1);
        if (!event_ring_segment_table) {
            Uart::Puts("XHCI: Failed to allocate event ring segment table\n");
            return false;
        }
        
        // Setup event ring segment table
        event_ring_segment_table[0].base_address = get_physical_address(event_ring);
        event_ring_segment_table[0].size = EVENT_RING_SIZE;
        event_ring_segment_table[0].reserved = 0;
        
        // Allocate device context base array
        uint32_t dcbaa_size = (max_device_slots + 1) * sizeof(uint64_t);
        device_context_base_array = Mmu::AllocateGpuMemory<uint64_t>((dcbaa_size + Mmu::PageSize - 1) / Mmu::PageSize);
        if (!device_context_base_array) {
            Uart::Puts("XHCI: Failed to allocate device context base array\n");
            return false;
        }
        std::memset(device_context_base_array, 0, dcbaa_size);
        
        // Setup command ring control register
        uint64_t crcr = get_physical_address(command_ring) | 1; // Set ring cycle state
        write_reg64(capability_length + XHCI_OP_CRCR, crcr);
        
        // Setup device context base address array pointer
        write_reg64(capability_length + XHCI_OP_DCBAAP, get_physical_address(device_context_base_array));
        
        // Setup event ring
        write_reg32(runtime_offset + XHCI_RT_IR0_ERSTSZ, 1); // One segment
        write_reg64(runtime_offset + XHCI_RT_IR0_ERSTBA, get_physical_address(event_ring_segment_table));
        write_reg64(runtime_offset + XHCI_RT_IR0_ERDP, get_physical_address(event_ring));
        
        // Enable interrupter
        write_reg32(runtime_offset + XHCI_RT_IR0_IMAN, 0x2); // Interrupt Enable
        write_reg32(runtime_offset + XHCI_RT_IR0_IMOD, 0x00004000); // 1ms moderation
        
        Uart::Puts("XHCI: Rings setup complete\n");
        return true;
    }
    
    void process_events() {
        while (true) {
            TRB* event = &event_ring[event_ring_dequeue.load()];
            
            // Check if this TRB belongs to current cycle
            bool cycle_bit = (event->control & 1) != 0;
            if (cycle_bit != event_ring_cycle_state) {
                break; // No more events
            }
            
            // Process the event
            uint32_t trb_type = (event->control >> 10) & 0x3F;
            uint32_t completion_code = (event->status >> 24) & 0xFF;
            
            Uart::Puts("XHCI: Event TRB type=");
            Uart::PutDec(trb_type);
            Uart::Puts(" completion=");
            Uart::PutDec(completion_code);
            Uart::Puts("\n");
            
            // Advance dequeue pointer
            uint32_t new_dequeue = advance_ring_pointer(event_ring_dequeue.load(), EVENT_RING_SIZE);
            event_ring_dequeue.store(new_dequeue);
            
            // Check for ring wrap
            if (new_dequeue == 0) {
                event_ring_cycle_state = !event_ring_cycle_state;
            }
        }
        
        // Update event ring dequeue pointer register
        write_reg64(runtime_offset + XHCI_RT_IR0_ERDP, 
                   get_physical_address(&event_ring[event_ring_dequeue.load()]));
    }
    
    static Exception::Spark xhci_interrupt_handler() {
        // This would be called by the interrupt system
        // For now, just a placeholder
        Uart::Puts("XHCI: Interrupt received\n");

        return {};
    }
    
    bool send_command(TRB command_trb) {
        std::lock_guard<std::mutex> lock(command_mutex);
        
        uint32_t enqueue_pos = command_ring_enqueue.load();
        
        // Set cycle bit
        command_trb.control |= command_ring_cycle_state ? 1 : 0;
        
        // Copy command to ring
        command_ring[enqueue_pos] = command_trb;
        
        // Advance enqueue pointer
        uint32_t new_enqueue = advance_ring_pointer(enqueue_pos, COMMAND_RING_SIZE);
        command_ring_enqueue.store(new_enqueue);
        
        // Check for ring wrap
        if (new_enqueue == 0) {
            command_ring_cycle_state = !command_ring_cycle_state;
        }
        
        // Ring doorbell 0 (command ring)
        ring_doorbell(0, 0);
        
        return true;
    }
    
    uint32_t allocate_device_slot() {
        for (uint32_t i = 1; i <= max_device_slots; ++i) {
            bool expected = false;
            if (device_slots[i].in_use.compare_exchange_strong(expected, true)) {
                device_slots[i].slot_id = i;
                return i;
            }
        }
        return 0; // No available slots
    }
    
    void free_device_slot(uint32_t slot_id) {
        if (slot_id > 0 && slot_id <= max_device_slots) {
            device_slots[slot_id].in_use.store(false);
            device_slots[slot_id].slot_id = 0;
        }
    }
    
    bool wait_for_ready(uint32_t timeout_ms = 1000) {
        auto start_time = Cpu::GetPerformanceCounter();
        auto timeout_ticks = Cpu::GetPerformanceTicksForMs(timeout_ms);
        
        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            uint32_t status = read_reg32(capability_length + XHCI_OP_USBSTS);
            if (!(status & XHCI_STS_CNR)) {
                return true;
            }
        }
        return false;
    }
    
    bool reset_controller() {
        auto start_time = Cpu::GetPerformanceCounter();
        auto timeout_ticks = Cpu::GetPerformanceTicksForMs(5000);
        
        uint32_t status = 0;
        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            status = read_reg32(capability_length + XHCI_OP_USBSTS);
            if (!(status & XHCI_STS_CNR)) {
                break;
            }
        }

        if (status & XHCI_STS_CNR) {
            printf("Timed out waiting for ready state. Command: 0x%X, Status: 0x%X\n", read_reg32(capability_length + XHCI_OP_USBCMD), read_reg32(capability_length + XHCI_OP_USBSTS));
        }

        printf("Command: 0x%X, Status: 0x%X\n", read_reg32(capability_length + XHCI_OP_USBCMD), read_reg32(capability_length + XHCI_OP_USBSTS));

        Uart::Puts("XHCI: Resetting controller...\n");
        
        // Stop the controller first
        uint32_t cmd = read_reg32(capability_length + XHCI_OP_USBCMD);
        cmd &= ~XHCI_CMD_RUN;
        write_reg32(capability_length + XHCI_OP_USBCMD, cmd);
        
        // Wait for halt
        start_time = Cpu::GetPerformanceCounter();
        timeout_ticks = Cpu::GetPerformanceTicksForMs(5000);
        
        status = 0;
        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            status = read_reg32(capability_length + XHCI_OP_USBSTS);
            if (status & XHCI_STS_HCH) {
                break;
            }
        }
        
        Uart::Puts("XHCI: Controller halted...\n");

        // Reset the controller
        cmd = read_reg32(capability_length + XHCI_OP_USBCMD);
        cmd |= XHCI_CMD_HCRST;
        write_reg32(capability_length + XHCI_OP_USBCMD, cmd);
        
        // Wait for reset to complete
        start_time = Cpu::GetPerformanceCounter();
        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            cmd = read_reg32(capability_length + XHCI_OP_USBCMD);
            if (!(cmd & XHCI_CMD_HCRST)) {
                break;
            }
        }
        
        if (cmd & XHCI_CMD_HCRST) {
            Uart::Puts("XHCI: Reset timeout\n");
            printf("Command: 0x%X, Status: 0x%X\n", cmd, status);
            return false;
        }
        
        return wait_for_ready();
    }

public:
    Status initialize() override {
        Uart::Puts("XHCI: Initializing controller...\n");
        
        // Test memory access first
        Uart::Puts("XHCI: Testing memory access...\n");
        uint32_t test_value = read_reg32(0);
        Uart::Puts("XHCI: First word: 0x");
        Uart::PutHex(test_value);
        Uart::Puts("\n");
        
        if (test_value == 0xdeaddead || test_value == 0xffffffff || test_value == 0x00000000) {
            Uart::Puts("XHCI: Invalid response, device not accessible\n");
            return Status::Error;
        }

        // Read capability registers
        capability_length = read_reg8(XHCI_CAP_CAPLENGTH);
        uint16_t hci_version = read_reg16(XHCI_CAP_HCIVERSION);
        uint32_t hcsparams1 = read_reg32(XHCI_CAP_HCSPARAMS1);
        HcsParams2 hcsparams2{ .Raw32 = read_reg32(XHCI_CAP_HCSPARAMS2) };
        uint32_t hccparams1 = read_reg32(XHCI_CAP_HCCPARAMS1);
        uint32_t doorbell_offset = read_reg32(XHCI_CAP_DOORBELL_OFFSET);
        runtime_offset = read_reg32(XHCI_CAP_RUNTIME_OFFSET);

        printf("XHCI: Capability length: 0x%X\n", capability_length);
        printf("XHCI: HCI Version: 0x%X\n", hci_version);

        // Extract parameters
        max_device_slots = hcsparams1 & 0xFF;
        max_interrupters = (hcsparams1 >> 8) & 0x7FF;
        max_ports = (hcsparams1 >> 24) & 0xFF;

        printf("XHCI: Max device slots: %u\n", max_device_slots);
        printf("XHCI: Max interrupters: %u\n", max_interrupters);
        printf("XHCI: Max ports: %u\n", max_ports);
        printf("XHCI: Isochronous scheduling threshold: %u %s\n", hcsparams2.IsoSchedThreshold, hcsparams2.IsoSchedThresholdIsInFrames ? "frames" : "microframes");
        printf("XHCI: Event ring segment table max: %u\n", hcsparams2.EventRingSegmentTableMax);
        printf("XHCI: Max scratchpad buffers: %u\n", (hcsparams2.MaxScratchpadBuffersHi << 5) + hcsparams2.MaxScratchpadBuffersLo);
        printf("XHCI: Save/restore uses scratchpad: %u\n", hcsparams2.SaveRestoreUsesScratchpad);
        printf("XHCI: Doorbells offset: 0x%X\n", doorbell_offset);
        printf("XHCI: Runtime offset: 0x%X\n", runtime_offset);
        Uart::Puts("\n");

        printf("Command: 0x%X, Status: 0x%X\n", read_reg32(capability_length + XHCI_OP_USBCMD), read_reg32(capability_length + XHCI_OP_USBSTS));

        // Setup register pointers
        capability_regs = reinterpret_cast<volatile uint32_t*>(CpuBaseAddress);
        operational_regs = reinterpret_cast<volatile uint32_t*>(CpuBaseAddress + capability_length);
        runtime_regs = reinterpret_cast<volatile uint32_t*>(CpuBaseAddress + runtime_offset);
        
        // Allocate device slots array
        device_slots = std::make_unique<DeviceSlot[]>(max_device_slots + 1);
        
        // Reset the controller
        if (!reset_controller()) {
            Uart::Puts("XHCI: Controller reset failed\n");
            return Status::Error;
        }
        
        // Setup memory structures
        if (!setup_rings()) {
            Uart::Puts("XHCI: Ring setup failed\n");
            return Status::Error;
        }
        
        // Set number of device slots
        OpConfig config{ .Raw32 = read_reg32(capability_length + XHCI_OP_CONFIG) };
        config.MaxDeviceSlotsEnabled = max_device_slots;
        write_reg32(capability_length + XHCI_OP_CONFIG, config.Raw32);
        
        // Enable interrupts
        uint32_t cmd = read_reg32(capability_length + XHCI_OP_USBCMD);
        cmd |= XHCI_CMD_INTE;
        write_reg32(capability_length + XHCI_OP_USBCMD, cmd);
        
        // Register interrupt handler
        // TODO: Interrupts::EnableXhci(xhci_interrupt_handler);
        
        // Start the controller
        cmd = read_reg32(capability_length + XHCI_OP_USBCMD);
        cmd |= XHCI_CMD_RUN;
        write_reg32(capability_length + XHCI_OP_USBCMD, cmd);
        
        // Wait for controller to start
        if (!wait_for_ready()) {
            Uart::Puts("XHCI: Controller not ready\n");
            return Status::Error;
        }
        
        Uart::Puts("XHCI: Controller initialized successfully\n");
        
        // Start device enumeration
        enumerate_devices();
        
        return Status::Success;
    }

    Status shutdown() override {
        Uart::Puts("XHCI: Shutting down controller...\n");
        
        // Stop the controller
        uint32_t cmd = read_reg32(capability_length + XHCI_OP_USBCMD);
        cmd &= ~XHCI_CMD_RUN;
        write_reg32(capability_length + XHCI_OP_USBCMD, cmd);
        
        // Wait for halt
        auto start_time = Cpu::GetPerformanceCounter();
        auto timeout_ticks = Cpu::GetPerformanceTicksForMs(1000);
        
        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            uint32_t status = read_reg32(capability_length + XHCI_OP_USBSTS);
            if (status & XHCI_STS_HCH) {
                Uart::Puts("XHCI: Controller halted\n");
                return Status::Success;
            }
        }
        
        Uart::Puts("XHCI: Shutdown timeout\n");
        return Status::Timeout;
    }

    Status read(uint8_t endpoint, std::span<uint8_t> buffer) override {
        Uart::Puts("XHCI: Read from endpoint ");
        Uart::PutDec(endpoint);
        Uart::Puts(" (");
        Uart::PutDec(buffer.size());
        Uart::Puts(" bytes)\n");
        
        if (buffer.empty()) {
            return Status::Error;
        }
        
        // For now, implement a basic normal TRB transfer
        // This would typically involve:
        // 1. Finding the device context for the endpoint
        // 2. Setting up transfer TRBs on the endpoint ring
        // 3. Ringing the doorbell for the device/endpoint
        // 4. Waiting for completion event
        
        // Placeholder: zero out buffer for now
        std::memset(buffer.data(), 0, buffer.size());
        
        return Status::Success;
    }

    Status write(uint8_t endpoint, std::span<uint8_t const> data) override {
        Uart::Puts("XHCI: Write to endpoint ");
        Uart::PutDec(endpoint);
        Uart::Puts(" (");
        Uart::PutDec(data.size());
        Uart::Puts(" bytes)\n");
        
        if (data.empty()) {
            return Status::Error;
        }
        
        // For now, implement a basic normal TRB transfer
        // This would typically involve:
        // 1. Finding the device context for the endpoint
        // 2. Setting up transfer TRBs on the endpoint ring
        // 3. Ringing the doorbell for the device/endpoint
        // 4. Waiting for completion event
        
        // Placeholder: just log the data for now
        Uart::Puts("XHCI: Data: ");
        for (size_t i = 0; i < std::min(data.size(), size_t(16)); ++i) {
            Uart::PutHex(data[i]);
            Uart::Puts(" ");
        }
        if (data.size() > 16) {
            Uart::Puts("...");
        }
        Uart::Puts("\n");
        
        return Status::Success;
    }

    Status controlTransfer(uint8_t requestType, uint8_t request,
                           uint16_t value, uint16_t index,
                           std::span<uint8_t> data = {}) override {
        Uart::Puts("XHCI: Control transfer - Type: 0x");
        Uart::PutHex(requestType);
        Uart::Puts(", Request: 0x");
        Uart::PutHex(request);
        Uart::Puts(", Value: 0x");
        Uart::PutHex(value);
        Uart::Puts(", Index: 0x");
        Uart::PutHex(index);
        Uart::Puts(", Length: ");
        Uart::PutDec(data.size());
        Uart::Puts("\n");
        
        // Implement basic control transfer using Setup, Data, and Status TRBs
        // This is the most common USB operation for device enumeration
        
        UsbRequest setup_packet;
        setup_packet.bmRequestType = requestType;
        setup_packet.bRequest = request;
        setup_packet.wValue = value;
        setup_packet.wIndex = index;
        setup_packet.wLength = static_cast<uint16_t>(data.size());
        
        // For basic implementation, we need a device slot
        // In a full implementation, this would be associated with a specific device
        uint32_t slot_id = 1; // Use slot 1 for default device
        
        if (slot_id == 0 || slot_id > max_device_slots) {
            Uart::Puts("XHCI: Invalid device slot\n");
            return Status::Error;
        }
        
        // In a full implementation, this would:
        // 1. Allocate transfer ring if not exists for this endpoint
        // 2. Create Setup TRB with setup packet data
        // 3. Create Data TRB(s) if data length > 0
        // 4. Create Status TRB 
        // 5. Ring doorbell for device
        // 6. Wait for completion events
        
        // For now, simulate successful completion
        if ((requestType & 0x80) == 0x80 && !data.empty()) {
            // Device-to-host: fill with dummy data
            std::memset(data.data(), 0x42, data.size());
        }
        
        // Handle some common requests
        switch (request) {
            case 0x06: // GET_DESCRIPTOR
                Uart::Puts("XHCI: GET_DESCRIPTOR request\n");
                if (!data.empty()) {
                    // Simulate a basic device descriptor
                    data[0] = 18; // bLength
                    if (data.size() > 1) data[1] = 1; // bDescriptorType
                    if (data.size() > 2) data[2] = 0x00; // bcdUSB low
                    if (data.size() > 3) data[3] = 0x02; // bcdUSB high (USB 2.0)
                }
                break;
                
            case 0x05: // SET_ADDRESS
                Uart::Puts("XHCI: SET_ADDRESS request, address=");
                Uart::PutDec(value);
                Uart::Puts("\n");
                break;
                
            case 0x09: // SET_CONFIGURATION
                Uart::Puts("XHCI: SET_CONFIGURATION request, config=");
                Uart::PutDec(value);
                Uart::Puts("\n");
                break;
                
            default:
                Uart::Puts("XHCI: Unknown control request\n");
                break;
        }
        
        return Status::Success;
    }
    
    // Public method to manually process pending events (useful for testing)
    void process_pending_events() {
        process_events();
    }
    
    // Additional utility methods
    uint32_t get_port_status(uint32_t port) {
        if (port == 0 || port > max_ports) {
            return 0;
        }
        
        // Port status registers start at operational base + 0x400
        uint32_t port_offset = 0x400 + ((port - 1) * 0x10);
        return read_reg32(capability_length + port_offset);
    }
    
    void reset_port(uint32_t port) {
        if (port == 0 || port > max_ports) {
            return;
        }
        
        Uart::Puts("XHCI: Resetting port ");
        Uart::PutDec(port);
        Uart::Puts("\n");
        
        uint32_t port_offset = 0x400 + ((port - 1) * 0x10);
        uint32_t portsc = read_reg32(capability_length + port_offset);
        
        // Set port reset bit (bit 4)
        portsc |= (1 << 4);
        write_reg32(capability_length + port_offset, portsc);
        
        // Wait for reset to complete (simplified)
        for (int i = 0; i < 1000; ++i) {
            portsc = read_reg32(capability_length + port_offset);
            if (!(portsc & (1 << 4))) {
                break; // Reset complete
            }
        }
    }
    
    void enumerate_devices() {
        Uart::Puts("XHCI: Enumerating devices...\n");
        
        // Check all ports for connected devices
        for (uint32_t port = 1; port <= max_ports; ++port) {
            uint32_t portsc = get_port_status(port);
            
            Uart::Puts("XHCI: Port ");
            Uart::PutDec(port);
            Uart::Puts(" status: 0x");
            Uart::PutHex(portsc);
            Uart::Puts("\n");
            
            // Check if device is connected (Current Connect Status bit 0)
            if (portsc & 1) {
                Uart::Puts("XHCI: Device connected on port ");
                Uart::PutDec(port);
                Uart::Puts("\n");
                
                // Reset the port
                reset_port(port);
                
                // Enable device slot
                TRB enable_slot_cmd;
                enable_slot_cmd.parameter = 0;
                enable_slot_cmd.status = 0;
                enable_slot_cmd.control = (TRB_TYPE_ENABLE_SLOT << 10) | 1; // Set cycle bit
                
                if (send_command(enable_slot_cmd)) {
                    Uart::Puts("XHCI: Enable slot command sent for port ");
                    Uart::PutDec(port);
                    Uart::Puts("\n");
                }
            }
        }
    }
};

// Factory function to create XHCI controller
std::unique_ptr<UsbController> CreateXhciController() {
    return std::make_unique<UsbXhci>();
}

}
// namespace Usb