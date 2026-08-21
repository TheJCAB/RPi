#include "Usb.h"
#include "UsbDriver.h"
#include "Uart.h"
#include "Cpu.h"
#include "Mmu.h"
#include "Interrupts.h"
#include "Mailbox.h"
#include "Mmio.h"
#include "PCIe.h"
#include "Processor.h"

#include "emb-stdio.h"

#include <mutex>
#include <atomic>
#include <cstring>
#include <algorithm>
#include <array>
#include <vector>

extern uintptr_t GpuMemBase;

HidDevice* AllocateHidPayload();

namespace Usb::Xhci
{

union HcsParams1
{
    struct
    {
        uint32_t MaxDeviceSlots  :  8;
        uint32_t MaxInterrupters : 11;
        uint32_t                 :  5;
        uint32_t MaxPorts        :  8;
    };
    
    uint32_t Raw32;
};

union HcsParams2
{
    struct
    {
        uint32_t IsoSchedThreshold           :  3; // Minimum number of frames (1 ms) or microframes (128 us) to stay ahead.
        uint32_t IsoSchedThresholdIsInFrames :  1; // 1 = frames, 0 = microframes
        uint32_t EventRingSegmentTableMax    :  4;
        uint32_t                             : 13;
        uint32_t MaxScratchpadBuffersHi      :  5;
        uint32_t SaveRestoreUsesScratchpad   :  1;
        uint32_t MaxScratchpadBuffersLo      :  5;
    };
    
    uint32_t Raw32;
};

// XHCI Capability Registers
union CapabilityRegisters
{
    Mmio::Register<uint8_t    const, 0x00> CapLength;
    Mmio::Register<uint16_t   const, 0x02> InterfaceVersion;
    Mmio::Register<HcsParams1 const, 0x04> StructuralParams1;
    Mmio::Register<HcsParams2 const, 0x08> StructuralParams2;
    Mmio::Register<uint32_t   const, 0x0C> StructuralParams3;
    Mmio::Register<uint32_t   const, 0x10> CapabilityParams1;
    Mmio::Register<uint32_t   const, 0x14> DoorbellOffset;
    Mmio::Register<uint32_t   const, 0x18> RuntimeOffset;
};

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
union OperationalRegisters
{
    Mmio::Register<uint32_t, 0x00> UsbCommand;
    Mmio::Register<uint32_t, 0x04> UsbStatus;
    Mmio::Register<uint32_t, 0x08> PageSize;
    Mmio::Register<uint32_t, 0x14> DeviceNotificationControl;
    Mmio::Register<uint64_t, 0x18> CommandRingControl;
    Mmio::Register<uint64_t, 0x30> DeviceContextBaseAddressArrayPointer;
    Mmio::Register<OpConfig, 0x38> Configure;
    
    Mmio::RegisterArray<uint32_t, 0x400, 256, 0x10> PortStatusControl;
};

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
union RuntimeRegisters
{
    // TODO: Implement this
    Mmio::Register<uint32_t, 0x00> MicroframeIndex                 ; // XHCI_RT_MFINDEX   ;  // Microframe Index
    Mmio::Register<uint32_t, 0x20> InterrupterManagement           ; // XHCI_RT_IR0_IMAN  ;  // Interrupter Management
    Mmio::Register<uint32_t, 0x24> InterrupterModeration           ; // XHCI_RT_IR0_IMOD  ;  // Interrupter Moderation
    Mmio::Register<uint32_t, 0x28> EventRingSegmentTableSize       ; // XHCI_RT_IR0_ERSTSZ;  // Event Ring Segment Table Size
    Mmio::Register<uint64_t, 0x30> EventRingSegmentTableBaseAddress; // XHCI_RT_IR0_ERSTBA;  // Event Ring Segment Table Base Address
    Mmio::Register<uint64_t, 0x38> EventRingDequeuePointer         ; // XHCI_RT_IR0_ERDP  ;  // Event Ring Dequeue Pointer
};


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
constexpr uint32_t TRB_TYPE_TRANSFER_EVENT = 32;
constexpr uint32_t TRB_TYPE_CMD_COMPLETION_EVENT = 33;

constexpr uint32_t TRB_CTRL_CYCLE = (1u << 0);
constexpr uint32_t TRB_CTRL_IOC   = (1u << 5);
constexpr uint32_t TRB_CTRL_IDT   = (1u << 6);
constexpr uint32_t TRB_CTRL_CHAIN = (1u << 4);
constexpr uint32_t TRB_CTRL_DIR_IN = (1u << 16);
constexpr uint32_t TRB_CTRL_TC    = (1u << 1);

// TRB completion codes
constexpr uint32_t TRB_CC_SUCCESS         = 1;
constexpr uint32_t TRB_CC_SHORT_PACKET    = 13;

constexpr uint32_t TRB_CC_ENDPOINT_NOT_ENABLED = 12;

// Transfer Request Block structure (16 bytes, 64-byte aligned for rings)
struct alignas(16) TRB {
    uint64_t parameter = 0;
    uint32_t status    = 0;
    uint32_t control   = 0;

    friend constexpr bool operator==(TRB const&, TRB const&) = default;
};

// Event Ring Segment Table Entry
struct alignas(16) EventRingSegment {
    uint64_t base_address;
    uint32_t size;
    uint32_t reserved;

    friend constexpr bool operator==(EventRingSegment const&, EventRingSegment const&) = default;
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

using DoorbellRegisters = Mmio::RegisterArray<uint32_t, 0, 256>;

class Controller : public Usb::Controller
{
public:
    Controller(CapabilityRegisters& capabilityRegisters)
        : capabilityRegisters_ { capabilityRegisters }
        , operationalRegisters_{ *reinterpret_cast<OperationalRegisters*>(reinterpret_cast<uintptr_t>(&capabilityRegisters) + capabilityRegisters.CapLength) }
        , runtimeRegisters_    { *reinterpret_cast<RuntimeRegisters*    >(reinterpret_cast<uintptr_t>(&capabilityRegisters) + (capabilityRegisters.RuntimeOffset.get() & ~0x1Fu)) }
        , doorbellRegisters_   { *reinterpret_cast<DoorbellRegisters*   >(reinterpret_cast<uintptr_t>(&capabilityRegisters) + (capabilityRegisters.DoorbellOffset.get() & ~0x3u)) }
    {
    }

private:
    struct alignas(32) InputControlContext {
        uint32_t drop_context_flags;
        uint32_t add_context_flags;
        uint32_t reserved[5];
        uint32_t configuration_value;
    };

    //PCIe::Bcm2711Driver& pcie_;
    //PCIe::DeviceAddress  devAddress_;

    // Register access pointers
    CapabilityRegisters&  capabilityRegisters_;
    OperationalRegisters& operationalRegisters_;
    RuntimeRegisters&     runtimeRegisters_;
    DoorbellRegisters&    doorbellRegisters_;

    // Device capabilities
    uint32_t max_device_slots = 0;
    uint32_t max_interrupters = 0;
    uint32_t max_ports = 0;
    uint32_t max_scratchpad_buffers = 0;
    
    // DMA allocated memory regions
    std::span<TRB> command_ring{};
    std::span<TRB> event_ring  {};
    std::span<EventRingSegment> event_ring_segment_table{};
    std::span<uint64_t> device_context_base_array{};
    std::span<uint64_t> scratchpad_buffer_array{};
    std::array<void*, 32> scratchpad_buffers{};
    
    // Ring management
    std::atomic<uint32_t> command_ring_enqueue{0};
    std::atomic<uint32_t> command_ring_dequeue{0};
    std::atomic<uint32_t> event_ring_enqueue{0};
    std::atomic<uint32_t> event_ring_dequeue{0};
    bool command_ring_cycle_state = true;
    bool event_ring_cycle_state = true;
    
    // Synchronization
    std::mutex command_mutex;
    std::mutex transfer_mutex;
    std::atomic<bool> interrupt_pending{false};

    // Transfer ring state
    TRB* transfer_ring = nullptr;
    std::atomic<uint32_t> transfer_ring_enqueue{0};
    std::atomic<uint32_t> transfer_ring_dequeue{0};
    bool transfer_ring_cycle_state = true;
    uint8_t* control_dma_buffer = nullptr;
    size_t control_dma_buffer_size = 0;

    // Slot selected for endpoint 0 control transfers.
    std::atomic<uint32_t> default_control_slot_id{0};

    uint32_t controller_hccparams1 = 0;

    // Device slots
    struct DeviceSlot {
        uint32_t slot_id = 0;
        TRB* transfer_ring = nullptr;
        std::atomic<uint32_t> transfer_ring_enqueue{0};
        std::atomic<uint32_t> transfer_ring_dequeue{0};
        bool transfer_ring_cycle_state{true};
        void* input_context = nullptr;
        void* device_context = nullptr;
        std::atomic<bool> in_use{false};
    };
    std::unique_ptr<DeviceSlot[]> device_slots;
    std::vector<DeviceInfo> discovered_devices_;

private:
    // Get physical address for DMA (assuming identity mapping for now)
    uint64_t get_physical_address(void* virtual_addr) const {
        return reinterpret_cast<uintptr_t>(virtual_addr) - GpuMemBase + 0x4'0000'0000ull;
    }

    void set_command_ring_control(uint64_t value) {
        operationalRegisters_.CommandRingControl = value;
    }

    uint64_t get_command_ring_control() const {
        return operationalRegisters_.CommandRingControl;
    }

    void set_dcbaap(uint64_t value) {
       operationalRegisters_.DeviceContextBaseAddressArrayPointer = value;
    }

    void set_erstba(uint64_t value) {
        runtimeRegisters_.EventRingSegmentTableBaseAddress = value;
    }

    uint64_t get_erstba() const {
        return runtimeRegisters_.EventRingSegmentTableBaseAddress;
    }

    void set_erdp(uint64_t value) {
        runtimeRegisters_.EventRingDequeuePointer = value;
    }

    uint64_t get_erdp() const {
        return runtimeRegisters_.EventRingDequeuePointer;
    }
    
    // Ring buffer operations
    uint32_t advance_ring_pointer(uint32_t current, uint32_t ring_size) {
        return (current + 1) % ring_size;
    }

    uint32_t advance_command_ring_pointer(uint32_t current) {
        uint32_t const next = current + 1;
        return (next >= (COMMAND_RING_SIZE - 1)) ? 0u : next;
    }

    uint32_t advance_transfer_ring_pointer(uint32_t current) {
        uint32_t const next = current + 1;
        return (next >= (TRANSFER_RING_SIZE - 1)) ? 0u : next;
    }
    
    void ring_doorbell(uint32_t doorbell, uint32_t target = 0) {
        doorbellRegisters_[doorbell] = target;
    }

    uint32_t context_size_bytes() {
        // HCCPARAMS1[2] = CSZ: 0 -> 32-byte contexts, 1 -> 64-byte contexts.
        return ((controller_hccparams1 >> 2) & 1u) ? 64u : 32u;
    }

    static uint32_t endpoint0_max_packet_size(uint32_t speed) {
        // xHCI PSIV values commonly used on RPi4 VL805.
        switch (speed) {
            case 1: return 8;   // Full speed
            case 2: return 8;   // Low speed
            case 3: return 64;  // High speed
            case 4: return 512; // SuperSpeed
            case 5: return 512; // SuperSpeed+
            default: return 64;
        }
    }

    public:
    bool address_device(uint32_t slot_id, uint32_t port, uint32_t port_speed, uint32_t route, uint32_t parentHubSlotId, uint32_t parentHubPort) {
        printf("XHCI: Addressing device on slot %u, port %u, speed %u\n", slot_id, port, port_speed);

        if (slot_id == 0 || slot_id > max_device_slots) {
            return false;
        }

        if (!ensure_transfer_ring(slot_id)) {
            printf("XHCI: Transfer ring unavailable for Address Device (slot %u)\n", slot_id);
            return false;
        }

        uint32_t const ctx_size = context_size_bytes();
        uint32_t const page_count_for_two_context_pages = 2;

        void* device_context = device_slots[slot_id].device_context;
        if (!device_context) {
            device_context = Mmu::AllocateGpuMemory(page_count_for_two_context_pages);
            if (!device_context) {
                printf("XHCI: Failed to allocate device context for slot %u\n", slot_id);
                return false;
            }
            device_slots[slot_id].device_context = device_context;
        }
        std::memset(device_context, 0, page_count_for_two_context_pages * Mmu::PageSize);

        void* input_context = device_slots[slot_id].input_context;
        if (!input_context) {
            input_context = Mmu::AllocateGpuMemory(page_count_for_two_context_pages);
            if (!input_context) {
                printf("XHCI: Failed to allocate input context for slot %u\n", slot_id);
                return false;
            }
            device_slots[slot_id].input_context = input_context;
        }
        std::memset(input_context, 0, page_count_for_two_context_pages * Mmu::PageSize);

        auto* icc = reinterpret_cast<InputControlContext*>(input_context);
        icc->drop_context_flags = 0;
        icc->add_context_flags = 0x3; // Slot context + EP0 context.

        auto* input_ctx_dw = reinterpret_cast<uint32_t*>(input_context);
        uint32_t const slot_ctx_index = 1;
        uint32_t const ep0_ctx_index  = 2;
        uint32_t* const slot_ctx = input_ctx_dw + (slot_ctx_index * (ctx_size / sizeof(uint32_t)));
        uint32_t* const ep0_ctx  = input_ctx_dw + (ep0_ctx_index  * (ctx_size / sizeof(uint32_t)));
        auto& slot = device_slots[slot_id];

        // Slot Context
        // DW0: Speed[23:20], Context Entries[31:27]
        slot_ctx[0] = ((port_speed & 0xF) << 20) | (1u << 27) | (route & 0xF'FFFFu);
        // DW1: Root Hub Port Number[23:16]
        slot_ctx[1] = ((port & 0xFF) << 16);
        // DW2: TT Hub Slot ID[23:16], TT Port Number[31:24]
        slot_ctx[2] = (parentHubSlotId & 0xFF) | ((parentHubPort & 0xFF) << 8);

        // Endpoint 0 Context (DCI 1)
        uint32_t const mps = endpoint0_max_packet_size(port_speed);
        // DW1: EP Type[5:3]=4(Control), Max Packet Size[31:16], Max Burst Size[15:8]=0, Error Count[2:1]=3
        ep0_ctx[1] = (4u << 3) | (3u << 1) | ((mps & 0xFFFF) << 16);
        // Dequeue Pointer + DCS. Use current producer position rather than force-resetting
        // ring state, so shared ring bookkeeping stays coherent.
        uint32_t ep0_ring_index = slot.transfer_ring_enqueue.load();
        if (ep0_ring_index >= (TRANSFER_RING_SIZE - 1)) {
            ep0_ring_index = 0;
        }
        uint64_t const tr_dequeue = get_physical_address(&slot.transfer_ring[ep0_ring_index]) |
                                    (slot.transfer_ring_cycle_state ? 1u : 0u);
        ep0_ctx[2] = static_cast<uint32_t>(tr_dequeue & 0xFFFF'FFFFu);
        ep0_ctx[3] = static_cast<uint32_t>(tr_dequeue >> 32);
        // Average TRB Length in DW4 lower 16 bits.
        ep0_ctx[4] = 8;

        uint64_t const device_context_phys = get_physical_address(device_context);
        device_context_base_array[slot_id] = device_context_phys;

        Processor::FlushDataCache(device_context, page_count_for_two_context_pages * Mmu::PageSize);
        Processor::FlushDataCache(input_context, page_count_for_two_context_pages * Mmu::PageSize);
        Processor::FlushDataCache(&device_context_base_array[slot_id], sizeof(device_context_base_array[slot_id]));

        TRB address_device_cmd{};
        address_device_cmd.parameter = get_physical_address(input_context);
        address_device_cmd.status = 0;
        address_device_cmd.control = (TRB_TYPE_ADDRESS_DEV << 10) | (slot_id << 24);

        if (!send_command(address_device_cmd)) {
            printf("XHCI: Failed to submit Address Device for slot %u\n", slot_id);
            return false;
        }

        uint32_t completed_slot = 0;
        if (!wait_for_command_completion(completed_slot) || completed_slot != slot_id) {
            printf("XHCI: Address Device did not complete for slot %u (completed slot %u)\n", slot_id, completed_slot);
            return false;
        }

        printf("XHCI: Address Device completed for slot %u (port %u speed %u MPS %u)\n", slot_id, port, port_speed, mps);
        return true;
    }
    private:

    bool setup_rings() {
        printf("XHCI: Setting up rings...\n");
        
        // Allocate command ring (64-byte aligned)
        command_ring = { Mmu::AllocateGpuMemory<TRB>((COMMAND_RING_SIZE * sizeof(TRB) + Mmu::PageSize - 1) / Mmu::PageSize), COMMAND_RING_SIZE };
        if (!command_ring.data()) {
            command_ring = {};
            printf("XHCI: Failed to allocate command ring\n");
            return false;
        }
        std::fill(command_ring.begin(), command_ring.end(), TRB{});
        // Command ring is a segmented ring and requires a terminal Link TRB.
        command_ring[COMMAND_RING_SIZE - 1].parameter = get_physical_address(command_ring.data());
        command_ring[COMMAND_RING_SIZE - 1].status = 0;
        command_ring[COMMAND_RING_SIZE - 1].control =
            (TRB_TYPE_LINK << 10) |
            TRB_CTRL_CYCLE |
            TRB_CTRL_TC;
        Processor::FlushDataCache(command_ring.data(), command_ring.size_bytes());
        
        // Allocate event ring
        event_ring = { Mmu::AllocateGpuMemory<TRB>((EVENT_RING_SIZE * sizeof(TRB) + Mmu::PageSize - 1) / Mmu::PageSize), EVENT_RING_SIZE };
        if (!event_ring.data()) {
            event_ring = {};
            printf("XHCI: Failed to allocate event ring\n");
            return false;
        }
        std::fill(event_ring.begin(), event_ring.end(), TRB{});
        Processor::FlushDataCache(event_ring.data(), event_ring.size_bytes());
        
        // Allocate event ring segment table
        event_ring_segment_table = { Mmu::AllocateGpuMemory<EventRingSegment>(1), 1 };
        if (!event_ring_segment_table.data()) {
            event_ring_segment_table = {};
            printf("XHCI: Failed to allocate event ring segment table\n");
            return false;
        }
        
        // Setup event ring segment table
        event_ring_segment_table[0].base_address = get_physical_address(event_ring.data());
        event_ring_segment_table[0].size         = static_cast<uint32_t>(event_ring.size());
        event_ring_segment_table[0].reserved     = 0;
        Processor::FlushDataCache(event_ring_segment_table.data(), event_ring_segment_table.size_bytes());
        
        // Allocate device context base array
        uint32_t dcbaa_size = (max_device_slots + 1) * sizeof(uint64_t);
        device_context_base_array = { Mmu::AllocateGpuMemory<uint64_t>((dcbaa_size + Mmu::PageSize - 1) / Mmu::PageSize), max_device_slots + 1 };
        if (!device_context_base_array.data()) {
            device_context_base_array = {};
            printf("XHCI: Failed to allocate device context base array\n");
            return false;
        }
        std::fill(device_context_base_array.begin(), device_context_base_array.end(), 0);
        Processor::FlushDataCache(device_context_base_array.data(), device_context_base_array.size_bytes());

        if (max_scratchpad_buffers > 0) {
            if (max_scratchpad_buffers > scratchpad_buffers.size()) {
                printf("XHCI: Scratchpad count %u exceeds supported max %zu\n",
                       max_scratchpad_buffers,
                       scratchpad_buffers.size());
                return false;
            }

            scratchpad_buffer_array = {
                Mmu::AllocateGpuMemory<uint64_t>((max_scratchpad_buffers * sizeof(uint64_t) + Mmu::PageSize - 1) / Mmu::PageSize),
                max_scratchpad_buffers
            };
            if (!scratchpad_buffer_array.data()) {
                scratchpad_buffer_array = {};
                printf("XHCI: Failed to allocate scratchpad buffer pointer array\n");
                return false;
            }

            std::fill(scratchpad_buffer_array.begin(), scratchpad_buffer_array.end(), 0);
            scratchpad_buffers.fill(nullptr);

            for (uint32_t i = 0; i < max_scratchpad_buffers; ++i) {
                void* const scratch = Mmu::AllocateGpuMemory(1);
                if (!scratch) {
                    printf("XHCI: Failed to allocate scratchpad buffer %u\n", i);
                    return false;
                }
                scratchpad_buffers[i] = scratch;
                std::memset(scratch, 0, Mmu::PageSize);
                Processor::FlushDataCache(scratch, Mmu::PageSize);
                scratchpad_buffer_array[i] = get_physical_address(scratch);
            }

            Processor::FlushDataCache(scratchpad_buffer_array.data(), scratchpad_buffer_array.size_bytes());
            device_context_base_array[0] = get_physical_address(scratchpad_buffer_array.data());
            Processor::FlushDataCache(&device_context_base_array[0], sizeof(device_context_base_array[0]));
            printf("XHCI: Programmed %u scratchpad buffers\n", max_scratchpad_buffers);
        }
        
        // Setup command ring control register
        set_command_ring_control(get_physical_address(command_ring.data()) | 1); // Set ring cycle state
        
        // Setup device context base address array pointer
        set_dcbaap(get_physical_address(device_context_base_array.data()));
        
        // Setup event ring
        runtimeRegisters_.EventRingSegmentTableSize        = 1; // One segment
        set_erstba(get_physical_address(event_ring_segment_table.data()));
        set_erdp(get_physical_address(event_ring.data()));
        
        // Enable interrupter
        runtimeRegisters_.InterrupterManagement = 2; // Interrupt Enable (IE)
        runtimeRegisters_.InterrupterModeration = 0x00004000; // 1ms moderation
        
        printf("XHCI: Rings setup complete\n");
        return true;
    }
    
    void process_events() {
        while (true) {
            TRB* event = &event_ring[event_ring_dequeue.load()];
            Processor::InvalidateDataCache(event, sizeof(TRB));
            
            // Check if this TRB belongs to current cycle
            bool cycle_bit = (event->control & 1) != 0;
            if (cycle_bit != event_ring_cycle_state) {
                break; // No more events
            }

            // Process the event
            uint32_t trb_type = (event->control >> 10) & 0x3F;
            uint32_t completion_code = (event->status >> 24) & 0xFF;
            
            printf("XHCI: Event TRB type=%u completion=%u\n", trb_type, completion_code);
            
            // Advance dequeue pointer
            uint32_t new_dequeue = advance_ring_pointer(event_ring_dequeue.load(), EVENT_RING_SIZE);
            event_ring_dequeue.store(new_dequeue);
            
            // Check for ring wrap
            if (new_dequeue == 0) {
                event_ring_cycle_state = !event_ring_cycle_state;
            }
        }
        
        // Update event ring dequeue pointer register
        set_erdp(get_physical_address(&event_ring[event_ring_dequeue.load()]) | (1ull << 3));
    }
    
    static Exception::Spark xhci_interrupt_handler() {
        // This would be called by the interrupt system
        // For now, just a placeholder
        printf("XHCI: Interrupt received\n");

        return {};
    }

    bool ensure_transfer_ring(uint32_t slot_id) {
        if (slot_id == 0 || slot_id > max_device_slots) {
            return false;
        }

        auto& slot = device_slots[slot_id];
        if (slot.transfer_ring != nullptr) {
            return true;
        }

        slot.transfer_ring = Mmu::AllocateGpuMemory<TRB>((TRANSFER_RING_SIZE * sizeof(TRB) + Mmu::PageSize - 1) / Mmu::PageSize);
        if (!slot.transfer_ring) {
            printf("XHCI: Failed to allocate transfer ring for slot %u\n", slot_id);
            return false;
        }

        std::memset(slot.transfer_ring, 0, TRANSFER_RING_SIZE * sizeof(TRB));
        slot.transfer_ring[TRANSFER_RING_SIZE - 1].parameter = get_physical_address(slot.transfer_ring);
        slot.transfer_ring[TRANSFER_RING_SIZE - 1].status = 0;
        slot.transfer_ring[TRANSFER_RING_SIZE - 1].control =
            (TRB_TYPE_LINK << 10) |
            TRB_CTRL_CYCLE |
            TRB_CTRL_TC;
        slot.transfer_ring_enqueue.store(0);
        slot.transfer_ring_dequeue.store(0);
        slot.transfer_ring_cycle_state = true;
        Processor::FlushDataCache(slot.transfer_ring, TRANSFER_RING_SIZE * sizeof(TRB));
        return true;
    }

    bool ensure_control_dma_buffer(size_t size) {
        if (size == 0) {
            return true;
        }

        if (control_dma_buffer && control_dma_buffer_size >= size) {
            return true;
        }

        uint32_t const pages = static_cast<uint32_t>((size + Mmu::PageSize - 1) / Mmu::PageSize);
        control_dma_buffer = static_cast<uint8_t*>(Mmu::AllocateGpuMemory(pages));
        if (!control_dma_buffer) {
            control_dma_buffer_size = 0;
            printf("XHCI: Failed to allocate control DMA buffer (%zu bytes)\n", size);
            return false;
        }

        control_dma_buffer_size = static_cast<size_t>(pages) * Mmu::PageSize;
        std::memset(control_dma_buffer, 0, control_dma_buffer_size);
        Processor::FlushDataCache(control_dma_buffer, control_dma_buffer_size);
        return true;
    }

    Status wait_for_transfer_event(uint32_t slot_id, uint32_t timeout_ms = 1000) {
        if (event_ring.empty())
        {
            printf("XHCI: Event ring is not initialized\n");
            return Status::Error;
        }

        auto const start_time = Cpu::GetPerformanceCounter();
        auto const timeout_ticks = Cpu::GetPerformanceTicksForMs(timeout_ms);

        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            uint32_t const usbsts = operationalRegisters_.UsbStatus;
            if (usbsts & XHCI_STS_HSE) {
                printf("XHCI: Host system error while waiting for transfer event\n");
                printf("XHCI: USBCMD=0x%08X USBSTS=0x%08X CRCR=0x%016llX ERSTBA=0x%016llX ERDP=0x%016llX\n",
                       operationalRegisters_.UsbCommand.get(),
                       usbsts,
                       get_command_ring_control(),
                       get_erstba(),
                       get_erdp());
                return Status::Error;
            }

            TRB* event = &event_ring[event_ring_dequeue.load()];
            Processor::InvalidateDataCache(event, sizeof(TRB));
            bool const cycle_bit = (event->control & TRB_CTRL_CYCLE) != 0;
            if (cycle_bit != event_ring_cycle_state) {
                Cpu::DelayInMicroseconds(50);
                continue;
            }

            printf("XHCI: Event received: Control=0x%08X Status=0x%08X Parameter=0x%016llX\n", event->control, event->status, event->parameter);

            uint32_t const trb_type        = (event->control >> 10) & 0x3F;
            uint32_t const completion_code = (event->status >> 24) & 0xFF;
            uint32_t const event_slot_id   = (event->control >> 24) & 0xFF;

            uint32_t const new_dequeue = advance_ring_pointer(event_ring_dequeue.load(), EVENT_RING_SIZE);
            event_ring_dequeue.store(new_dequeue);
            if (new_dequeue == 0) {
                event_ring_cycle_state = !event_ring_cycle_state;
            }

            set_erdp(get_physical_address(&event_ring[event_ring_dequeue.load()]) | (1ull << 3));

            if (event_slot_id != 0 && slot_id != 0 && event_slot_id != slot_id) {
                continue;
            }

            if (trb_type == TRB_TYPE_TRANSFER_EVENT) {
                if (completion_code == TRB_CC_SUCCESS || completion_code == TRB_CC_SHORT_PACKET) {
                    return Status::Success;
                }

                if (completion_code == TRB_CC_ENDPOINT_NOT_ENABLED) {
                    printf("XHCI: Transfer event failed, completion=%u (endpoint not enabled)\n", completion_code);
                    return Status::Error;
                }

                printf("XHCI: Transfer event failed, completion=%u\n", completion_code);
                return Status::Error;
            }

            if (trb_type == TRB_TYPE_CMD_COMPLETION_EVENT) {
                if (completion_code != TRB_CC_SUCCESS) {
                    printf("XHCI: Command completion failed, completion=%u\n", completion_code);
                }
                continue;
            }

            printf("XHCI: Ignoring event type=%u completion=%u\n", trb_type, completion_code);
        }

        printf("XHCI: Timed out waiting for transfer completion event\n");
        size_t i = 0;
        for (auto& trb : event_ring) {
            if (trb != TRB{}) {
                printf("Event TRB[%zu]: parameter=0x%016llX status=0x%08X control=0x%08X\n", i, trb.parameter, trb.status, trb.control);
            }
            ++i;
        }
        return Status::Timeout;
    }

    bool wait_for_command_completion(uint32_t& slot_id, uint32_t timeout_ms = 1000) {
        auto const start_time = Cpu::GetPerformanceCounter();
        auto const timeout_ticks = Cpu::GetPerformanceTicksForMs(timeout_ms);

        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            uint32_t const usbsts = operationalRegisters_.UsbStatus;
            if (usbsts & XHCI_STS_HSE) {
                printf("XHCI: Host system error while waiting for command completion\n");
                printf("XHCI: USBCMD=0x%08X USBSTS=0x%08X CRCR=0x%016llX ERSTBA=0x%016llX ERDP=0x%016llX\n",
                       operationalRegisters_.UsbCommand.get(),
                       usbsts,
                       get_command_ring_control(),
                       get_erstba(),
                       get_erdp());
                return false;
            }

            TRB* event = &event_ring[event_ring_dequeue.load()];
            Processor::InvalidateDataCache(event, sizeof(TRB));

            bool const cycle_bit = (event->control & TRB_CTRL_CYCLE) != 0;
            if (cycle_bit != event_ring_cycle_state) {
                Cpu::DelayInMicroseconds(50);
                continue;
            }

            printf("XHCI: Event received: Control=0x%08X Status=0x%08X Parameter=0x%016llX\n", event->control, event->status, event->parameter);

            uint32_t const trb_type        = (event->control >> 10) & 0x3F;
            uint32_t const completion_code = (event->status >> 24) & 0xFF;
            uint32_t const event_slot_id   = (event->control >> 24) & 0xFF;

            uint32_t const new_dequeue = advance_ring_pointer(event_ring_dequeue.load(), EVENT_RING_SIZE);
            event_ring_dequeue.store(new_dequeue);
            if (new_dequeue == 0) {
                event_ring_cycle_state = !event_ring_cycle_state;
            }
            set_erdp(get_physical_address(&event_ring[event_ring_dequeue.load()]) | (1ull << 3));

            if (trb_type != TRB_TYPE_CMD_COMPLETION_EVENT) {
                continue;
            }

            if (completion_code != TRB_CC_SUCCESS) {
                printf("XHCI: Command completion failed, completion=%u\n", completion_code);
                return false;
            }

            slot_id = event_slot_id;
            return true;
        }

        printf("XHCI: Timed out waiting for command completion event\n");
         printf("XHCI: USBSTS=0x%08X IMAN=0x%08X ERDP=0x%016llX\n",
             operationalRegisters_.UsbStatus.get(),
             runtimeRegisters_.InterrupterManagement.get(),
             get_erdp());
        return false;
    }
    
    bool send_command(TRB command_trb) {
        std::lock_guard<std::mutex> lock(command_mutex);

        uint32_t const usbsts_before = operationalRegisters_.UsbStatus;
        uint32_t usbcmd_before = operationalRegisters_.UsbCommand;
        if (usbsts_before & XHCI_STS_HSE) {
            printf("XHCI: Refusing command submit while HSE is set (USBSTS=0x%08X)\n", usbsts_before);
            return false;
        }

        if (usbsts_before & XHCI_STS_HCH) {
            printf("XHCI: Controller halted before command submit, attempting restart\n");
            operationalRegisters_.UsbCommand |= XHCI_CMD_RUN;
            Cpu::DelayInMicroseconds(100);
            usbcmd_before = operationalRegisters_.UsbCommand;
        }
        
        uint32_t enqueue_pos = command_ring_enqueue.load();
        
        // Set cycle bit
        command_trb.control |= command_ring_cycle_state ? 1 : 0;
        
        // Copy command to ring
        command_ring[enqueue_pos] = command_trb;
        Processor::FlushDataCache(&command_ring[enqueue_pos], sizeof(TRB));
        
        // Advance enqueue pointer
        uint32_t new_enqueue = advance_command_ring_pointer(enqueue_pos);
        command_ring_enqueue.store(new_enqueue);
        
        // Check for ring wrap
        if (new_enqueue == 0) {
            command_ring_cycle_state = !command_ring_cycle_state;
        }
        
        // Ring doorbell 0 (command ring)
        ring_doorbell(0, 0);

         printf("XHCI: Command submitted Control=0x%08X Status=0x%08X Parameter=0x%016llX (USBCMD=0x%08X USBSTS=0x%08X CRCR=0x%016llX)\n",
            command_trb.control,
            command_trb.status,
            command_trb.parameter,
            usbcmd_before,
            usbsts_before,
            get_command_ring_control());
        
        return true;
    }
    
    uint32_t allocate_device_slot() override
    {

        if (!send_command(TRB{ .control = TRB_TYPE_ENABLE_SLOT << 10 }))
        {
            return 0;
        }
        printf("XHCI: Enable slot command sent\n");

        uint32_t slot_id = 0;
        if (!wait_for_command_completion(slot_id) || slot_id == 0) {
            printf("XHCI: Enable slot command did not complete successfully\n");
            return false;
        }

        printf("XHCI: Enabled slot %u\n", slot_id);

        bool expected = false;
        if (device_slots[slot_id].in_use.compare_exchange_strong(expected, true))
         {
            device_slots[slot_id].slot_id = slot_id;
            return slot_id;
        }

        printf("XHCI: Failed to enable slot %u in the slot table\n", slot_id);

        return 0; // No available slots
    }
    
    void free_device_slot(uint32_t slot_id) {
        if (slot_id > 0 && slot_id <= max_device_slots) {
            device_slots[slot_id].in_use.store(false);
            device_slots[slot_id].slot_id = 0;
        }
    }
    
    void remember_discovered_device(uint32_t slot_id, uint32_t port, uint32_t root_hub_port, uint32_t speed,
                                     std::span<uint8_t const> descriptor,
                                     std::span<uint8_t const> config_descriptor)
    {
        if (descriptor.empty()) {
            return;
        }

        DeviceInfo info{};
        info.SlotId = slot_id;
        info.Port = port;
        info.RootHubPort = root_hub_port;
        info.Speed = speed;
        info.HasConfiguration = !config_descriptor.empty();

        if (descriptor.size() >= sizeof(DeviceDescriptor)) {
            std::memcpy(&info.Descriptor, descriptor.data(), sizeof(DeviceDescriptor));
        }
        else {
            std::memset(&info.Descriptor, 0, sizeof(DeviceDescriptor));
        }

        if (config_descriptor.size() >= sizeof(ConfigurationDescriptor)) {
            std::memcpy(&info.Configuration, config_descriptor.data(), sizeof(ConfigurationDescriptor));
        }

        if (config_descriptor.size() >= 2) {
            size_t offset = 0;
            while (offset + 2 <= config_descriptor.size()) {
                uint8_t const length = config_descriptor[offset];
                if (length == 0) {
                    break;
                }

                uint8_t const descriptor_type = config_descriptor[offset + 1];
                if (descriptor_type == USB_DESCRIPTOR_TYPE_INTERFACE && length >= sizeof(UsbInterfaceDescriptor)) {
                    UsbInterfaceDescriptor interface{};
                    std::memcpy(&interface, config_descriptor.data() + offset, sizeof(UsbInterfaceDescriptor));
                    if (interface.Number >= info.Interfaces.size()) {
                        info.Interfaces.resize(interface.Number + 1);
                        info.Endpoints.resize(interface.Number + 1);
                    }
                    info.Interfaces[interface.Number] = interface;
                }
                else if (descriptor_type == USB_DESCRIPTOR_TYPE_ENDPOINT && length >= sizeof(UsbEndpointDescriptor)) {
                    UsbEndpointDescriptor endpoint{};
                    std::memcpy(&endpoint, config_descriptor.data() + offset, sizeof(UsbEndpointDescriptor));
                    if (endpoint.EndpointAddress.Number >= info.Endpoints.size()) {
                        info.Endpoints.resize(endpoint.EndpointAddress.Number + 1);
                    }
                    info.Endpoints[endpoint.EndpointAddress.Number].push_back(endpoint);
                }

                offset += length;
                if (offset >= config_descriptor.size()) {
                    break;
                }
            }
        }

        discovered_devices_.push_back(info);
    }

    bool wait_for_ready(uint32_t timeout_ms = 1000) {
        auto start_time = Cpu::GetPerformanceCounter();
        auto timeout_ticks = Cpu::GetPerformanceTicksForMs(timeout_ms);
        
        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            uint32_t status = operationalRegisters_.UsbStatus;
            if (!(status & XHCI_STS_CNR)) {
                return true;
            }
        }
        return false;
    }

    bool wait_for_controller_running(uint32_t timeout_ms = 1000) {
        auto start_time = Cpu::GetPerformanceCounter();
        auto timeout_ticks = Cpu::GetPerformanceTicksForMs(timeout_ms);

        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            uint32_t const status = operationalRegisters_.UsbStatus;
            bool const controller_not_ready = (status & XHCI_STS_CNR) != 0;
            bool const halted = (status & XHCI_STS_HCH) != 0;
            if (!controller_not_ready && !halted) {
                return true;
            }
            Cpu::DelayInMicroseconds(50);
        }
        return false;
    }

    void clear_usb_status_bits() {
        uint32_t const sticky = operationalRegisters_.UsbStatus.get() &
                                (XHCI_STS_HSE | XHCI_STS_EINT | XHCI_STS_PCD | XHCI_STS_SSS | XHCI_STS_RSS | XHCI_STS_SRE | XHCI_STS_HCE);
        if (sticky != 0) {
            operationalRegisters_.UsbStatus = sticky;
        }
    }
    
    bool reset_controller() {
        auto start_time = Cpu::GetPerformanceCounter();
        auto timeout_ticks = Cpu::GetPerformanceTicksForMs(5000);
        
        uint32_t status = 0;
        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            status = operationalRegisters_.UsbStatus;
            if (!(status & XHCI_STS_CNR)) {
                break;
            }
        }

        if (status & XHCI_STS_CNR) {
            printf("Timed out waiting for ready state. Command: 0x%X, Status: 0x%X\n", operationalRegisters_.UsbCommand.get(), operationalRegisters_.UsbStatus.get());
        }

        printf("Command: 0x%X, Status: 0x%X\n", operationalRegisters_.UsbCommand.get(), operationalRegisters_.UsbStatus.get());

        printf("XHCI: Resetting controller...\n");
        
        // Stop the controller first
        operationalRegisters_.UsbCommand &= ~XHCI_CMD_RUN;
        
        // Wait for halt
        start_time = Cpu::GetPerformanceCounter();
        timeout_ticks = Cpu::GetPerformanceTicksForMs(5000);
        
        status = 0;
        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            status = operationalRegisters_.UsbStatus;
            if (status & XHCI_STS_HCH) {
                break;
            }
        }
        
        printf("XHCI: Controller halted...\n");

        // Reset the controller
        operationalRegisters_.UsbCommand |= XHCI_CMD_HCRST;
        
        // Wait for reset to complete
        uint32_t cmd = 0;
        start_time = Cpu::GetPerformanceCounter();
        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            cmd = operationalRegisters_.UsbCommand;
            if (!(cmd & XHCI_CMD_HCRST)) {
                break;
            }
        }
        
        if (cmd & XHCI_CMD_HCRST) {
            printf("XHCI: Reset timeout\n");
            printf("Command: 0x%X, Status: 0x%X\n", cmd, status);
            return false;
        }
        
        return wait_for_ready();
    }

public:
    Status initialize() override {
        printf("XHCI: Initializing controller...\n");
        
        // Test memory access first
        printf("XHCI: Testing memory access...\n");
        uint32_t test_value = *reinterpret_cast<uint32_t volatile*>(&capabilityRegisters_);
        printf("XHCI: First word: 0x%X\n", test_value);
        
        if (test_value == 0xdeaddead || test_value == 0xffffffff || test_value == 0x00000000) {
            printf("XHCI: Invalid response, device not accessible\n");
            return Status::Error;
        }

        // Read capability registers
        HcsParams1 hcsparams1        = capabilityRegisters_.StructuralParams1;
        HcsParams2 hcsparams2        = capabilityRegisters_.StructuralParams2;
        uint32_t   hccparams1        = capabilityRegisters_.CapabilityParams1;

        printf("XHCI: Capability length: 0x%X\n", capabilityRegisters_.CapLength.get());
        printf("XHCI: HCI Version: 0x%X\n", capabilityRegisters_.InterfaceVersion.get());

        // Extract parameters
        max_device_slots = hcsparams1.MaxDeviceSlots;
        max_interrupters = hcsparams1.MaxInterrupters;
        max_ports        = hcsparams1.MaxPorts;

        printf("XHCI: Max device slots: %u\n", hcsparams1.MaxDeviceSlots);
        printf("XHCI: Max interrupters: %u\n", hcsparams1.MaxInterrupters);
        printf("XHCI: Max ports: %u\n"       , hcsparams1.MaxPorts);

        printf("XHCI: Isochronous scheduling threshold: %u %s\n", hcsparams2.IsoSchedThreshold, hcsparams2.IsoSchedThresholdIsInFrames ? "frames" : "microframes");
        printf("XHCI: Event ring segment table max: %u\n", hcsparams2.EventRingSegmentTableMax);
        max_scratchpad_buffers = (hcsparams2.MaxScratchpadBuffersHi << 5) + hcsparams2.MaxScratchpadBuffersLo;
        printf("XHCI: Max scratchpad buffers: %u\n", max_scratchpad_buffers);
        printf("XHCI: Save/restore uses scratchpad: %u\n", hcsparams2.SaveRestoreUsesScratchpad);
        printf("XHCI: Doorbells offset: 0x%X\n", capabilityRegisters_.DoorbellOffset.get());
        printf("XHCI: Runtime offset: 0x%X\n", capabilityRegisters_.RuntimeOffset.get());
        printf("\n");

        printf("Command: 0x%X, Status: 0x%X\n", operationalRegisters_.UsbCommand.get(), operationalRegisters_.UsbStatus.get());

        // Allocate device slots array
        device_slots = std::make_unique<DeviceSlot[]>(max_device_slots + 1);
        
        // Reset the controller
        if (!reset_controller()) {
            printf("XHCI: Controller reset failed\n");
            return Status::Error;
        }
        
        // Setup memory structures
        if (!setup_rings()) {
            printf("XHCI: Ring setup failed\n");
            return Status::Error;
        }

        controller_hccparams1 = hccparams1;
        
        // Set number of device slots
        operationalRegisters_.Configure = [&](auto& reg){ reg.MaxDeviceSlotsEnabled = max_device_slots; };

        // Clear sticky status events before enabling run/interrupts.
        clear_usb_status_bits();
        
        // Enable interrupts
        operationalRegisters_.UsbCommand |= XHCI_CMD_INTE;
        
        // Register interrupt handler
        // TODO: Interrupts::EnableXhci(xhci_interrupt_handler);
        
        // Start the controller
        operationalRegisters_.UsbCommand |= XHCI_CMD_RUN;
        
        // Wait for controller to start and leave halted state.
        if (!wait_for_controller_running()) {
            printf("XHCI: Controller did not enter running state\n");
            printf("XHCI: USBCMD=0x%08X USBSTS=0x%08X\n",
                   operationalRegisters_.UsbCommand.get(),
                   operationalRegisters_.UsbStatus.get());
            return Status::Error;
        }
        
        printf("XHCI: Controller initialized successfully\n");
        
        // Start device enumeration
        enumerate_devices();
        
        return Status::Success;
    }

    Status shutdown() override {
        printf("XHCI: Shutting down controller...\n");
        
        // Stop the controller
        operationalRegisters_.UsbCommand &= ~XHCI_CMD_RUN;
        
        // Wait for halt
        auto start_time = Cpu::GetPerformanceCounter();
        auto timeout_ticks = Cpu::GetPerformanceTicksForMs(1000);
        
        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks) {
            uint32_t status = operationalRegisters_.UsbStatus;
            if (status & XHCI_STS_HCH) {
                printf("XHCI: Controller halted\n");
                return Status::Success;
            }
        }
        
        printf("XHCI: Shutdown timeout\n");
        return Status::Timeout;
    }

    Status read(uint8_t endpoint, std::span<uint8_t> buffer) override {
        printf("XHCI: Read from endpoint %u (buffer size: %zu bytes)\n", endpoint, buffer.size());

        if (buffer.empty()) {
            return Status::Error;
        }

        uint32_t const slot_id = default_control_slot_id.load();
        if (slot_id == 0 || slot_id > max_device_slots) {
            return Status::Error;
        }

        if (!ensure_transfer_ring(slot_id)) {
            return Status::Error;
        }

        if (!ensure_control_dma_buffer(buffer.size())) {
            return Status::Error;
        }

        std::memset(control_dma_buffer, 0, buffer.size());
        Processor::FlushDataCache(control_dma_buffer, buffer.size());

        auto& slot = device_slots[slot_id];

        std::lock_guard<std::mutex> lock(transfer_mutex);
        uint32_t const position = slot.transfer_ring_enqueue.load();
        TRB& trb = slot.transfer_ring[position];
        trb.parameter = get_physical_address(control_dma_buffer);
        trb.status = static_cast<uint32_t>(buffer.size());
        trb.control =
            (TRB_TYPE_NORMAL << 10) |
            TRB_CTRL_IOC |
            TRB_CTRL_DIR_IN |
            (slot.transfer_ring_cycle_state ? TRB_CTRL_CYCLE : 0) |
            (static_cast<uint32_t>(endpoint) << 16);
        Processor::FlushDataCache(&trb, sizeof(TRB));

        uint32_t next = advance_transfer_ring_pointer(position);
        slot.transfer_ring_enqueue.store(next);
        if (next == 0) {
            slot.transfer_ring_cycle_state = !slot.transfer_ring_cycle_state;
        }

        ring_doorbell(slot_id, endpoint);
        Status const transfer_status = wait_for_transfer_event(slot_id);
        if (transfer_status == Status::Success) {
            Processor::InvalidateDataCache(control_dma_buffer, buffer.size());
            std::memcpy(buffer.data(), control_dma_buffer, buffer.size());
        }
        return transfer_status;
    }

    Status write(uint8_t endpoint, std::span<uint8_t const> data) override {
        printf("XHCI: Write to endpoint %u (data size: %zu bytes)\n", endpoint, data.size());

        if (data.empty()) {
            return Status::Error;
        }

        uint32_t const slot_id = default_control_slot_id.load();
        if (slot_id == 0 || slot_id > max_device_slots) {
            return Status::Error;
        }

        if (!ensure_transfer_ring(slot_id)) {
            return Status::Error;
        }

        if (!ensure_control_dma_buffer(data.size())) {
            return Status::Error;
        }

        std::memcpy(control_dma_buffer, data.data(), data.size());
        Processor::FlushDataCache(control_dma_buffer, data.size());

        auto& slot = device_slots[slot_id];

        std::lock_guard<std::mutex> lock(transfer_mutex);
        uint32_t const position = slot.transfer_ring_enqueue.load();
        TRB& trb = slot.transfer_ring[position];
        trb.parameter = get_physical_address(control_dma_buffer);
        trb.status = static_cast<uint32_t>(data.size());
        trb.control =
            (TRB_TYPE_NORMAL << 10) |
            TRB_CTRL_IOC |
            (slot.transfer_ring_cycle_state ? TRB_CTRL_CYCLE : 0) |
            (static_cast<uint32_t>(endpoint) << 16);
        Processor::FlushDataCache(&trb, sizeof(TRB));

        uint32_t next = advance_transfer_ring_pointer(position);
        slot.transfer_ring_enqueue.store(next);
        if (next == 0) {
            slot.transfer_ring_cycle_state = !slot.transfer_ring_cycle_state;
        }

        ring_doorbell(slot_id, endpoint);
        return wait_for_transfer_event(slot_id);
    }

    Status controlTransfer(uint8_t slotId, uint8_t requestType, uint8_t request,
                           uint16_t value, uint16_t index,
                           std::span<uint8_t> data = {}) override {
        printf("XHCI: Control transfer on slot %u - Type: 0x%02X, Request: 0x%02X, Value: 0x%04X, Index: 0x%04X, Length: %zu\n",
               slotId, requestType, request, value, index, data.size()
        );

        if (!ensure_transfer_ring(slotId)) {
            return Status::Error;
        }

        if (!ensure_control_dma_buffer(data.size())) {
            return Status::Error;
        }

        uint32_t slot_id = slotId;
        if (slot_id == 0) {
            slot_id = default_control_slot_id.load();
        }
        if (slot_id == 0) {
            slot_id = 1;
        }
        if (slot_id == 0 || slot_id > max_device_slots) {
            printf("XHCI: Invalid device slot\n");
            return Status::Error;
        }

        uint16_t const transfer_length    = static_cast<uint16_t>(data.size());
        bool     const data_stage_present = transfer_length != 0;
        bool     const data_stage_in      = (requestType & 0x80) != 0;
        uint8_t* const dma_data_ptr       = data_stage_present ? control_dma_buffer : nullptr;

        if (data_stage_present && !data_stage_in) {
            std::memcpy(dma_data_ptr, data.data(), data.size());
            Processor::FlushDataCache(dma_data_ptr, data.size());
        }

        uint32_t trt = 0;
        if (data_stage_present) {
            trt = data_stage_in ? 3u : 2u;
        }

        uint64_t const setup_packet_data =
            static_cast<uint64_t>(requestType) |
            (static_cast<uint64_t>(request) << 8) |
            (static_cast<uint64_t>(value) << 16) |
            (static_cast<uint64_t>(index) << 32) |
            (static_cast<uint64_t>(transfer_length) << 48);

        auto& slot = device_slots[slot_id];

        {
            std::lock_guard<std::mutex> lock(transfer_mutex);

            uint32_t setup_pos = slot.transfer_ring_enqueue.load();
            TRB& setup_trb = slot.transfer_ring[setup_pos];
            setup_trb.parameter = setup_packet_data;
            setup_trb.status = 8;
            setup_trb.control =
                (TRB_TYPE_SETUP << 10) |
                (trt << 16) |
                TRB_CTRL_CHAIN |
                TRB_CTRL_IDT |
                (slot.transfer_ring_cycle_state ? TRB_CTRL_CYCLE : 0);
            printf("XHCI: Setup TRB - 0x%016X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", &setup_trb,
                reinterpret_cast<uint8_t*>(&setup_trb)[0], reinterpret_cast<uint8_t*>(&setup_trb)[1], reinterpret_cast<uint8_t*>(&setup_trb)[2], reinterpret_cast<uint8_t*>(&setup_trb)[3],
                reinterpret_cast<uint8_t*>(&setup_trb)[4], reinterpret_cast<uint8_t*>(&setup_trb)[5], reinterpret_cast<uint8_t*>(&setup_trb)[6], reinterpret_cast<uint8_t*>(&setup_trb)[7],
                reinterpret_cast<uint8_t*>(&setup_trb)[8], reinterpret_cast<uint8_t*>(&setup_trb)[9], reinterpret_cast<uint8_t*>(&setup_trb)[10], reinterpret_cast<uint8_t*>(&setup_trb)[11],
                reinterpret_cast<uint8_t*>(&setup_trb)[12], reinterpret_cast<uint8_t*>(&setup_trb)[13], reinterpret_cast<uint8_t*>(&setup_trb)[14], reinterpret_cast<uint8_t*>(&setup_trb)[15]
            );
            Processor::FlushDataCache(&setup_trb, sizeof(TRB));

            uint32_t next_pos = advance_transfer_ring_pointer(setup_pos);
            slot.transfer_ring_enqueue.store(next_pos);
            if (next_pos == 0) {
                slot.transfer_ring_cycle_state = !slot.transfer_ring_cycle_state;
            }

            if (!data.empty()) {
                uint32_t data_pos = slot.transfer_ring_enqueue.load();
                TRB& data_trb = slot.transfer_ring[data_pos];
                data_trb.parameter = get_physical_address(dma_data_ptr);
                data_trb.status = static_cast<uint32_t>(data.size());
                data_trb.control = (TRB_TYPE_DATA << 10) | TRB_CTRL_CHAIN | (slot.transfer_ring_cycle_state ? TRB_CTRL_CYCLE : 0);
                if (data_stage_in) {
                    data_trb.control |= TRB_CTRL_DIR_IN;
                }
                printf("XHCI: Data TRB - 0x%016X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", &data_trb,
                    reinterpret_cast<uint8_t*>(&data_trb)[0], reinterpret_cast<uint8_t*>(&data_trb)[1], reinterpret_cast<uint8_t*>(&data_trb)[2], reinterpret_cast<uint8_t*>(&data_trb)[3],
                    reinterpret_cast<uint8_t*>(&data_trb)[4], reinterpret_cast<uint8_t*>(&data_trb)[5], reinterpret_cast<uint8_t*>(&data_trb)[6], reinterpret_cast<uint8_t*>(&data_trb)[7],
                    reinterpret_cast<uint8_t*>(&data_trb)[8], reinterpret_cast<uint8_t*>(&data_trb)[9], reinterpret_cast<uint8_t*>(&data_trb)[10], reinterpret_cast<uint8_t*>(&data_trb)[11],
                    reinterpret_cast<uint8_t*>(&data_trb)[12], reinterpret_cast<uint8_t*>(&data_trb)[13], reinterpret_cast<uint8_t*>(&data_trb)[14], reinterpret_cast<uint8_t*>(&data_trb)[15]
                );

                Processor::FlushDataCache(&data_trb, sizeof(TRB));

                next_pos = advance_transfer_ring_pointer(data_pos);
                slot.transfer_ring_enqueue.store(next_pos);
                if (next_pos == 0) {
                    slot.transfer_ring_cycle_state = !slot.transfer_ring_cycle_state;
                }
            }

            uint32_t status_pos = slot.transfer_ring_enqueue.load();
            TRB& status_trb = slot.transfer_ring[status_pos];
            status_trb.parameter = 0;
            status_trb.status = 0;
            status_trb.control =
            (TRB_TYPE_STATUS << 10) |
            TRB_CTRL_IOC |
            ((!data_stage_present || !data_stage_in) ? TRB_CTRL_DIR_IN : 0) |
            (slot.transfer_ring_cycle_state ? TRB_CTRL_CYCLE : 0);
            printf("XHCI: Status TRB - 0x%016X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", &status_trb,
                reinterpret_cast<uint8_t*>(&status_trb)[0], reinterpret_cast<uint8_t*>(&status_trb)[1], reinterpret_cast<uint8_t*>(&status_trb)[2], reinterpret_cast<uint8_t*>(&status_trb)[3],
                reinterpret_cast<uint8_t*>(&status_trb)[4], reinterpret_cast<uint8_t*>(&status_trb)[5], reinterpret_cast<uint8_t*>(&status_trb)[6], reinterpret_cast<uint8_t*>(&status_trb)[7],
                reinterpret_cast<uint8_t*>(&status_trb)[8], reinterpret_cast<uint8_t*>(&status_trb)[9], reinterpret_cast<uint8_t*>(&status_trb)[10], reinterpret_cast<uint8_t*>(&status_trb)[11],
                reinterpret_cast<uint8_t*>(&status_trb)[12], reinterpret_cast<uint8_t*>(&status_trb)[13], reinterpret_cast<uint8_t*>(&status_trb)[14], reinterpret_cast<uint8_t*>(&status_trb)[15]
            );
            Processor::FlushDataCache(&status_trb, sizeof(TRB));

            next_pos = advance_transfer_ring_pointer(status_pos);
            slot.transfer_ring_enqueue.store(next_pos);
            if (next_pos == 0) {
                slot.transfer_ring_cycle_state = !slot.transfer_ring_cycle_state;
            }
        }

        // Doorbell 0 is the command ring; transfer rings use the slot's doorbell.
        ring_doorbell(slot_id, 1);

        Status const transfer_status = wait_for_transfer_event(slot_id);
        printf("XHCI: Control transfer completed with status %s\n",
            transfer_status == Status::Success  ? "Success" :
            transfer_status == Status::Timeout  ? "Timeout" :
            transfer_status == Status::Error    ? "Error" :
            /*transfer_status == Status::NotFound ?*/ "NotFound");
        if (/*transfer_status == Status::Success &&*/ data_stage_in && !data.empty()) {
            Processor::InvalidateDataCache(dma_data_ptr, data.size());
            std::memcpy(data.data(), dma_data_ptr, data.size());

            printf("Data:");
            for (auto data_byte : data) {
                printf(" %02X", data_byte);
            }
            printf("\n");
        }
        return transfer_status;
    }
    
    // Public method to manually process pending events (useful for testing)
    void process_pending_events() override {
        process_events();
    }
    
    // Additional utility methods
    uint32_t get_port_status(uint32_t port) {
        if (port == 0 || port > max_ports) {
            return 0;
        }
        
        return operationalRegisters_.PortStatusControl[port - 1];
    }
    
    void reset_port(uint32_t port) {
        if (port == 0 || port > max_ports) {
            return;
        }
        
        printf("XHCI: Resetting port %u\n", port);
        
        auto& portscReg = operationalRegisters_.PortStatusControl[port - 1];

        portscReg |= (1 << 4);

        // Wait for reset to complete and the device to become enabled.
        for (int i = 0; i < 5000; ++i) {
            uint32_t const portsc = portscReg;
            bool const reset_in_progress = (portsc & (1u << 4)) != 0;
            bool const enabled = (portsc & (1u << 1)) != 0;
            if (!reset_in_progress && enabled) {
                return;
            }
            Cpu::DelayInMicroseconds(100);
        }

        uint32_t const final_portsc = portscReg;
        printf("XHCI: Port %u reset timeout, PORTSC=0x%08X\n", port, final_portsc);
    }
    
    static char const* describe_device_class(uint8_t device_class) {
        switch (device_class) {
            case 0x00: return "Interface-specific";
            case 0x01: return "Audio";
            case 0x02: return "Communications";
            case 0x03: return "HID";
            case 0x05: return "Physical";
            case 0x06: return "Image";
            case 0x07: return "Printer";
            case 0x08: return "Mass Storage";
            case 0x09: return "Hub";
            case 0x0A: return "CDC Data";
            case 0x0E: return "Video";
            case 0x0F: return "Personal Healthcare";
            case 0x10: return "Audio/Video";
            case 0x11: return "Billboard";
            case 0x12: return "USB Type-C Bridge";
            default: return "Unknown";
        }
    }

    void print_device_summary(uint32_t port, uint32_t slot_id, std::span<uint8_t const> descriptor, std::span<uint8_t const> config_descriptor) {
        if (descriptor.size() < 18) {
            printf("XHCI: Port %u slot %u: descriptor too small to parse\n", port, slot_id);
            return;
        }

        uint8_t  const bLength            = descriptor[0];
        uint8_t  const bDescriptorType    = descriptor[1];
        uint8_t  const bDeviceClass       = descriptor[4];
        uint8_t  const bDeviceSubClass    = descriptor[5];
        uint8_t  const bDeviceProtocol    = descriptor[6];
        uint8_t  const bMaxPacketSize0    = descriptor[7];
        uint16_t const vendor_id          = static_cast<uint16_t>(descriptor[8] | (descriptor[9] << 8));
        uint16_t const product_id         = static_cast<uint16_t>(descriptor[10] | (descriptor[11] << 8));
        uint8_t  const bNumConfigurations = descriptor[17];

        printf("XHCI: Port %u -> device slot %u identified\n", port, slot_id);
        printf("XHCI:   Function: %s (class=0x%02X, subclass=0x%02X, protocol=0x%02X)\n",
               describe_device_class(bDeviceClass), bDeviceClass, bDeviceSubClass, bDeviceProtocol);
        printf("XHCI:   Vendor/Product: 0x%04X / 0x%04X\n", vendor_id, product_id);
        printf("XHCI:   Capabilities: max-packet-size0=0x%02X, configurations=%u\n",
               bMaxPacketSize0, bNumConfigurations);

        printf("XHCI:   Descriptor length: %u, type: 0x%02X", bLength, bDescriptorType);
        for (size_t i = 0; i < std::min<size_t>(descriptor.size(), 18); ++i) {
            if (i % 16 == 0) {
                printf("\nXHCI:   ");
            }
            printf(" %02X", descriptor[i]);
        }
        printf("\n");

        if (config_descriptor.size() >= 9) {
            uint8_t const config_length  = config_descriptor[0];
            uint8_t const config_type    = config_descriptor[1];
            uint8_t const num_interfaces = config_descriptor[4];
            uint8_t const config_value   = config_descriptor[5];
            uint8_t const max_power      = config_descriptor[8];
            printf("XHCI:   Configuration: value=0x%02X, interfaces=%u, max-power=%umA, attrs=0x%02X\n",
                   config_value, num_interfaces, max_power * 2u, config_descriptor[7]);
            if (config_length < 9) {
                printf("XHCI:   Interface data is incomplete\n");
            }
        }
        else {
            printf("XHCI:   Configuration descriptor unavailable\n");
        }

        if (bLength < 18) {
            printf("XHCI:   Descriptor length is shorter than expected (%u)\n", bLength);
        }
    }

    bool reset_hub_port(uint32_t hub_slot_id, uint32_t port_number) {
        if (controlTransfer(hub_slot_id, 0x23, 0x04, 0x0004, port_number, {}) != Status::Success) {
            printf("XHCI: Failed to reset hub port %u on slot %u\n", port_number, hub_slot_id);
            return false;
        }
        Cpu::DelayInMicroseconds(100'000);
        return true;
    }

    bool enumerate_hub_children(uint32_t hub_slot_id, uint32_t hub_root_port) {
        std::array<uint8_t, 9> hub_descriptor{};
        if (controlTransfer(hub_slot_id, 0xA0, 0x06, 0x2900, 0, hub_descriptor) != Status::Success) {
            printf("XHCI: Failed to read hub descriptor for slot %u\n", hub_slot_id);
            return false;
        }

        if (hub_descriptor.size() < 3) {
            return false;
        }

        uint8_t const port_count = hub_descriptor[2];
        printf("XHCI: Enumerating %u downstream ports for hub slot %u\n", port_count, hub_slot_id);

        for (uint8_t port = 1; port <= port_count; ++port) {
            std::array<uint8_t, 4> port_status_data{};
            if (controlTransfer(hub_slot_id, 0xA3, 0x00, 0x0000, port, port_status_data) != Status::Success) {
                continue;
            }

            uint16_t const port_status = static_cast<uint16_t>(port_status_data[0] | (port_status_data[1] << 8));
            bool const connected = (port_status & 0x0001u) != 0;
            if (!connected) {
                continue;
            }

            printf("XHCI: Hub slot %u port %u has a child device attached\n", hub_slot_id, port);
            (void)reset_hub_port(hub_slot_id, port);
            enumerate_device_at_port(hub_root_port, port, true);
        }

        return true;
    }

    bool enumerate_device_at_port(uint32_t address_port, uint32_t logical_port, bool is_hub_child) {
        if (!is_hub_child) {
            uint32_t portsc = get_port_status(address_port);
            bool     const connected    = (portsc & 1u) != 0;
            bool     const enabled      = (portsc & (1u << 1)) != 0;
            bool     const over_current = (portsc & (1u << 3)) != 0;
            uint32_t const speed        = (portsc >> 10) & 0x0F;

            printf("XHCI: Port %u status: 0x%08X [connected=%u enabled=%u speed=%u over-current=%u]\n",
                   address_port, portsc, connected ? 1u : 0u, enabled ? 1u : 0u, speed, over_current ? 1u : 0u);

            if (!connected) {
                printf("XHCI: Port %u has no device attached\n", address_port);
                return false;
            }

            reset_port(address_port);
        }
        else {
            printf("XHCI: Enumerating child device behind hub port %u using root port %u\n", logical_port, address_port);
        }

        uint32_t slot_id = allocate_device_slot();
        if (slot_id == 0) {
            printf("XHCI: No free device slots available for port %u\n", logical_port);
            return false;
        }

        printf("XHCI: Enable slot completion returned slot ID %u\n", slot_id);

        uint32_t const post_reset_portsc = get_port_status(address_port);
        uint32_t const post_reset_speed = (post_reset_portsc >> 10) & 0x0F;
        if (!address_device(slot_id, address_port, post_reset_speed, 0, 0, 0)) {
            printf("XHCI: Failed to address device on root port %u (slot %u)\n", address_port, slot_id);
            free_device_slot(slot_id);
            return false;
        }

        default_control_slot_id.store(slot_id);

//        remember_discovered_device(slot_id, logical_port, address_port, post_reset_speed, {}, {});
        //print_device_summary(logical_port, slot_id, {}, {});

        std::array<uint8_t, 64> device_descriptor{};
        if (controlTransfer(slot_id, 0x80, 0x06, 0x0100, 0, device_descriptor) != Status::Success)
        {
            printf("XHCI: Port %u could not retrieve a device descriptor\n", logical_port);
            return false;
        }
        std::array<uint8_t, 64> config_descriptor{};
        if (controlTransfer(slot_id, 0x80, 0x06, 0x0200, 0, config_descriptor) == Status::Success) {
            remember_discovered_device(slot_id, logical_port, address_port, post_reset_speed, device_descriptor, config_descriptor);
            print_device_summary(logical_port, slot_id, device_descriptor, config_descriptor);

            controlTransfer(slot_id, 0x00, 0x09, config_descriptor[5], 0, {});
        }
        else {
            remember_discovered_device(slot_id, logical_port, address_port, post_reset_speed, device_descriptor, {});
            print_device_summary(logical_port, slot_id, device_descriptor, {});

            controlTransfer(slot_id, 0x00, 0x09, 1, 0, {});
        }

        // Do this later using common code. We only do the roots here.
        //if (device_descriptor.size() >= 4 && device_descriptor[4] == 0x09) {
        //    enumerate_hub_children(slot_id, address_port);
        //}

        return true;
    }

    void enumerate_devices() {
        printf("XHCI: Enumerating devices...\n");

        for (uint32_t port = 1; port <= max_ports; ++port) {
            enumerate_device_at_port(port, port, false);
        }
    }

    std::span<DeviceInfo const> discovered_devices() const override {
        return discovered_devices_;
    }

    bool run_hello_world_test() override
    {
        
        // 1. Setup Test Parameters
        // We will request the Device Descriptor (Request 0x06)
        uint8_t request_type = 0x80; // Device-to-Host, Standard Request
        uint8_t request_code = 0x06; // GET_DESCRIPTOR
        uint16_t value = 0x0100; // Descriptor Type (Device) and Index (0)
        uint16_t index = 0x0000;
        
        // The maximum expected size for the device descriptor is typically 256 bytes.
        // We allocate a buffer large enough for the test.
        std::array<uint8_t, 256> data_buffer{};

        // 2. Execute the Control Transfer
        // This calls the driver's controlTransfer method, which should internally:
        // a) Allocate necessary transfer rings/contexts.
        // b) Construct Setup, Data, and Status TRBs.
        // c) Ring the appropriate doorbell.
        // d) Wait for the completion event/interrupt.
        // e) Process the event and populate the data_buffer.
        
        Status result = controlTransfer(
            0,
            request_type,
            request_code,
            value,
            index,
            std::span<uint8_t>(data_buffer.data(), data_buffer.size())
        );

        // 3. Verification Phase
        
        // Check 3a: Driver execution status
        if (result != Status::Success) {
            // Test failed: The driver reported an internal error during the transfer.
            printf("Test Failed: Control transfer returned status %d.\n", result);
            return false;
        }

        // Check 3b: Data integrity (The most critical check)
        // The driver should have populated the buffer with the device descriptor data.
        // We check for a known signature or a specific field (e.g., bLength).
        
        // Check if the buffer is non-empty and contains a valid descriptor length (e.g., > 18 bytes)
        if (data_buffer[0] < 18) {
            printf("Test Failed: Received descriptor length is too short or zero.\n");
            return false;
        }
        
        // Check a specific field, e.g., bDescriptorType (should be 1 for Device Descriptor)
        if (data_buffer[1] != 1) {
            printf("Test Failed: Descriptor type mismatch. Expected 1, got %d.\n", data_buffer[1]);
            return false;
        }
        
        // 4. Event/Interrupt Verification (Optional but recommended)
        // If the driver relies on asynchronous event processing, we must ensure the event was processed.
        // This might involve calling a driver-specific function to drain the event queue.
        process_pending_events(); 
        
        // If the test reaches this point, the command was sent, the hardware responded, 
        // the driver processed the event, and the data was correctly received.
        printf("Test Succeeded: Minimal control transfer completed successfully.\n");
        return true;
    }
};

namespace {

class XhciUsbDriver : public UsbDriver
{
public:
    explicit XhciUsbDriver(std::unique_ptr<Usb::Controller> controller)
        : controller_(std::move(controller))
    {
    }

    IoHandle GetIoHandle(uint8_t deviceAddress) override
    {
        if (deviceAddress == 0 || deviceAddress > deviceTable_.size())
        {
            return {};
        }
        auto& devicePtr = deviceTable_[deviceAddress - 1];
        if (!devicePtr)
        {
            return {};
        }
        return IoHandle(reinterpret_cast<void*>(deviceAddress), IoHandleDeleter{ nullptr });
    }

    void DeleteIoHandle(void* ptr) override {}

    void RemoveHubPayload(UsbDevice& device) {
        if (device.PayLoadId == PayLoadType::Hub && device.HubPayload) {
            for (auto* pChild : device.HubPayload->Children) {
                if (pChild) {
                    UsbDeallocateDevice(pChild);
                }
            }

            device.HubPayload = nullptr;
            device.PayLoadId = PayLoadType::None;
        }
    }

    std::shared_ptr<UsbDriver> DriverRef()
    {
        return std::shared_ptr<UsbDriver>(this, [](UsbDriver*) {});
    }

    std::expected<std::shared_ptr<UsbDevice>, RESULT> UsbAllocateDevice(uint32_t address, UsbDevice* parentHubDevice, uint8_t parentHubPort)
    {
        std::shared_ptr<UsbDevice> device = std::make_shared<UsbDevice>(address, DriverRef());
        if (!device)
        {
            return std::unexpected(RESULT::ErrorMemory);
        }

        device->Config.Status = USB_STATUS_ATTACHED;
        device->ParentHub.PortNumber = parentHubPort;
        device->ParentHub.Device = parentHubDevice ? deviceTable_[parentHubDevice->GetAddress() - 1] : nullptr;
        device->PayLoadId = PayLoadType::None;

        if (deviceTable_.size() < address)
        {
            deviceTable_.resize(address);
        }

        deviceTable_[address - 1] = device;
        return std::move(device);
    }

    std::expected<std::shared_ptr<UsbDevice>, RESULT> UsbAllocateDevice(UsbDevice* parentHubDevice, uint8_t parentHubPort) override
    {
        auto const address = controller_->allocate_device_slot();
        if (address == 0)
        {
            return std::unexpected(RESULT::ErrorMemory);
        }

        return UsbAllocateDevice(address, parentHubDevice, parentHubPort);
    }

    void UsbDeallocateDevice (struct UsbDevice *device) override
    {
        if (device == nullptr) {
            return;
        }

        if (device->IsHub()) {								// If this device is a hub we will need to deal with the children
            for (auto* child : device->HubPayload->Children) {
                if (child != nullptr)
                    UsbDeallocateDevice(child);
            }
            RemoveHubPayload(*device);
        }

        if (auto& parent = device->ParentHub.Device; parent && parent->PayLoadId == PayLoadType::Hub && parent->HubPayload)
        {
            auto const port = static_cast<size_t>(device->ParentHub.PortNumber);
            if (port < parent->HubPayload->Children.size() && parent->HubPayload->Children[port] == device)
            {
                parent->HubPayload->Children[port] = nullptr;
            }
        }

        auto const deviceAddress = static_cast<size_t>(device->GetAddress());
        if (deviceAddress > 0 && deviceAddress <= deviceTable_.size())
        {
            deviceTable_[deviceAddress - 1].reset();
        }
    }
    // Sets the address of the device with control endpoint given by the pipe. Zero
    // is a restricted address for the rootHub and will return if attempted.
    Async::task<RESULT> HCDSetAddress (UsbDevice* device, IoHandle const& ioHandle, uint8_t address) override
    {
        if (device->RootHubPort == address)
        {
            // Already done during initialization.
            co_return RESULT::Ok;
        }
        auto slot = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(ioHandle.get()));
        if (slot == 0 || slot != LookupSlot(device)) {
            co_return RESULT::ErrorDevice;
        }
        uint32_t route = device->ParentHub.PortNumber;
        uint32_t parentHubSlotId = 0;
        uint32_t parentHubPort = device->ParentHub.PortNumber;

        if (device->ParentHub.Device)
        {
            route = device->ParentHub.PortNumber;
            auto parentHub = device->ParentHub.Device.get();
            parentHubSlotId = parentHub->GetAddress();
            for (uint32_t i = 1; i <= 5 && parentHub->ParentHub.Device; ++i)
            {
                route = (route << 5) | (parentHub->ParentHub.PortNumber & 0x1F);
                parentHub = parentHub->ParentHub.Device.get();
            }
        }

        uint32_t const address_port = device->RootHubPort;
        uint32_t const post_reset_portsc = static_cast<Controller*>(controller_.get())->get_port_status(address_port);
        uint32_t const post_reset_speed = (post_reset_portsc >> 10) & 0x0F;
        if (!static_cast<Controller*>(controller_.get())->address_device(slot, address_port, post_reset_speed, route, 0, 0 /*parentHubSlotId, parentHubPort*/)) {
            printf("XHCI: Failed to address device on root port %u (slot %u) route %05X parentHubSlotId %u parentHubPort %u\n", address_port, slot, route, parentHubSlotId, parentHubPort);
            co_return RESULT::ErrorDevice;
        }
        co_return RESULT::Ok;
    }

    Async::task<> Initialize()
    {
        if (!controller_) {
            error_ = RESULT::ErrorGeneral;
            co_return;
        }

        if (controller_->initialize() != Usb::Status::Success) {
            error_ = RESULT::ErrorHardware;
            co_return;
        }

        printf("discovered devices: %zu\n", controller_->discovered_devices().size());

        for (auto const& info : controller_->discovered_devices())
        {
            auto deviceEx = UsbAllocateDevice(info.SlotId, nullptr, 0);
            if (!deviceEx) {
                error_ = deviceEx.error();
                co_return;
            }
            auto& device = deviceEx.value();
            device->RootHubPort = info.RootHubPort;
            device->Descriptor = info.Descriptor;
            device->Interfaces = info.Interfaces;
            device->Endpoints = info.Endpoints;
            device->Config.Status = info.HasConfiguration ? USB_STATUS_CONFIGURED : USB_STATUS_DEFAULT;
            if (info.Descriptor.bDeviceClass == DeviceClassHub) {
                device->PayLoadId = PayLoadType::Hub;
            }
            else if (info.Descriptor.bDeviceClass == DeviceClassInInterface || info.Descriptor.bDeviceClass == 0x03) {
                device->PayLoadId = PayLoadType::Hid;
                device->HidPayload = AllocateHidPayload();
            }
            else {
                device->PayLoadId = PayLoadType::None;
            }
        }

        if (deviceTable_.empty())
        {
            printf("Adding empty device\n");
            auto deviceEx = UsbAllocateDevice(0, nullptr, 0);
            if (!deviceEx) {
                error_ = deviceEx.error();
                co_return;
            }
        }

        for (auto&& rootDevice : deviceTable_)
        {
            auto const result = co_await EnumerateDevice(rootDevice.get(), nullptr, 0);
            if (result != RESULT::Ok)
            {
                LOG("FATAL ERROR: Could not enumerate root HUB\n");
                error_ = result;
                co_return;
            }
        }

        error_ = RESULT::Ok;
        co_return;
    }

    RESULT GetError() override { return error_; }

    std::generator<UsbDevice&> EnumerateDevices() override
    {
        for (auto const& device : deviceTable_) {
            if (device) {
                co_yield *device;
            }
        }
    }

    UsbDevice* UsbGetRootHub() override { return deviceTable_.empty() ? nullptr : deviceTable_.front().get(); }

    UsbDevice* UsbDeviceAtAddress(uint8_t devNumber) override
    {
        if (devNumber == 0 || devNumber > deviceTable_.size()) {
            return nullptr;
        }
        auto const& device = deviceTable_[devNumber - 1];
        return device ? device.get() : nullptr;
    }

    const char* UsbGetDescription(UsbDevice* device) override
    {
        if (!device) {
            return "USB Device";
        }
        if (device->Descriptor.bDeviceClass == DeviceClassHub) {
            return "USB Hub";
        }
        if (device->PayLoadId == PayLoadType::Hid) {
            return "USB HID";
        }
        return "USB Device";
    }

    Async::task<RESULT> HCDSubmitControlMessageOUT(
        UsbDevice* device,
        IoHandle const& ioHandle,
        std::span<std::byte const> buffer,
        UsbDeviceRequest request,
        uint32_t timeout,
        uint32_t* bytesTransferred) override
    {
        if (request.Type & 0x80) {
            LOG("HCDSubmitControlMessageOUT called with IN request type: %#x\n", request.Type);
            co_return RESULT::ErrorArgument;
        }

        auto slot = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(ioHandle.get()));
        if (slot == 0 || slot != LookupSlot(device)) {
            co_return RESULT::ErrorDevice;
        }
        auto const status = co_await SubmitControlTransfer(slot, request.Type, request.Request, request.Value, request.Index, const_cast<std::byte*>(buffer.data()), buffer.size(), timeout, bytesTransferred);
        co_return status;
    }

    Async::task<RESULT> HCDSubmitControlMessageIN(UsbDevice* device,
        IoHandle const& ioHandle,
        std::span<std::byte> buffer,
        UsbDeviceRequest request,
        uint32_t timeout,
        uint32_t* bytesTransferred) override
    {
        if (!(request.Type & 0x80)) {
            LOG("HCDSubmitControlMessageIN called with OUT request type: %#x\n", request.Type);
            co_return RESULT::ErrorArgument;
        }

        auto slot = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(ioHandle.get()));
        printf("ioHandle slot = %u, device slot = %u\n", slot, LookupSlot(device));
        if (slot == 0 || slot != LookupSlot(device)) {
            co_return RESULT::ErrorDevice;
        }
        auto const status = co_await SubmitControlTransfer(slot, request.Type, request.Request, request.Value, request.Index, buffer.data(), buffer.size(), timeout, bytesTransferred);
        co_return status;
    }

    Async::task<RESULT> HCDEndpointTransfer(UsbDevice* device, UsbEndpointDescriptor endpoint, std::byte* buffer, uint32_t& bufferLength) override
    {
        if (!device || !buffer || bufferLength == 0) {
            co_return RESULT::ErrorArgument;
        }

        auto const slot = LookupSlot(device);
        if (slot == 0) {
            co_return RESULT::ErrorDevice;
        }

        auto const direction = endpoint.EndpointAddress.Direction;
        if (direction == USB_DIRECTION_IN) {
            auto const status = controller_->read(endpoint.EndpointAddress.Number, std::span<uint8_t>(reinterpret_cast<uint8_t*>(buffer), bufferLength));
            co_return status == Usb::Status::Success ? RESULT::Ok : RESULT::ErrorTransmission;
        }

        auto const status = controller_->write(endpoint.EndpointAddress.Number, std::span<uint8_t const>(reinterpret_cast<uint8_t const*>(buffer), bufferLength));
        co_return status == Usb::Status::Success ? RESULT::Ok : RESULT::ErrorTransmission;
    }

private:
    uint8_t LookupSlot(UsbDevice* device) const
    {
        if (!device) {
            return 0;
        }
        for (size_t i = 0; i < deviceTable_.size(); ++i) {
            if (deviceTable_[i].get() == device) {
                return static_cast<uint8_t>(i + 1);
            }
        }
        return 0;
    }

    Async::task<RESULT> SubmitControlTransfer(uint8_t slot,
                                              uint8_t requestType,
                                              UsbDeviceRequestRequest requestCode,
                                              uint16_t value,
                                              uint16_t index,
                                              std::byte* buffer,
                                              uint32_t bufferLength,
                                              uint32_t timeout = ControlMessageTimeout,
                                              uint32_t* bytesTransferred = nullptr)
    {
        std::span<uint8_t> payload;
        if (buffer != nullptr && bufferLength > 0) {
            payload = std::span<uint8_t>(reinterpret_cast<uint8_t*>(buffer), bufferLength);
        }

        auto const status = controller_->controlTransfer(slot, requestType, static_cast<uint8_t>(requestCode), value, index, payload);
        if (bytesTransferred) {
            *bytesTransferred = bufferLength;
        }
        co_return status == Usb::Status::Success ? RESULT::Ok : RESULT::ErrorTransmission;
    }

    std::unique_ptr<Usb::Controller> controller_;
    std::vector<std::shared_ptr<UsbDevice>> deviceTable_;
    RESULT error_ = RESULT::ErrorGeneral;
};

} // namespace

Async::task<std::shared_ptr<UsbDriver>> UsbInitializeXhci(PCIe::Bcm2711Driver& pcie, PCIe::DeviceAddress const& deviceAddress)
{
    printf("Initializing xHCI USB Driver\n");
    auto controller = CreateController(pcie, deviceAddress);
    auto driver = std::make_shared<XhciUsbDriver>(std::move(controller));
    co_await driver->Initialize();
    co_return driver;
}


// Factory function to create XHCI controller
std::unique_ptr<Usb::Controller> CreateController(PCIe::Bcm2711Driver& pcie, PCIe::DeviceAddress const& devAddress)
{
    PCIe::Configuration config{ pcie, devAddress };

    printf("XHCI    Loading USB firmware...\n");
    Mailbox::TagMessage<Mailbox::Tag::RPI4_PCIE_XHCI_USB_RESET, 1> resetTag{{ 1u << 20 }};
    if (!Mailbox::SendTags(resetTag)) {
        printf("XHCI    ✗ Failed to load USB firmware\n");
    }
    else
    {
        printf("XHCI    New state: %u\n", resetTag.args[0]);
    }

    auto& common = config.Common();

    common.CacheLineSize = 64 / 4; // ??

    // Display BARs if any and find Bar0
    PCIe::BarInfo bar0{};
    for (auto&& bar : config.enumerate_bars())
    {
        if (bar0.size == 0)
        {
            //printf("  BARs:\n");
            bar0 = bar;
        }
        //PrintBar(bar);
    }

    // Display capabilities if any
    //printf("  Capabilities:\n");
    for (auto const& cap : config.enumerate_capabilities())
    {
        //PrintCapability(cap, config.Common());
    }
    
    // Enable BAR 0 at the beginning of PCIe aperture
    printf("XHCI    Configuring BAR 0: CPU=0x%016llx Size = 0x%zX\n", bar0.physical_address, bar0.size);
    if (bar0.size == 0)
    {
        printf("    BAR 0 size is zero. Halting...\n");
        Cpu::Halt();
    }
    
    auto bar0Memory = config.map_bar(bar0);

    printf("    Configured BAR 0: CPU=0x%016llx Size = 0x%zX\n", bar0.physical_address, bar0.size);
    
    // Verify the BAR was written correctly
    //auto const rebar0 = config.get_bar(0);

    //printf("    BAR 0 readback: CPU=0x%016llx\n", rebar0.physical_address);

    common.InterruptPin = 1; // INTA
    common.Command = 0x146; // !IO, Memory, Master, SERR, PARITY

    // Add a delay to ensure the configuration takes effect
    Cpu::DelayInMicroseconds(100'000);

    asm volatile("dsb sy" : : : "memory");  // ARM64

    return std::make_unique<Controller>(*reinterpret_cast<CapabilityRegisters*>(bar0Memory.data()));
}

}
// namespace Usb::Xhci
