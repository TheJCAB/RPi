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
#include <expected>
#include <fmt/format.h>
#include <vector>

extern uintptr_t GpuMemBase;

HidDevice* AllocateHidPayload();

#define LOG(...) fmt::print(__VA_ARGS__)
#define LOGln(...) fmt::println(__VA_ARGS__)
/*
#define LOG_DEBUG(...) fmt::print(__VA_ARGS__)
#define LOG_DEBUGln(...) fmt::println(__VA_ARGS__)
/*/
#define LOG_DEBUG(...) ((void)0)
#define LOG_DEBUGln(...) ((void)0)
//*/

namespace Usb::Xhci
{

// These are RPi4 specifics.
// TODO: Abstract them out to allow this driver to work more generically, for other platforms.

// Get physical address for DMA (assuming identity mapping for now)
uint64_t get_physical_address(void* virtual_addr)
{
    return reinterpret_cast<uintptr_t>(virtual_addr) - GpuMemBase + (BootLib::Cpu::IsRpi4() ? 0x4'0000'0000ull : 0);
}    

uint32_t endpoint0_max_packet_size(uint32_t speed) {
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

// XHCI register definitions.

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

union HcsParams3
{
    struct
    {
        uint8_t U1DeviceExitlatency;
        uint8_t U2DeviceExitlatency;
        uint8_t Reserved[2];
    };
    
    uint32_t Raw32;
};

union HccParams1
{
    struct
    {
        uint32_t AddressingCapability64              :  1;
        uint32_t BandwidthNegotiationCapability      :  1;
        uint32_t ContextSize                         :  1;
        uint32_t PortPowerControl                    :  1;
        uint32_t PortIndicators                      :  1;
        uint32_t LightHostControllerResetCapability  :  1;
        uint32_t LatencyToleranceMessagingCapability :  1;
        uint32_t NoSecondaryStreamIdSupport          :  1; 
        uint32_t ParseAllEventData                   :  1; 
        uint32_t StoppedShortPacketCapable           :  1;
        uint32_t StoppedEDTLACapable                 :  1;
        uint32_t ContiguousFrameIdCapable            :  1;
        uint32_t MaxPrimaryStreamArraySize           :  4;
        uint32_t ExtendedCapabilitiesPointer         :  8;
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
    Mmio::Register<HcsParams3 const, 0x0C> StructuralParams3;
    Mmio::Register<HccParams1 const, 0x10> CapabilityParams1;
    Mmio::Register<uint32_t   const, 0x14> DoorbellOffset;
    Mmio::Register<uint32_t   const, 0x18> RuntimeOffset;
    Mmio::Register<uint32_t   const, 0x1C> CapabilityParams2;
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
enum class TrbType : uint8_t
{
    Invalid            =  0,
    // Transfers
    Normal             =  1,
    Setup              =  2,
    Data               =  3,
    Status             =  4,
    Isoch              =  5,
    Link               =  6, // Note: also command
    EventData          =  7,
    NoOp               =  8,
    // Commands
    EnableSlot         =  9,
    DisableSlot        = 10,
    AddressDev         = 11,
    NoopCmd            = 23,
    // Events
    TransferEvent      = 32,
    CmdCompletionEvent = 33,
    PortStatusChange   = 34,
};

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
enum class CompletionCode : uint8_t
{
    Invalid            =  0,
    Success            =  1,
    TrbError           =  5,
    BandwidthError     =  8,
    ShortPacket        = 13,
    EndpointNotEnabled = 12,
};

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

// Driver structures.

struct DeviceSlot
{
    uint8_t                 slotId                   = 0;
    std::atomic<bool>       in_use                    = false;
    bool                    transfer_ring_cycle_state = true;
    TRB*                    transfer_ring             = nullptr;
    std::atomic<uint32_t>   transfer_ring_enqueue     = 0;
    std::atomic<uint32_t>   transfer_ring_dequeue     = 0;
    void*                   input_context             = nullptr;
    void*                   device_context            = nullptr;
};

struct SlotArray
{
    std::unique_ptr<DeviceSlot[]> deviceSlotStorage{};
    std::span      <DeviceSlot>   deviceSlots{};

    SlotArray() = default;
    SlotArray(SlotArray&&) = default;
    SlotArray& operator=(SlotArray&& other)
    {
        // Explicit implementation so we can neutralize `other.deviceSlots`, to avoid "accidents".
        deviceSlotStorage = std::exchange(other.deviceSlotStorage, {});
        deviceSlots       = std::exchange(other.deviceSlots      , {});
        return *this;
    }

    void Init(size_t maxSlots)
    {
        deviceSlotStorage = std::make_unique<DeviceSlot[]>(maxSlots);
        deviceSlots       = std::span<DeviceSlot>(deviceSlotStorage.get(), maxSlots);
    }

    size_t size() const { return deviceSlots.size(); }

    DeviceSlot* operator[](size_t index)
    {
        if (index == 0 || index >= deviceSlots.size()) {
            return nullptr;
        }
        return &deviceSlots[index - 1];
    }
};

class XhciUsbDriver : public UsbDriver
{
public:
    explicit XhciUsbDriver(CapabilityRegisters& capabilityRegisters)
        : capabilityRegisters_ { capabilityRegisters }
        , operationalRegisters_{ *reinterpret_cast<OperationalRegisters*>(reinterpret_cast<uintptr_t>(&capabilityRegisters) + capabilityRegisters.CapLength) }
        , runtimeRegisters_    { *reinterpret_cast<RuntimeRegisters*    >(reinterpret_cast<uintptr_t>(&capabilityRegisters) + (capabilityRegisters.RuntimeOffset.get() & ~0x1Fu)) }
        , doorbellRegisters_   { *reinterpret_cast<DoorbellRegisters*   >(reinterpret_cast<uintptr_t>(&capabilityRegisters) + (capabilityRegisters.DoorbellOffset.get() & ~0x3u)) }
    {
    }

    ~XhciUsbDriver() override = default;

    struct alignas(32) InputControlContext {
        uint32_t drop_context_flags;
        uint32_t add_context_flags;
        uint32_t reserved[5];
        uint32_t configuration_value;
    };

    // Register access pointers
    CapabilityRegisters&  capabilityRegisters_;
    OperationalRegisters& operationalRegisters_;
    RuntimeRegisters&     runtimeRegisters_;
    DoorbellRegisters&    doorbellRegisters_;

    // Device capabilities
    uint32_t maxRootPorts = 0;
    uint32_t maxScratchpadBuffers = 0;
    
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

    HccParams1 controller_hccparams1{};

    // Device slots
    SlotArray deviceSlots_;

    std::vector<DeviceInfo> discovered_devices_;


    void SetCommand_ring_control(uint64_t value) {
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
        return controller_hccparams1.ContextSize ? 64u : 32u;
    }

    void dump_input_context(void const* input_context, uint32_t context_words) {
        auto const* const dwords = static_cast<uint32_t const*>(input_context);
        bool printed = false;

        for (uint32_t i = 0; i < context_words && i < (context_size_bytes() / 4); ++i) {
            if (dwords[i] == 0u) {
                continue;
            }

            if (!printed) {
                LOG_DEBUGln("XHCI: input context nonzero DWORDs:");
                printed = true;
            }

            LOG_DEBUGln("XHCI:   [{}] = 0x{:08X}", i, dwords[i]);
        }

        if (!printed) {
            LOG_DEBUGln("XHCI: input context is all-zero");
        }

        if (context_words > context_size_bytes() / 4)
        {
            dump_device_context(dwords + context_size_bytes() / 4, context_words - context_size_bytes() / 4);
        }
    }

    void dump_device_context(void const* input_context, uint32_t context_words) {
        auto const* const dwords = static_cast<uint32_t const*>(input_context);
        bool printed = false;

        for (uint32_t i = 0; i < context_words; ++i) {
            if (dwords[i] == 0u) {
                continue;
            }

            if (!printed) {
                LOG_DEBUGln("XHCI: device context nonzero DWORDs:");
                printed = true;
            }

            LOG_DEBUGln("XHCI:   [{}] = 0x{:08X}", i, dwords[i]);
        }

        if (!printed) {
            LOG_DEBUGln("XHCI: device context is all-zero");
        }
    }

    bool address_device(uint32_t const slotId, uint32_t port, uint32_t port_speed, uint32_t route, uint32_t parentHubSlotId, uint32_t parentHubPort, bool bsr)
    {
        LOG_DEBUGln("XHCI: Addressing device on slot {}, port {}, speed {}", slotId, port, port_speed);

        auto* slot = deviceSlots_[slotId];
        if (slot == nullptr)
        {
            LOG_DEBUGln("XHCI: Invalid slot {} for Address Device", slotId);
            return false;
        }

        if (!ensure_transfer_ring(slotId)) {
            LOG_DEBUGln("XHCI: Transfer ring unavailable for Address Device (slot {})", slotId);
            return false;
        }

        uint32_t const ctx_size = context_size_bytes();
        uint32_t const page_count_for_two_context_pages = 2;

        void* device_context = slot->device_context;
        if (!device_context) {
            device_context = Mmu::AllocateGpuMemory(page_count_for_two_context_pages);
            if (!device_context) {
                LOG_DEBUGln("XHCI: Failed to allocate device context for slot {}", slotId);
                return false;
            }
            slot->device_context = device_context;
        }
        if (bsr) // !
        {
            std::memset(device_context, 0, page_count_for_two_context_pages * Mmu::PageSize);
        }

        void* input_context = slot->input_context;
        if (!input_context) {
            input_context = Mmu::AllocateGpuMemory(page_count_for_two_context_pages);
            if (!input_context) {
                LOG_DEBUGln("XHCI: Failed to allocate input context for slot {}", slotId);
                return false;
            }
            slot->input_context = input_context;
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
        uint32_t ep0_ring_index = slot->transfer_ring_enqueue.load();
        if (ep0_ring_index >= (TRANSFER_RING_SIZE - 1)) {
            ep0_ring_index = 0;
        }
        uint64_t const tr_dequeue = get_physical_address(&slot->transfer_ring[ep0_ring_index]) |
                                    (slot->transfer_ring_cycle_state ? 1u : 0u);
        ep0_ctx[2] = static_cast<uint32_t>(tr_dequeue & 0xFFFF'FFFFu);
        ep0_ctx[3] = static_cast<uint32_t>(tr_dequeue >> 32);
        // Average TRB Length in DW4 lower 16 bits.
        ep0_ctx[4] = 8;

        dump_input_context(input_context, 3u * (ctx_size / sizeof(uint32_t)));

        uint64_t const device_context_phys = get_physical_address(device_context);
        device_context_base_array[slotId] = device_context_phys;

        Processor::FlushDataCache(device_context, page_count_for_two_context_pages * Mmu::PageSize);
        Processor::FlushDataCache(input_context, page_count_for_two_context_pages * Mmu::PageSize);
        Processor::FlushDataCache(&device_context_base_array[slotId], sizeof(device_context_base_array[slotId]));

        TRB address_device_cmd{};
        address_device_cmd.parameter = get_physical_address(input_context);
        address_device_cmd.status = 0;
        address_device_cmd.control = (TRB_TYPE_ADDRESS_DEV << 10) | (slotId << 24) | (bsr ? 0x200 : 0);

        if (!send_command(address_device_cmd)) {
            LOG_DEBUGln("XHCI: Failed to submit Address Device for slot {}", slotId);
            return false;
        }

        uint32_t completed_slot = 0;
        if (!wait_for_command_completion(completed_slot) || completed_slot != slotId) {
            LOG_DEBUGln("XHCI: Address Device did not complete for slot {} (completed slot {})", slotId, completed_slot);
            return false;
        }

        LOG_DEBUGln("XHCI: Address Device completed for slot {} (port {} speed {} MPS {})", slotId, port, port_speed, mps);

        dump_device_context(device_context, 2u * (ctx_size / sizeof(uint32_t)));

        return true;
    }

    bool setup_rings()
    {
        LOG_DEBUGln("XHCI: Setting up rings...");
        
        // Allocate command ring (64-byte aligned)
        command_ring = { Mmu::AllocateGpuMemory<TRB>((COMMAND_RING_SIZE * sizeof(TRB) + Mmu::PageSize - 1) / Mmu::PageSize), COMMAND_RING_SIZE };
        if (!command_ring.data()) {
            command_ring = {};
            LOG_DEBUGln("XHCI: Failed to allocate command ring");
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
            LOG_DEBUGln("XHCI: Failed to allocate event ring");
            return false;
        }
        std::fill(event_ring.begin(), event_ring.end(), TRB{});
        Processor::FlushDataCache(event_ring.data(), event_ring.size_bytes());
        
        // Allocate event ring segment table
        event_ring_segment_table = { Mmu::AllocateGpuMemory<EventRingSegment>(1), 1 };
        if (!event_ring_segment_table.data()) {
            event_ring_segment_table = {};
            LOG_DEBUGln("XHCI: Failed to allocate event ring segment table");
            return false;
        }
        
        // Setup event ring segment table
        event_ring_segment_table[0].base_address = get_physical_address(event_ring.data());
        event_ring_segment_table[0].size         = static_cast<uint32_t>(event_ring.size());
        event_ring_segment_table[0].reserved     = 0;
        Processor::FlushDataCache(event_ring_segment_table.data(), event_ring_segment_table.size_bytes());
        
        // Allocate device context base array
        uint32_t dcbaa_size = (deviceSlots_.size() + 1) * sizeof(uint64_t);
        device_context_base_array = { Mmu::AllocateGpuMemory<uint64_t>((dcbaa_size + Mmu::PageSize - 1) / Mmu::PageSize), deviceSlots_.size() + 1 };
        if (!device_context_base_array.data()) {
            device_context_base_array = {};
            LOG_DEBUGln("XHCI: Failed to allocate device context base array");
            return false;
        }
        std::fill(device_context_base_array.begin(), device_context_base_array.end(), 0);
        Processor::FlushDataCache(device_context_base_array.data(), device_context_base_array.size_bytes());

        if (maxScratchpadBuffers > 0)
        {
            if (maxScratchpadBuffers > scratchpad_buffers.size()) {
                LOG_DEBUGln("XHCI: Scratchpad count {} exceeds supported max {}",
                       maxScratchpadBuffers,
                       scratchpad_buffers.size());
                return false;
            }

            scratchpad_buffer_array = {
                Mmu::AllocateGpuMemory<uint64_t>((maxScratchpadBuffers * sizeof(uint64_t) + Mmu::PageSize - 1) / Mmu::PageSize),
                maxScratchpadBuffers
            };
            if (!scratchpad_buffer_array.data()) {
                scratchpad_buffer_array = {};
                LOG_DEBUGln("XHCI: Failed to allocate scratchpad buffer pointer array");
                return false;
            }

            std::fill(scratchpad_buffer_array.begin(), scratchpad_buffer_array.end(), 0);
            scratchpad_buffers.fill(nullptr);

            for (uint32_t i = 0; i < maxScratchpadBuffers; ++i) {
                void* const scratch = Mmu::AllocateGpuMemory(1);
                if (!scratch) {
                    LOG_DEBUGln("XHCI: Failed to allocate scratchpad buffer {}", i);
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
            LOG_DEBUGln("XHCI: Programmed {} scratchpad buffers", maxScratchpadBuffers);
        }
        
        // Setup command ring control register
        SetCommand_ring_control(get_physical_address(command_ring.data()) | 1); // Set ring cycle state
        
        // Setup device context base address array pointer
        set_dcbaap(get_physical_address(device_context_base_array.data()));
        
        // Setup event ring
        runtimeRegisters_.EventRingSegmentTableSize = 1; // One segment
        set_erstba(get_physical_address(event_ring_segment_table.data()));
        set_erdp  (get_physical_address(event_ring.data()));

        // Enable interrupter
        runtimeRegisters_.InterrupterManagement = 2; // Interrupt Enable (IE)
        runtimeRegisters_.InterrupterModeration = 0x00004000; // 1ms moderation

        LOG_DEBUGln("XHCI: Rings setup complete");
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
            
            LOG_DEBUGln("XHCI: Event TRB type={} completion={}", trb_type, completion_code);
            
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
        LOG_DEBUGln("XHCI: Interrupt received");

        return {};
    }

    bool ensure_transfer_ring(uint32_t slotId)
    {
        auto* slot = deviceSlots_[slotId];
        if (slot == nullptr)
        {
            return false;
        }

        if (slot->transfer_ring != nullptr) {
            return true;
        }

        slot->transfer_ring = Mmu::AllocateGpuMemory<TRB>((TRANSFER_RING_SIZE * sizeof(TRB) + Mmu::PageSize - 1) / Mmu::PageSize);
        if (!slot->transfer_ring) {
            LOG_DEBUGln("XHCI: Failed to allocate transfer ring for slot {}", slotId);
            return false;
        }

        std::memset(slot->transfer_ring, 0, TRANSFER_RING_SIZE * sizeof(TRB));
        slot->transfer_ring[TRANSFER_RING_SIZE - 1].parameter = get_physical_address(slot->transfer_ring);
        slot->transfer_ring[TRANSFER_RING_SIZE - 1].status = 0;
        slot->transfer_ring[TRANSFER_RING_SIZE - 1].control =
            (TRB_TYPE_LINK << 10) |
            TRB_CTRL_CYCLE |
            TRB_CTRL_TC;
        slot->transfer_ring_enqueue.store(0);
        slot->transfer_ring_dequeue.store(0);
        slot->transfer_ring_cycle_state = true;
        Processor::FlushDataCache(slot->transfer_ring, TRANSFER_RING_SIZE * sizeof(TRB));
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
            LOG_DEBUGln("XHCI: Failed to allocate control DMA buffer ({} bytes)", size);
            return false;
        }

        control_dma_buffer_size = static_cast<size_t>(pages) * Mmu::PageSize;
        std::memset(control_dma_buffer, 0, control_dma_buffer_size);
        Processor::FlushDataCache(control_dma_buffer, control_dma_buffer_size);
        return true;
    }

    static char const* GetTrbTypeName(uint32_t trbType) {
        switch (trbType) {
            case TRB_TYPE_NORMAL: return "Normal";
            case TRB_TYPE_SETUP: return "Setup";
            case TRB_TYPE_DATA: return "Data";
            case TRB_TYPE_STATUS: return "Status";
            case TRB_TYPE_LINK: return "Link";
            case TRB_TYPE_EVENT_DATA: return "Event Data";
            case TRB_TYPE_NOOP_CMD: return "NoOp Command";
            case TRB_TYPE_ENABLE_SLOT: return "Enable Slot";
            case TRB_TYPE_DISABLE_SLOT: return "Disable Slot";
            case TRB_TYPE_ADDRESS_DEV: return "Address Device";
            case TRB_TYPE_TRANSFER_EVENT: return "Transfer Event";
            case TRB_TYPE_CMD_COMPLETION_EVENT: return "Command Completion Event";
            default: return "Unknown";
        }
    }

    static char const* GetCompletionCodeName(uint32_t completionCode) {
        switch (completionCode) {
            case 0x00: return "Invalid";
            case 0x01: return "Success";
            case 0x02: return "Data Buffer Error";
            case 0x03: return "Babble Detected Error";
            case 0x04: return "USB Transaction Error";
            case 0x05: return "TRB Error";
            case 0x06: return "Stall Error";
            case 0x07: return "Resource Error";
            case 0x08: return "Bandwidth Error";
            case 0x09: return "No Slots Error";
            case 0x0A: return "Invalid Stream Type Error";
            case 0x0B: return "Slot Not Enabled Error";
            case 0x0C: return "Endpoint Not Enabled";
            case 0x0D: return "Short Packet";
            case 0x0E: return "Ring Underrun";
            case 0x0F: return "Ring Overrun";
            case 0x10: return "VF Event Ring Full Error";
            case 0x11: return "Parameter Error";
            case 0x12: return "Bandwidth Overrun";
            case 0x13: return "Context State Error";
            case 0x14: return "No Ping Response Error";
            case 0x15: return "Event Ring Full";
            case 0x16: return "Incompatible Device Error";
            case 0x17: return "Missing Device";
            case 0x18: return "Command Ring Stopped";
            case 0x19: return "Command Aborted";
            case 0x1A: return "Stopped";
            case 0x1B: return "Stopped";
            case 0x1C: return "Stopped";
            case 0x1D: return "Control Error";
            case 0x1E: return "Not Enough Bandwidth";
            case 0x1F: return "Isoc Burst Error";
            default: return "Unknown";
        }
    }

    static char const* GetBooleanName(bool value) {
        return value ? "true" : "false";
    }

    void PrintEventTrb(TRB const& event, char const* name = "")
    {
        uint32_t const trbType = (event.control >> 10) & 0x3Fu;
        uint32_t const slotId = (event.control >> 24) & 0xFFu;
        uint32_t const completionCode = (event.status >> 24) & 0xFFu;
        
        switch (trbType) {
            case TRB_TYPE_TRANSFER_EVENT: {
                bool const ioc = (event.control & TRB_CTRL_IOC) != 0;
                bool const chain = (event.control & TRB_CTRL_CHAIN) != 0;
                bool const idt = (event.control & TRB_CTRL_IDT) != 0;
                bool const dirIn = (event.control & TRB_CTRL_DIR_IN) != 0;
                uint32_t const endpointId = (event.control >> 16) & 0x1Fu;
                uint32_t const residualLength = event.status & 0x00FF'FFFFu;
                LOG_DEBUGln(
                    "XHCI: Transfer Event: type={} ({}), slot={}, endpoint={}, completion={} ({}), residualLength={}, IOC={}, chain={}, IDT={}, direction={}, status=0x{:08X}, parameter=0x{:016X}, control=0x{:08X}",
                    GetTrbTypeName(trbType), trbType,
                    slotId, endpointId,
                    GetCompletionCodeName(completionCode), completionCode,
                    residualLength,
                    GetBooleanName(ioc), GetBooleanName(chain), GetBooleanName(idt),
                    dirIn ? "IN" : "OUT",
                    event.status, event.parameter, event.control);
                return;
            }

            case TRB_TYPE_CMD_COMPLETION_EVENT: {
                uint64_t const commandTrbPointer = event.parameter;
                uint32_t const completionFlags = event.status & 0x00FF'FFFFu;
                LOG_DEBUGln(
                    "XHCI: Command Completion Event: type={} ({}), slot={}, completion={} ({}), completionFlags=0x{:06X}, commandTRB=0x{:016X}, status=0x{:08X}, control=0x{:08X}",
                    GetTrbTypeName(trbType), trbType,
                    slotId,
                    GetCompletionCodeName(completionCode), completionCode,
                    completionFlags,
                    commandTrbPointer,
                    event.status, event.control);
                return;
            }

            case 34u: { // Port Status Change Event
                uint32_t const portId = (event.control >> 24) & 0xFFu;
                LOG_DEBUGln(
                    "XHCI: Port Status Change Event: type={} ({}), port={}, completion={} ({}), status=0x{:08X}, parameter=0x{:016X}, control=0x{:08X}",
                    GetTrbTypeName(trbType), trbType,
                    portId,
                    GetCompletionCodeName(completionCode), completionCode,
                    event.status, event.parameter, event.control);
                return;
            }

            default: {
                LOG_DEBUGln(
                    "XHCI: {} TRB: type={} ({}), status=0x{:08X}, parameter=0x{:016X}, control=0x{:08X}",
                    name,
                    GetTrbTypeName(trbType), trbType,
                    event.status, event.parameter, event.control);
                return;
            }
        }
    }

    void PrintTrb(TRB const& event)
    {
        PrintEventTrb(event);
    }

    struct EventResult
    {
        Status         Result     = Status::Error;
        uint8_t        SlotId     = 0;
        TrbType        Type       = TrbType::Invalid;
        CompletionCode Completion = CompletionCode::Invalid;
    };

    EventResult GetNextEvent()
    {
        uint32_t const usbsts = operationalRegisters_.UsbStatus;
        if (usbsts & XHCI_STS_HSE) {
            LOG_DEBUGln("XHCI: Host system error while waiting for transfer event");
            LOG_DEBUGln("XHCI: USBCMD=0x{:08X} USBSTS=0x{:08X} CRCR=0x{:016X} ERSTBA=0x{:016X} ERDP=0x{:016X}",
                    operationalRegisters_.UsbCommand.get(),
                    usbsts,
                    get_command_ring_control(),
                    get_erstba(),
                    get_erdp());
            return EventResult{ .Result = Status::Error };
        }

        TRB* event = &event_ring[event_ring_dequeue.load()];
        Processor::InvalidateDataCache(event, sizeof(TRB));
        bool const cycle_bit = (event->control & TRB_CTRL_CYCLE) != 0;
        if (cycle_bit != event_ring_cycle_state) {
            return EventResult{ .Result = Status::NotFound };
        }

        PrintEventTrb(*event);

        EventResult result{
            .Result      = Status::Success,
            .SlotId      = static_cast<uint8_t>((event->control >> 24) & 0xFF),
            .Type        = static_cast<TrbType>((event->control >> 10) & 0x3F),
            .Completion  = static_cast<CompletionCode>((event->status >> 24) & 0xFF),
        };

        uint32_t const new_dequeue = advance_ring_pointer(event_ring_dequeue.load(), EVENT_RING_SIZE);
        event_ring_dequeue.store(new_dequeue);
        if (new_dequeue == 0) {
            event_ring_cycle_state = !event_ring_cycle_state;
        }
        set_erdp(get_physical_address(&event_ring[event_ring_dequeue.load()]) | (1ull << 3));

        return result;
    }


    Status wait_for_transfer_event(uint32_t slotId, std::chrono::microseconds timeout = 1s)
    {
        if (event_ring.empty())
        {
            LOG_DEBUGln("XHCI: Event ring is not initialized");
            return Status::Error;
        }

        auto result = Cpu::WaitUntilWithTimeout(timeout,
            [&]() -> std::optional<Status>
            {
                EventResult event = GetNextEvent();
                if (event.Result != Status::Success) {
                    if (event.Result == Status::NotFound) {
                        Cpu::Delay(50us);
                        return std::nullopt;
                    }

                    return event.Result;
                }

                if (event.SlotId != 0 && slotId != 0 && event.SlotId != slotId)
                {
                    return std::nullopt;
                }

                if (event.Type == TrbType::TransferEvent) {
                    if (event.Completion == CompletionCode::Success || event.Completion == CompletionCode::ShortPacket) {
                        return Status::Success;
                    }

                    if (event.Completion == CompletionCode::EndpointNotEnabled) {
                        LOG_DEBUGln("XHCI: Transfer event failed, completion={} (endpoint not enabled)", static_cast<uint32_t>(event.Completion));
                        return Status::Error;
                    }

                    LOG_DEBUGln("XHCI: Transfer event failed, completion={}", static_cast<uint32_t>(event.Completion));
                    return Status::Error;
                }

                if (event.Type == TrbType::CmdCompletionEvent) {
                    if (event.Completion != CompletionCode::Success) {
                        LOG_DEBUGln("XHCI: Command completion failed, completion={}", static_cast<uint32_t>(event.Completion));
                    }
                    return std::nullopt;
                }

                LOG_DEBUGln("XHCI: Ignoring event type={} completion={}", static_cast<uint32_t>(event.Type), static_cast<uint32_t>(event.Completion));
                return std::nullopt;
            }
        );
        if (result)
        {
            return result.value();
        }

        LOG_DEBUGln("XHCI: Timed out waiting for transfer completion event");
        size_t i = 0;
        for (auto& trb : event_ring)
        {
            if (trb != TRB{}) {
                LOG_DEBUGln("Event TRB[{}]: parameter=0x{:016X} status=0x{:08X} control=0x{:08X}", i, trb.parameter, trb.status, trb.control);
            }
            ++i;
        }
        return Status::Timeout;
    }

    bool wait_for_command_completion(uint32_t& slotId, std::chrono::microseconds timeout = 1s)
    {
        auto result = Cpu::WaitUntilWithTimeout(timeout,
            [&] -> std::optional<bool>
            {
                EventResult event = GetNextEvent();
                if (event.Result != Status::Success) {
                    if (event.Result == Status::NotFound) {
                        Cpu::Delay(50us);
                        return std::nullopt;
                    }

                    return false;
                }

                if (event.Type != TrbType::CmdCompletionEvent) {
                    return std::nullopt;
                }

                if (event.Completion != CompletionCode::Success) {
                    LOG_DEBUGln("XHCI: Command completion failed, completion={}", static_cast<uint32_t>(event.Completion));
                    return false;
                }

                slotId = event.SlotId;
                return true;
            }
        );
        if (result)
        {
            return result.value();
        }

        LOG_DEBUGln("XHCI: Timed out waiting for command completion event");
        LOG_DEBUGln("XHCI: USBSTS=0x{:08X} IMAN=0x{:08X} ERDP=0x{:016X}",
            operationalRegisters_.UsbStatus.get(),
            runtimeRegisters_.InterrupterManagement.get(),
            get_erdp()
        );
        return false;
    }
    
    bool send_command(TRB command_trb) {
        std::lock_guard<std::mutex> lock(command_mutex);

        uint32_t const usbsts_before = operationalRegisters_.UsbStatus;
        uint32_t usbcmd_before = operationalRegisters_.UsbCommand;
        if (usbsts_before & XHCI_STS_HSE) {
            LOG_DEBUGln("XHCI: Refusing command submit while HSE is set (USBSTS=0x{:08X})", usbsts_before);
            return false;
        }

        if (usbsts_before & XHCI_STS_HCH) {
            LOG_DEBUGln("XHCI: Controller halted before command submit, attempting restart");
            operationalRegisters_.UsbCommand |= XHCI_CMD_RUN;
            Cpu::Delay(100us);
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

         LOG_DEBUGln("XHCI: Command submitted Control=0x{:08X} Status=0x{:08X} Parameter=0x{:016X} (USBCMD=0x{:08X} USBSTS=0x{:08X} CRCR=0x{:016X})",
            command_trb.control,
            command_trb.status,
            command_trb.parameter,
            usbcmd_before,
            usbsts_before,
            get_command_ring_control());
        
        return true;
    }
    
    uint32_t allocate_device_slot()
    {

        if (!send_command(TRB{ .control = TRB_TYPE_ENABLE_SLOT << 10 }))
        {
            return 0;
        }
        LOG_DEBUGln("XHCI: Enable slot command sent");

        uint32_t slotId = 0;
        if (!wait_for_command_completion(slotId) || slotId == 0) {
            LOG_DEBUGln("XHCI: Enable slot command did not complete successfully");
            return false;
        }

        LOG_DEBUGln("XHCI: Enabled slot {}", slotId);

        auto* slot = deviceSlots_[slotId];
        if (slot == nullptr)
        {
            LOG_DEBUGln("XHCI: Invalid slot {} for allocation", slotId);
            return 0;
        }

        bool expected = false;
        if (slot->in_use.compare_exchange_strong(expected, true))
         {
            slot->slotId = slotId;
            return slotId;
        }

        LOG_DEBUGln("XHCI: Failed to enable slot {} in the slot table", slotId);

        return 0; // No available slots
    }
    
    void free_device_slot(uint32_t slotId)
    {
        auto* slot = deviceSlots_[slotId];
        if (slot == nullptr)
        {
            LOG_DEBUGln("XHCI: Invalid slot {} for free_device_slot", slotId);
            return;
        }
        slot->in_use.store(false);
        slot->slotId = 0;
    }
    
    DeviceInfo remember_discovered_device(uint32_t slotId, uint32_t port, uint32_t root_hub_port, uint32_t speed,
                                     std::span<uint8_t const> descriptor,
                                     std::span<uint8_t const> config_descriptor)
    {
        if (descriptor.empty()) {
            return DeviceInfo{};
        }

        DeviceInfo info{};
        info.SlotId = slotId;
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

        return info;
    }

    bool wait_for_ready(std::chrono::microseconds timeout = 1s)
    {
        uint32_t status = 0;
        if (!Cpu::WaitUntilWithTimeout(timeout, [&]{
                status = operationalRegisters_.UsbStatus;
                return !(status & XHCI_STS_CNR);
            }))
        {
            LOG_DEBUGln("Timed out waiting for ready state. Command: {:#X}, Status: {:#X}", operationalRegisters_.UsbCommand.get(), status);
            return false;
        }
        else
        {
            return true;
        }
    }

    bool reset_controller()
    {
        wait_for_ready(5s);

        LOG_DEBUGln("Command: {:#X}, Status: {:#X}", operationalRegisters_.UsbCommand.get(), operationalRegisters_.UsbStatus.get());

        LOG_DEBUGln("XHCI: Resetting controller...");
        
        // Stop the controller first
        operationalRegisters_.UsbCommand &= ~XHCI_CMD_RUN;
        
        // Wait for halt
        uint32_t status = 0;
        if (!Cpu::WaitUntilWithTimeout(5s, [&]{
                status = operationalRegisters_.UsbStatus;
                return (status & XHCI_STS_HCH) != 0;
            }))
        {
            LOG_DEBUGln("Timed out waiting for halt state. Command: {:#X}, Status: {:#X}", operationalRegisters_.UsbCommand.get(), status);
        }
        
        LOG_DEBUGln("XHCI: Controller halted...");

        // Reset the controller
        operationalRegisters_.UsbCommand |= XHCI_CMD_HCRST;
        
        // Wait for reset to complete
        uint32_t cmd = 0;
        if (!Cpu::WaitUntilWithTimeout(5s, [&]{
                cmd = operationalRegisters_.UsbCommand;
                return !(cmd & XHCI_CMD_HCRST);
            }))
        {
            LOG_DEBUGln("XHCI: Reset timeout");
            LOG_DEBUGln("Command: {:#X}, Status: {:#X}", cmd, status);
            return false;
        }
        
        return wait_for_ready();
    }

    Status shutdown()
    {
        LOG_DEBUGln("XHCI: Shutting down controller...");
        
        // Stop the controller
        operationalRegisters_.UsbCommand &= ~XHCI_CMD_RUN;
        
        // Wait for halt
        if (!Cpu::WaitUntilWithTimeout(1s, [&]{
                uint32_t status = operationalRegisters_.UsbStatus;
                if (status & XHCI_STS_HCH) {
                    LOG_DEBUGln("XHCI: Controller halted");
                    return true;
                }
                else
                {
                    return false;
                }
            }))
        {
            LOG_DEBUGln("XHCI: Shutdown timeout");
            return Status::Timeout;
        }

        return Status::Success;
    }

    Status read(DeviceSlot& slot, uint8_t endpoint, std::span<uint8_t> buffer)
    {
        LOG_DEBUGln("XHCI: Read from endpoint {} (buffer size: {} bytes)", endpoint, buffer.size());

        if (buffer.empty()) {
            return Status::Error;
        }

        if (!ensure_transfer_ring(slot.slotId)) {
            return Status::Error;
        }

        if (!ensure_control_dma_buffer(buffer.size())) {
            return Status::Error;
        }

        std::memset(control_dma_buffer, 0, buffer.size());
        Processor::FlushDataCache(control_dma_buffer, buffer.size());

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

        ring_doorbell(slot.slotId, endpoint);
        Status const transfer_status = wait_for_transfer_event(slot.slotId);
        if (transfer_status == Status::Success) {
            Processor::InvalidateDataCache(control_dma_buffer, buffer.size());
            std::memcpy(buffer.data(), control_dma_buffer, buffer.size());
        }
        return transfer_status;
    }

    Status write(DeviceSlot& slot, uint8_t endpoint, std::span<uint8_t const> data)
    {
        LOG_DEBUGln("XHCI: Write to endpoint {} (data size: {} bytes)", endpoint, data.size());

        if (data.empty()) {
            return Status::Error;
        }

        if (!ensure_transfer_ring(slot.slotId)) {
            return Status::Error;
        }

        if (!ensure_control_dma_buffer(data.size())) {
            return Status::Error;
        }

        std::memcpy(control_dma_buffer, data.data(), data.size());
        Processor::FlushDataCache(control_dma_buffer, data.size());

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

        ring_doorbell(slot.slotId, endpoint);
        return wait_for_transfer_event(slot.slotId);
    }

    Status controlTransfer(uint8_t slotId, uint8_t requestType, uint8_t request,
                           uint16_t value, uint16_t index,
                           std::span<uint8_t> data = {})
    {
        LOG_DEBUGln("XHCI: Control transfer on slot {} - Type: 0x{:02X}, Request: 0x{:02X}, Value: 0x{:04X}, Index: 0x{:04X}, Length: {}",
               slotId, requestType, request, value, index, data.size()
        );

        if (!ensure_transfer_ring(slotId)) {
            return Status::Error;
        }

        if (!ensure_control_dma_buffer(data.size())) {
            return Status::Error;
        }

        auto* slot = deviceSlots_[slotId];
        if (slot == nullptr)
        {
            LOG_DEBUGln("XHCI: Invalid device slot");
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

        {
            std::lock_guard<std::mutex> lock(transfer_mutex);

            uint32_t setup_pos = slot->transfer_ring_enqueue.load();
            TRB& setup_trb = slot->transfer_ring[setup_pos];
            setup_trb.parameter = setup_packet_data;
            setup_trb.status = 8;
            setup_trb.control =
                (TRB_TYPE_SETUP << 10) |
                (trt << 16) |
                TRB_CTRL_CHAIN |
                TRB_CTRL_IDT |
                (slot->transfer_ring_cycle_state ? TRB_CTRL_CYCLE : 0);
            LOG_DEBUG("XHCI: Setup TRB - ");
            PrintTrb(setup_trb);
            //printf("XHCI: Setup TRB - 0x%016X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", &setup_trb,
            //    reinterpret_cast<uint8_t*>(&setup_trb)[0], reinterpret_cast<uint8_t*>(&setup_trb)[1], reinterpret_cast<uint8_t*>(&setup_trb)[2], reinterpret_cast<uint8_t*>(&setup_trb)[3],
            //    reinterpret_cast<uint8_t*>(&setup_trb)[4], reinterpret_cast<uint8_t*>(&setup_trb)[5], reinterpret_cast<uint8_t*>(&setup_trb)[6], reinterpret_cast<uint8_t*>(&setup_trb)[7],
            //    reinterpret_cast<uint8_t*>(&setup_trb)[8], reinterpret_cast<uint8_t*>(&setup_trb)[9], reinterpret_cast<uint8_t*>(&setup_trb)[10], reinterpret_cast<uint8_t*>(&setup_trb)[11],
            //    reinterpret_cast<uint8_t*>(&setup_trb)[12], reinterpret_cast<uint8_t*>(&setup_trb)[13], reinterpret_cast<uint8_t*>(&setup_trb)[14], reinterpret_cast<uint8_t*>(&setup_trb)[15]
            //);
            Processor::FlushDataCache(&setup_trb, sizeof(TRB));

            uint32_t next_pos = advance_transfer_ring_pointer(setup_pos);
            slot->transfer_ring_enqueue.store(next_pos);
            if (next_pos == 0) {
                slot->transfer_ring_cycle_state = !slot->transfer_ring_cycle_state;
            }

            if (!data.empty()) {
                uint32_t data_pos = slot->transfer_ring_enqueue.load();
                TRB& data_trb = slot->transfer_ring[data_pos];
                data_trb.parameter = get_physical_address(dma_data_ptr);
                data_trb.status = static_cast<uint32_t>(data.size());
                data_trb.control = (TRB_TYPE_DATA << 10) | TRB_CTRL_CHAIN | (slot->transfer_ring_cycle_state ? TRB_CTRL_CYCLE : 0);
                if (data_stage_in) {
                    data_trb.control |= TRB_CTRL_DIR_IN;
                }
                LOG_DEBUG("XHCI: Data TRB - ");
                PrintTrb(data_trb);
                //printf("XHCI: Data TRB - 0x%016X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", &data_trb,
                //    reinterpret_cast<uint8_t*>(&data_trb)[0], reinterpret_cast<uint8_t*>(&data_trb)[1], reinterpret_cast<uint8_t*>(&data_trb)[2], reinterpret_cast<uint8_t*>(&data_trb)[3],
                //    reinterpret_cast<uint8_t*>(&data_trb)[4], reinterpret_cast<uint8_t*>(&data_trb)[5], reinterpret_cast<uint8_t*>(&data_trb)[6], reinterpret_cast<uint8_t*>(&data_trb)[7],
                //    reinterpret_cast<uint8_t*>(&data_trb)[8], reinterpret_cast<uint8_t*>(&data_trb)[9], reinterpret_cast<uint8_t*>(&data_trb)[10], reinterpret_cast<uint8_t*>(&data_trb)[11],
                //    reinterpret_cast<uint8_t*>(&data_trb)[12], reinterpret_cast<uint8_t*>(&data_trb)[13], reinterpret_cast<uint8_t*>(&data_trb)[14], reinterpret_cast<uint8_t*>(&data_trb)[15]
                //);

                Processor::FlushDataCache(&data_trb, sizeof(TRB));

                next_pos = advance_transfer_ring_pointer(data_pos);
                slot->transfer_ring_enqueue.store(next_pos);
                if (next_pos == 0) {
                    slot->transfer_ring_cycle_state = !slot->transfer_ring_cycle_state;
                }
            }

            uint32_t status_pos = slot->transfer_ring_enqueue.load();
            TRB& status_trb = slot->transfer_ring[status_pos];
            status_trb.parameter = 0;
            status_trb.status = 0;
            status_trb.control =
            (TRB_TYPE_STATUS << 10) |
            TRB_CTRL_IOC |
            ((!data_stage_present || !data_stage_in) ? TRB_CTRL_DIR_IN : 0) |
            (slot->transfer_ring_cycle_state ? TRB_CTRL_CYCLE : 0);
            LOG_DEBUG("XHCI: Status TRB - ");
            PrintTrb(status_trb);
            //printf("XHCI: Status TRB - 0x%016X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", &status_trb,
            //    reinterpret_cast<uint8_t*>(&status_trb)[0], reinterpret_cast<uint8_t*>(&status_trb)[1], reinterpret_cast<uint8_t*>(&status_trb)[2], reinterpret_cast<uint8_t*>(&status_trb)[3],
            //    reinterpret_cast<uint8_t*>(&status_trb)[4], reinterpret_cast<uint8_t*>(&status_trb)[5], reinterpret_cast<uint8_t*>(&status_trb)[6], reinterpret_cast<uint8_t*>(&status_trb)[7],
            //    reinterpret_cast<uint8_t*>(&status_trb)[8], reinterpret_cast<uint8_t*>(&status_trb)[9], reinterpret_cast<uint8_t*>(&status_trb)[10], reinterpret_cast<uint8_t*>(&status_trb)[11],
            //    reinterpret_cast<uint8_t*>(&status_trb)[12], reinterpret_cast<uint8_t*>(&status_trb)[13], reinterpret_cast<uint8_t*>(&status_trb)[14], reinterpret_cast<uint8_t*>(&status_trb)[15]
            //);
            Processor::FlushDataCache(&status_trb, sizeof(TRB));

            next_pos = advance_transfer_ring_pointer(status_pos);
            slot->transfer_ring_enqueue.store(next_pos);
            if (next_pos == 0) {
                slot->transfer_ring_cycle_state = !slot->transfer_ring_cycle_state;
            }
        }

        // Doorbell 0 is the command ring; transfer rings use the slot's doorbell.
        ring_doorbell(slotId, 1);

        Status const transfer_status = wait_for_transfer_event(slotId);
        LOG_DEBUGln("XHCI: Control transfer completed with status {}",
            transfer_status == Status::Success  ? "Success" :
            transfer_status == Status::Timeout  ? "Timeout" :
            transfer_status == Status::Error    ? "Error" :
            /*transfer_status == Status::NotFound ?*/ "NotFound");
        if (/*transfer_status == Status::Success &&*/ data_stage_in && !data.empty()) {
            Processor::InvalidateDataCache(dma_data_ptr, data.size());
            std::memcpy(data.data(), dma_data_ptr, data.size());

            LOG_DEBUG("Data:");
            for (auto data_byte : data) {
                LOG_DEBUG(" {:02X}", data_byte);
            }
            LOG_DEBUGln("");
        }
        return transfer_status;
    }
    
    // Public method to manually process pending events (useful for testing)
    void process_pending_events()
    {
        process_events();
    }
    
    // Additional utility methods
    uint32_t get_port_status(uint32_t port)
    {
        if (port == 0 || port > maxRootPorts) {
            return 0;
        }
        
        return operationalRegisters_.PortStatusControl[port - 1];
    }
    
    void reset_port(uint32_t port) {
        if (port == 0 || port > maxRootPorts) {
            return;
        }
        
        LOG_DEBUGln("XHCI: Resetting port {}", port);
        
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
            Cpu::Delay(100us);
        }

        uint32_t const final_portsc = portscReg;
        LOG_DEBUGln("XHCI: Port {} reset timeout, PORTSC=0x{:08X}", port, final_portsc);
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

    void print_device_summary(uint32_t port, uint32_t slotId, std::span<uint8_t const> descriptor, std::span<uint8_t const> config_descriptor) {
        if (descriptor.size() < 18) {
            LOG_DEBUGln("XHCI: Port {} slot {}: descriptor too small to parse", port, slotId);
            return;
        }

        uint8_t  const bLength            = descriptor[0];
        uint8_t  const bDescriptorType    = descriptor[1];
        uint8_t  const bDeviceClass       = descriptor[4];
        uint8_t  const bDeviceSubClass    = descriptor[5];
        uint8_t  const bDeviceProtocol    = descriptor[6];
        uint8_t  const bMaxPacketSize0    = descriptor[7];
        uint16_t const GetVendorId          = static_cast<uint16_t>(descriptor[8] | (descriptor[9] << 8));
        uint16_t const product_id         = static_cast<uint16_t>(descriptor[10] | (descriptor[11] << 8));
        uint8_t  const bNumConfigurations = descriptor[17];

        LOG_DEBUGln("XHCI: Port {} -> device slot {} identified", port, slotId);
        LOG_DEBUGln("XHCI:   Function: {} (class=0x{:02X}, subclass=0x{:02X}, protocol=0x{:02X})",
               describe_device_class(bDeviceClass), bDeviceClass, bDeviceSubClass, bDeviceProtocol);
        LOG_DEBUGln("XHCI:   Vendor/Product: 0x{:04X} / 0x{:04X}", GetVendorId, product_id);
        LOG_DEBUGln("XHCI:   Capabilities: max-packet-size0=0x{:02X}, configurations={}",
               bMaxPacketSize0, bNumConfigurations);

        LOG_DEBUG("XHCI:   Descriptor length: {}, type: 0x{:02X}", bLength, bDescriptorType);
        for (size_t i = 0; i < std::min<size_t>(descriptor.size(), 18); ++i) {
            if (i % 16 == 0) {
                LOG_DEBUG("\nXHCI:   ");
            }
            LOG_DEBUG(" {:02X}", descriptor[i]);
        }
        LOG_DEBUGln("");

        if (config_descriptor.size() >= 9) {
            uint8_t const config_length  = config_descriptor[0];
            uint8_t const config_type    = config_descriptor[1];
            uint8_t const num_interfaces = config_descriptor[4];
            uint8_t const config_value   = config_descriptor[5];
            uint8_t const max_power      = config_descriptor[8];
            LOG_DEBUGln("XHCI:   Configuration: value=0x{:02X}, interfaces={}, max-power={}mA, attrs=0x{:02X}",
                   config_value, num_interfaces, max_power * 2u, config_descriptor[7]);
            if (config_length < 9) {
                LOG_DEBUGln("XHCI:   Interface data is incomplete");
            }
        }
        else {
            LOG_DEBUGln("XHCI:   Configuration descriptor unavailable");
        }

        if (bLength < 18) {
            LOG_DEBUGln("XHCI:   Descriptor length is shorter than expected ({})", bLength);
        }
    }

    bool initialize_root_port(uint32_t port)
    {
        uint32_t portsc = get_port_status(port);
        bool     const connected    = (portsc & 1u) != 0;
        bool     const enabled      = (portsc & (1u << 1)) != 0;
        bool     const over_current = (portsc & (1u << 3)) != 0;
        uint32_t const speed        = (portsc >> 10) & 0x0F;

        LOGln("XHCI: Port {} status: 0x{:08X} [connected={} enabled={} speed={} over-current={}]",
                port, portsc, connected ? 1u : 0u, enabled ? 1u : 0u, speed, over_current ? 1u : 0u);

        if (!connected) {
            LOGln("XHCI: Port {} has no device attached", port);
            return false;
        }

        reset_port(port);
        return true;
    }

    bool initialize_device_at_port(uint32_t slotId, uint32_t rootPort, uint32_t speed, uint32_t hubSlotId, uint32_t hubPort)
    {
        if (hubSlotId == 0)
        {
            LOG_DEBUGln("XHCI: Enumerating device in root port {}", rootPort);
        }
        else
        {
            LOG_DEBUGln("XHCI: Enumerating device in hub at slot {} port {} using root port {}", hubSlotId, hubPort, rootPort);
        }

        if (!address_device(slotId, rootPort, speed, 0, 0, 0, true))
        {
            LOG_DEBUGln("XHCI: Failed to address device on root port {} (slot {})", rootPort, slotId);
            free_device_slot(slotId);
            return false;
        }

//        remember_discovered_device(slotId, hubPort, rootPort, speed, {}, {});
        //print_device_summary(hubPort, slotId, {}, {});

        std::array<uint8_t, 64> device_descriptor{};
        //if (controlTransfer(slotId, 0x80, 0x06, 0x0100, 0, device_descriptor) != Status::Success)
        //{
        //    printf("XHCI: Port %u could not retrieve a device descriptor\n", hubPort);
        //    return false;
        //}
        std::array<uint8_t, 64> config_descriptor{};
        //DeviceInfo deviceInfo{};
//        if (controlTransfer(slotId, 0x80, 0x06, 0x0200, 0, config_descriptor) == Status::Success) {
//            //deviceInfo = remember_discovered_device(slotId, hubPort, rootPort, speed, device_descriptor, config_descriptor);
//            print_device_summary(hubPort, slotId, device_descriptor, config_descriptor);
//
//            //controlTransfer(slotId, 0x00, 0x09, config_descriptor[5], 0, {});
//        }
//        else
        {
            //deviceInfo = remember_discovered_device(slotId, hubPort, rootPort, speed, device_descriptor, {});
            print_device_summary(hubPort, slotId, device_descriptor, {});

            //controlTransfer(slotId, 0x00, 0x09, 1, 0, {});
        }

        return true;
    }

    std::span<DeviceInfo const> discovered_devices() const
    {
        return discovered_devices_;
    }

    bool run_hello_world_test()
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
            LOG_DEBUGln("Test Failed: Control transfer returned status {}.", static_cast<int>(result));
            return false;
        }

        // Check 3b: Data integrity (The most critical check)
        // The driver should have populated the buffer with the device descriptor data.
        // We check for a known signature or a specific field (e.g., bLength).
        
        // Check if the buffer is non-empty and contains a valid descriptor length (e.g., > 18 bytes)
        if (data_buffer[0] < 18) {
            LOG_DEBUGln("Test Failed: Received descriptor length is too short or zero.");
            return false;
        }
        
        // Check a specific field, e.g., bDescriptorType (should be 1 for Device Descriptor)
        if (data_buffer[1] != 1) {
            LOG_DEBUGln("Test Failed: Descriptor type mismatch. Expected 1, got {}.", data_buffer[1]);
            return false;
        }
        
        // 4. Event/Interrupt Verification (Optional but recommended)
        // If the driver relies on asynchronous event processing, we must ensure the event was processed.
        // This might involve calling a driver-specific function to drain the event queue.
        process_pending_events(); 
        
        // If the test reaches this point, the command was sent, the hardware responded, 
        // the driver processed the event, and the data was correctly received.
        LOG_DEBUGln("Test Succeeded: Minimal control transfer completed successfully.");
        return true;
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

    std::expected<std::shared_ptr<UsbDevice>, RESULT> UsbAllocateDevice(UsbDevice* parentHubDevice, uint8_t parentHubPort) override
    {
        auto const slotId = allocate_device_slot();

        LOG_DEBUGln("XHCI: UsbAllocateDevice: allocate slot completion returned slot ID {}", slotId);
        if (slotId == 0)
        {
            return std::unexpected(RESULT::ErrorMemory);
        }

        uint32_t route           = 0;
        uint32_t parentHubSlotId = 0;
        uint32_t rootPort        = parentHubPort;
        uint32_t speed = 0;
        
        if (parentHubDevice != nullptr && parentHubPort > 0)
        {
            route           = parentHubPort;
            parentHubSlotId = parentHubDevice->GetAddress();
            rootPort        = parentHubDevice->RootHubPort;

            LOG_DEBUG("Reset port %u of hub %u\n",  parentHubPort, parentHubSlotId);

            // Reset the port for what will be the second time.
            if (auto const result = Async::WaitOnTask(HubPortReset(*parentHubDevice, parentHubPort - 1)); !result.has_value()) {
                LOG("HCD: Failed to reset port again for new device at port %u of hub %u.\n", parentHubPort, parentHubSlotId);
                return std::unexpected(result.error());
            }
            else
            {
                if (result.value().Status.HighSpeedAttatched)
                {
                    speed = 3;
                }
                else if (result.value().Status.LowSpeedAttatched)
                {
                    speed = 2;
                }
                else
                {
                    speed = 1;
                }
            }

            auto parentHub = parentHubDevice;
            for (uint32_t i = 0; i < 4 && parentHub->ParentHub.Device; ++i)
            {
                route = (route << 4) | (parentHub->ParentHub.PortNumber & 0x0F);
                parentHub = parentHub->ParentHub.Device.get();
            }
        }
        else
        {
            uint32_t const post_reset_portsc = get_port_status(rootPort);
            speed = (post_reset_portsc >> 10) & 0x0F;
        }

        if (!address_device(slotId, rootPort, speed, route, parentHubSlotId, parentHubPort, false))
        {
            LOG_DEBUGln("XHCI: Failed to address device on root port {} (slot {}) route {:05X} parentHubSlotId {} parentHubPort {}", rootPort, slotId, route, parentHubSlotId, parentHubPort);
            return std::unexpected(RESULT::ErrorDevice);
        }

        std::shared_ptr<UsbDevice> device = std::make_shared<UsbDevice>(slotId, DriverRef());
        if (!device)
        {
            return std::unexpected(RESULT::ErrorMemory);
        }

        device->RootHubPort = rootPort;
        device->Config.Status = USB_STATUS_ADDRESSED;
        device->ParentHub.PortNumber = parentHubPort;
        device->ParentHub.Device = parentHubDevice ? deviceTable_[parentHubDevice->GetAddress() - 1] : nullptr;
        device->PayLoadId = PayLoadType::None;

        if (deviceTable_.size() < slotId)
        {
            deviceTable_.resize(slotId);
        }

        deviceTable_[slotId - 1] = device;

        Cpu::Delay(20ms);

        return std::move(device);
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
    Async::task<RESULT> HCDSetAddress(UsbDevice& device, IoHandle const& ioHandle) override
    {
//        auto slot = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(ioHandle.get()));
//        if (slot == 0 || slot != LookupSlot(&device)) {
//            co_return RESULT::ErrorDevice;
//        }
//        uint32_t route = device.ParentHub.PortNumber;
//        uint32_t parentHubSlotId = 0;
//        uint32_t parentHubPort = device.ParentHub.PortNumber;
//
//        if (device.ParentHub.Device)
//        {
//            route = device.ParentHub.PortNumber;
//            auto parentHub = device.ParentHub.Device.get();
//            parentHubSlotId = parentHub->GetAddress();
//            for (uint32_t i = 1; i <= 5 && parentHub->ParentHub.Device; ++i)
//            {
//                route = (route << 5) | (parentHub->ParentHub.PortNumber & 0x1F);
//                parentHub = parentHub->ParentHub.Device.get();
//            }
//        }
//
//        uint32_t const address_port = device.RootHubPort;
//        uint32_t const post_reset_portsc = get_port_status(address_port);
//        uint32_t const post_reset_speed = (post_reset_portsc >> 10) & 0x0F;
//        if (!address_device(slot, address_port, post_reset_speed, route, parentHubSlotId, parentHubPort, false))
//        {
//            printf("XHCI: Failed to address device on root port %u (slot %u) route %05X parentHubSlotId %u parentHubPort %u\n", address_port, slot, route, parentHubSlotId, parentHubPort);
//            co_return RESULT::ErrorDevice;
//        }
        co_return RESULT::Ok;
    }

    Async::task<> Initialize()
    {
        LOG_DEBUGln("XHCI: Initializing controller...");

        // Test memory access first
        LOG_DEBUGln("XHCI: Testing memory access...");
        uint32_t test_value = *reinterpret_cast<uint32_t volatile*>(&capabilityRegisters_);
        LOG_DEBUGln("XHCI: First word: 0x{:X}", test_value);
        if (test_value == 0xdeaddead || test_value == 0xffffffff || test_value == 0x00000000) {
            LOG_DEBUGln("XHCI: Invalid response, device not accessible");
            error_ = RESULT::ErrorHardware;
            co_return;
        }
        LOG_DEBUGln("XHCI: Valid response, proceeding with initialization");

        // Read the controller parameters
        HcsParams1 hcsparams1 = capabilityRegisters_.StructuralParams1;
        HcsParams2 hcsparams2 = capabilityRegisters_.StructuralParams2;
        HccParams1 hccparams1 = capabilityRegisters_.CapabilityParams1;

        size_t maxDeviceSlots  = hcsparams1.MaxDeviceSlots;
        size_t maxInterrupters = hcsparams1.MaxInterrupters;
        maxRootPorts           = hcsparams1.MaxPorts;
        maxScratchpadBuffers   = (hcsparams2.MaxScratchpadBuffersHi << 5) + hcsparams2.MaxScratchpadBuffersLo;

        LOGln("XHCI: Capability length: 0x{:X}", capabilityRegisters_.CapLength.get());
        LOGln("XHCI: HCI Version      : 0x{:X}", capabilityRegisters_.InterfaceVersion.get());
        LOGln("XHCI: StructuralParams1: 0x{:X}", hcsparams1.Raw32);
        LOGln("XHCI: StructuralParams2: 0x{:X}", hcsparams2.Raw32);
        LOGln("XHCI: StructuralParams3: 0x{:X}", capabilityRegisters_.StructuralParams3.get().Raw32);
        LOGln("XHCI: CapabilityParams1: 0x{:X}", hccparams1.Raw32);
        LOGln("XHCI: DoorbellOffset   : 0x{:X}", capabilityRegisters_.DoorbellOffset.get());
        LOGln("XHCI: RuntimeOffset    : 0x{:X}", capabilityRegisters_.RuntimeOffset.get());
        LOGln("XHCI: CapabilityParams2: 0x{:X}", capabilityRegisters_.CapabilityParams2.get());

        LOGln("XHCI: Max device slots: {}", maxDeviceSlots);
        LOGln("XHCI: Max interrupters: {}" , maxInterrupters);
        LOGln("XHCI: Max root ports  : {}" , maxRootPorts);

        LOGln("XHCI: Isochronous scheduling threshold: {} {}", static_cast<uint32_t>(hcsparams2.IsoSchedThreshold), hcsparams2.IsoSchedThresholdIsInFrames ? "frames" : "microframes");
        LOGln("XHCI: Event ring segment table max: {}", static_cast<uint32_t>(hcsparams2.EventRingSegmentTableMax));
        LOGln("XHCI: Max scratchpad buffers: {}", maxScratchpadBuffers);
        LOGln("XHCI: Save/restore uses scratchpad: {}", static_cast<bool>(hcsparams2.SaveRestoreUsesScratchpad));
        LOGln("XHCI: Doorbells offset: 0x{:X}", capabilityRegisters_.DoorbellOffset.get());
        LOGln("XHCI: Runtime offset: 0x{:X}", capabilityRegisters_.RuntimeOffset.get());
        LOGln("");

        LOGln("Command: 0x{:X}, Status: 0x{:X}", operationalRegisters_.UsbCommand.get(), operationalRegisters_.UsbStatus.get());

        deviceSlots_.Init(maxDeviceSlots);
        // TODO: Also these:
        // - Interrupters
        // - RootPorts
        // - ScratchpadBuffers

        // Reset the controller
        if (!reset_controller())
        {
            LOG_DEBUGln("XHCI: Controller reset failed");
            error_ = RESULT::ErrorHardware;
            co_return;
        }
        
        // Setup memory structures
        if (!setup_rings())
        {
            LOG_DEBUGln("XHCI: Ring setup failed");
            error_ = RESULT::ErrorHardware;
            co_return;
        }

        controller_hccparams1 = hccparams1;
        
        // Set number of device slots
        operationalRegisters_.Configure = [&](auto& reg){ reg.MaxDeviceSlotsEnabled = static_cast<uint8_t>(deviceSlots_.size() - 1); };

        // Clear sticky status events before enabling run/interrupts.
        uint32_t const sticky = operationalRegisters_.UsbStatus.get() &
                                (XHCI_STS_HSE | XHCI_STS_EINT | XHCI_STS_PCD | XHCI_STS_SSS | XHCI_STS_RSS | XHCI_STS_SRE | XHCI_STS_HCE);
        if (sticky != 0)
        {
            operationalRegisters_.UsbStatus = sticky;
        }
        
        // Enable interrupts
        operationalRegisters_.UsbCommand |= XHCI_CMD_INTE;
        
        // Register interrupt handler
        // TODO: Interrupts::EnableXhci(xhci_interrupt_handler);
        
        // Start the controller
        operationalRegisters_.UsbCommand |= XHCI_CMD_RUN;
        
        // Wait for controller to start and leave halted state.
        auto start_time = Cpu::GetPerformanceCounter();
        auto timeout_ticks = Cpu::ToTicks(1s);

        bool running = false;
        while ((Cpu::GetPerformanceCounter() - start_time) < timeout_ticks)
        {
            uint32_t const status = operationalRegisters_.UsbStatus;
            // We're waiting until the controller is no longer in the "Controller Not Ready" or "Halted" state.
            if ((status & (XHCI_STS_CNR | XHCI_STS_HCH)) == 0)
            {
                running = true;
                break;
            }
            co_await Async::Delay(50us);
        }

        if (!running)
        {
            LOG_DEBUGln("XHCI: Controller did not enter running state");
            LOG_DEBUGln("XHCI: USBCMD=0x{:08X} USBSTS=0x{:08X}",
                   operationalRegisters_.UsbCommand.get(),
                   operationalRegisters_.UsbStatus.get());
            error_ = RESULT::ErrorHardware;
            co_return;
        }
        
        LOGln("XHCI: Controller initialized successfully");
        // Start device enumeration
        LOGln("XHCI: Enumerating devices...");
        
        for (uint32_t port = 1; port <= maxRootPorts; ++port)
        {
            if (!initialize_root_port(port))
            {
                continue;
            }

            auto deviceEx = UsbAllocateDevice(nullptr, port);
            if (!deviceEx) {
                error_ = deviceEx.error();
                co_return;
            }
            auto& device = *deviceEx.value();
            //device.Descriptor = info.Descriptor;
            //device.Interfaces = info.Interfaces;
            //device.Endpoints = info.Endpoints;
            //device.Config.Status = USB_STATUS_ATTACHED;
            //if (info.Descriptor.bDeviceClass == DeviceClassHub) {
            //    device.PayLoadId = PayLoadType::Hub;
            //}
            //else if (info.Descriptor.bDeviceClass == DeviceClassInInterface || info.Descriptor.bDeviceClass == 0x03) {
            //    device.PayLoadId = PayLoadType::Hid;
            //    device.HidPayload = AllocateHidPayload();
            //}
            //else
            {
                device.PayLoadId = PayLoadType::None;
            }

            auto const result = co_await EnumerateDevice(device);
            if (result != RESULT::Ok)
            {
                LOG("FATAL ERROR: Could not enumerate root port %u\n", port);
                error_ = result;
                co_return;
            }
        }
        
        LOG_DEBUGln("discovered devices: {}", deviceTable_.size());

        error_ = RESULT::Ok;
        co_return;
    }

    Async::task<IoHandle> InitializeDevice(UsbDevice& device) override
    {
        //if (device.ParentHub.Device)
        //{
        //    initialize_device_at_port(device.GetAddress(), device.RootHubPort, device.ParentHub.Device->GetAddress(), device.ParentHub.PortNumber);
        //}
        //else
        //{
        //    // Root device.
        //    initialize_device_at_port(device.GetAddress(), device.RootHubPort, 0, 0);
        //}
        co_return GetIoHandle(device.GetAddress());
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
        LOG_DEBUGln("ioHandle slot = {}, device slot = {}", slot, LookupSlot(device));
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

        auto* const slot = deviceSlots_[device->GetAddress()];
        if (slot == nullptr)
        {
            co_return RESULT::ErrorDevice;
        }

        auto const direction = endpoint.EndpointAddress.Direction;
        if (direction == USB_DIRECTION_IN) {
            auto const status = read(*slot, endpoint.EndpointAddress.Number, std::span<uint8_t>(reinterpret_cast<uint8_t*>(buffer), bufferLength));
            co_return status == Usb::Status::Success ? RESULT::Ok : RESULT::ErrorTransmission;
        }

        auto const status = write(*slot, endpoint.EndpointAddress.Number, std::span<uint8_t const>(reinterpret_cast<uint8_t const*>(buffer), bufferLength));
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

        auto const status = controlTransfer(slot, requestType, static_cast<uint8_t>(requestCode), value, index, payload);
        if (bytesTransferred) {
            *bytesTransferred = bufferLength;
        }
        co_return status == Usb::Status::Success ? RESULT::Ok : RESULT::ErrorTransmission;
    }

    std::vector<std::shared_ptr<UsbDevice>> deviceTable_;
    RESULT error_ = RESULT::ErrorGeneral;
};

// Factory function to create XHCI controller
CapabilityRegisters& CreateController(PCIe::Driver& pcie, PCIe::DeviceAddress const& devAddress)
{
    auto config = pcie.ConfigureDevice(devAddress);

    // TODO: To make this more generic, we need to verify the controller model and (RPi4) environment
    // and only do these things when appropriate.
    // Also provide an abstracted interface for platform-specific functionality needed in the driver
    // (DMA memory management and max packet size per speed, that sort of thing).

    if (BootLib::Cpu::IsRpi4())
    {
        LOG_DEBUGln("XHCI    Loading USB firmware...");
        Mailbox::TagMessage<Mailbox::Tag::RPI4_PCIE_XHCI_USB_RESET, 1> resetTag{{ 1u << 20 }};
        if (!Mailbox::SendTags(resetTag)) {
            LOG_DEBUGln("XHCI    ✗ Failed to load USB firmware");
        }
        else
        {
            LOG_DEBUGln("XHCI    New state: {}", resetTag.args[0]);
        }
    }

    auto& common = config.Common();

    common.CacheLineSize = 64 / 4; // ??

    // Display BARs if any and find Bar0
    PCIe::BarInfo bar0{};
    for (auto&& bar : config.EnumerateBars())
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
    for (auto const& cap : config.EnumerateCapabilities())
    {
        //PrintCapability(cap, config.Common());
    }
    
    // Enable BAR 0 at the beginning of PCIe aperture
    LOG_DEBUGln("XHCI    Configuring BAR 0: CPU=0x{:016x} Size = 0x{:X}", bar0.physical_address, bar0.size);
    if (bar0.size == 0)
    {
        LOG_DEBUGln("    BAR 0 size is zero. Halting...");
        Cpu::Halt();
    }
    
    auto bar0Memory = pcie.MapBar(bar0);

    LOG_DEBUGln("    Configured BAR 0: CPU=0x{:016x} Size = 0x{:X}", bar0.physical_address, bar0.size);
    
    // Verify the BAR was written correctly
    //auto const rebar0 = config.GetBar(0);

    //printf("    BAR 0 readback: CPU=0x%016llx\n", rebar0.physical_address);

    common.InterruptPin = 1; // INTA
    common.Command = 0x146; // !IO, Memory, Master, SERR, PARITY

    // Add a delay to ensure the configuration takes effect
    Cpu::Delay(100ms);

    asm volatile("dsb sy" : : : "memory");  // ARM64

    return *reinterpret_cast<CapabilityRegisters*>(bar0Memory.data());
}

Async::task<std::shared_ptr<UsbDriver>> UsbInitializeXhci(PCIe::Driver& pcie, PCIe::DeviceAddress const& deviceAddress)
{
    LOG_DEBUGln("Initializing xHCI USB Driver");
    auto& capabilityRegisters = CreateController(pcie, deviceAddress);
    auto driver = std::make_shared<XhciUsbDriver>(capabilityRegisters);
    co_await driver->Initialize();
    co_return driver;
}

}
// namespace Usb::Xhci
