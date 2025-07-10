#pragma once

#include "Cpu.h"

#include <stdint.h>
#include <stddef.h>

#include <span>

extern uintptr_t GpuMemBase;

namespace Mailbox
{

template < typename T >
inline T* AsGpuPointer(T* ptr)
{
    return reinterpret_cast<T*>(reinterpret_cast<uintptr_t>(ptr) | GpuMemBase);
}

template < typename T >
inline uint32_t AsGpuAddress(T* ptr)
{
    return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(ptr) | GpuMemBase);
}

template < typename T >
inline T* AsArmPointer(T* ptr)
{
    return reinterpret_cast<T*>(reinterpret_cast<uintptr_t>(ptr) & ~GpuMemBase);
}

void Send(uint8_t ch, uint32_t             data);
void Send(uint8_t ch, void const volatile* data);

bool SendTags(std::span<uint32_t volatile> data);

enum class Tag : uint32_t;

template < Tag T, uint32_t argCount >
struct TagMessage
{
    static constexpr Tag 		tag = T;
    static constexpr uint32_t 	ArgCount = argCount;

    uint32_t args[ArgCount];
};

template < typename T >
concept TagMessageType = requires(T t)
{
    { T::tag } -> std::convertible_to<Tag>;
    { T::ArgCount } -> std::convertible_to<size_t>;
    { t.args } -> std::convertible_to<std::span<uint32_t>>;
};

struct MailboxMessage
{
    static constexpr uint32_t BufferSize = 256;
    alignas(64) uint32_t Buffer[BufferSize];
    uint32_t Size 	 = 2;
    uint32_t ReadPos = 2;

    uint32_t volatile* GpuBuffer() { return AsGpuPointer(Buffer); }

    MailboxMessage()
    {
        GpuBuffer()[1] = 0; // Status.
    }

    void AddTag(TagMessageType auto& message)
    {
        if (Size + message.ArgCount + 3 > BufferSize)
        {
            // Buffer overflow, halt the processor.
            // Uart::Puts("Oh, noes! Mailbox buffer overflow.\n");
            Cpu::Panic("Mailbox buffer overflow");
        }

        GpuBuffer()[Size++] = static_cast<uint32_t>(message.tag);
        GpuBuffer()[Size++] = message.ArgCount * 4;
        GpuBuffer()[Size++] = 0;
        for (auto arg : message.args)
        {
            GpuBuffer()[Size++] = arg;
        }
    }

    bool SendMessage()
    {
        if (Size <= 2)
        {
            // Uart::Puts("Oh, noes! Mailbox message is empty.\n");
            Cpu::Panic("Mailbox message is empty");
        }

        GpuBuffer()[Size++] = 0; // End tag.
        GpuBuffer()[0] = Size * 4; // Size in bytes.
        return SendTags(std::span<uint32_t volatile>(GpuBuffer(), Size));
    }

    void ReadTag(TagMessageType auto& message)
    {
        if (ReadPos + message.ArgCount + 3 > Size)
        {
            // Buffer overflow, halt the processor.
            // Uart::Puts("Oh, noes! Mailbox buffer overflow.\n");
            Cpu::Panic("Mailbox buffer overflow");
        }

        ReadPos += 3;
        for (auto& arg : message.args)
        {
            arg = GpuBuffer()[ReadPos++];
        }
    }
};

inline bool SendTags(TagMessageType auto&... message)
{
    MailboxMessage mailboxMessage;
    (mailboxMessage.AddTag(message), ...);
    if (!mailboxMessage.SendMessage())
    {
        return false;
    }
    (mailboxMessage.ReadTag(message), ...);
    return true;
}

enum class Tag : uint32_t
{
    /* Videocore info commands */
    GET_VERSION					= 0x00000001,			// Get firmware revision

    /* Hardware info commands */
    GET_BOARD_MODEL				= 0x00010001,			// Get board model
    GET_BOARD_REVISION			= 0x00010002,			// Get board revision
    GET_BOARD_MAC_ADDRESS		= 0x00010003,			// Get board MAC address
    GET_BOARD_SERIAL			= 0x00010004,			// Get board serial
    GET_ARM_MEMORY				= 0x00010005,			// Get ARM memory
    GET_VC_MEMORY				= 0x00010006,			// Get VC memory
    GET_CLOCKS					= 0x00010007,			// Get clocks

    /* Power commands */
    GET_POWER_STATE				= 0x00020001,			// Get power state
    GET_TIMING					= 0x00020002,			// Get timing
    SET_POWER_STATE				= 0x00028001,			// Set power state

    /* GPIO commands */
    GET_GET_GPIO_STATE			= 0x00030041,			// Get GPIO state
    SET_GPIO_STATE				= 0x00038041,			// Set GPIO state

    /* Clock commands */
    GET_CLOCK_STATE				= 0x00030001,			// Get clock state
    GET_CLOCK_RATE				= 0x00030002,			// Get clock rate
    GET_MAX_CLOCK_RATE			= 0x00030004,			// Get max clock rate
    GET_MIN_CLOCK_RATE			= 0x00030007,			// Get min clock rate
    GET_TURBO					= 0x00030009,			// Get turbo

    SET_CLOCK_STATE				= 0x00038001,			// Set clock state
    SET_CLOCK_RATE				= 0x00038002,			// Set clock rate
    SET_TURBO					= 0x00038009,			// Set turbo

    /* Voltage commands */
    GET_VOLTAGE					= 0x00030003,			// Get voltage
    GET_MAX_VOLTAGE				= 0x00030005,			// Get max voltage
    GET_MIN_VOLTAGE				= 0x00030008,			// Get min voltage

    SET_VOLTAGE					= 0x00038003,			// Set voltage

    /* Temperature commands */
    GET_TEMPERATURE				= 0x00030006,			// Get temperature
    GET_MAX_TEMPERATURE			= 0x0003000A,			// Get max temperature

    /* Memory commands */
    ALLOCATE_MEMORY				= 0x0003000C,			// Allocate Memory
    LOCK_MEMORY					= 0x0003000D,			// Lock memory
    UNLOCK_MEMORY				= 0x0003000E,			// Unlock memory
    RELEASE_MEMORY				= 0x0003000F,			// Release Memory
                                                                    
    /* Execute code commands */
    EXECUTE_CODE				= 0x00030010,			// Execute code

    /* QPU control commands */
    EXECUTE_QPU					= 0x00030011,			// Execute code on QPU
    ENABLE_QPU					= 0x00030012,			// QPU enable

    /* Displaymax commands */
    GET_DISPMANX_HANDLE			= 0x00030014,			// Get displaymax handle
    GET_EDID_BLOCK				= 0x00030020,			// Get HDMI EDID block

    GET_GPIO_PIN	            = 0x00030041,			// Get GPIO pin state
    SET_GPIO_PIN	            = 0x00038041,			// Set GPIO pin state

    /* SD Card commands */
    GET_SDHOST_CLOCK	        = 0x00030042,			// Get SD Card EMCC clock
    SET_SDHOST_CLOCK	        = 0x00038042,			// Set SD Card EMCC clock

    /* Framebuffer commands */
    ALLOCATE_FRAMEBUFFER		= 0x00040001,			// Allocate Framebuffer address
    BLANK_SCREEN				= 0x00040002,			// Blank screen
    GET_PHYSICAL_WIDTH_HEIGHT	= 0x00040003,			// Get physical screen width/height
    GET_VIRTUAL_WIDTH_HEIGHT	= 0x00040004,			// Get virtual screen width/height
    GET_COLOUR_DEPTH			= 0x00040005,			// Get screen colour depth
    GET_PIXEL_ORDER				= 0x00040006,			// Get screen pixel order
    GET_ALPHA_MODE				= 0x00040007,			// Get screen alpha mode
    GET_PITCH					= 0x00040008,			// Get screen line to line pitch
    GET_VIRTUAL_OFFSET			= 0x00040009,			// Get screen virtual offset
    GET_OVERSCAN				= 0x0004000A,			// Get screen overscan value
    GET_PALETTE					= 0x0004000B,			// Get screen palette

    RELEASE_FRAMEBUFFER			= 0x00048001,			// Release Framebuffer address
    SET_PHYSICAL_WIDTH_HEIGHT	= 0x00048003,			// Set physical screen width/heigh
    SET_VIRTUAL_WIDTH_HEIGHT	= 0x00048004,			// Set virtual screen width/height
    SET_COLOUR_DEPTH			= 0x00048005,			// Set screen colour depth
    SET_PIXEL_ORDER				= 0x00048006,			// Set screen pixel order
    SET_ALPHA_MODE				= 0x00048007,			// Set screen alpha mode
    SET_VIRTUAL_OFFSET			= 0x00048009,			// Set screen virtual offset
    SET_OVERSCAN				= 0x0004800A,			// Set screen overscan value
    SET_PALETTE					= 0x0004800B,			// Set screen palette
    SET_VSYNC					= 0x0004800E,			// Set screen VSync
    SET_BACKLIGHT				= 0x0004800F,			// Set screen backlight

    /* VCHIQ commands */
    VCHIQ_INIT					= 0x00048010,			// Enable VCHIQ

    /* Config commands */
    GET_COMMAND_LINE			= 0x00050001,			// Get command line 

    /* Shared resource management commands */
    GET_DMA_CHANNELS			= 0x00060001,			// Get DMA channels

    /* Cursor commands */
    SET_CURSOR_INFO				= 0x00008010,			// Set cursor info
    SET_CURSOR_STATE			= 0x00008011,			// Set cursor state
};

}
// namespace Mailbox
