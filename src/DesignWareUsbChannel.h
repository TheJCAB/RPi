#pragma once

#include "DesignWareUsb.h"

#include <stdint.h>
#include <stddef.h>

#include <memory>
#include <span>

union ChannelInterrupts;

class HCDHost;

class HCDChannel
{
public:
    // Maximum packet size of any USB endpoint.
    // 1024 is the maximum allowed by USB 2.0.
    // Most endpoints will provide maximum packet sizes much smaller than this.
    static constexpr uint32_t MaxPacketSize = 1024;

    HCDChannel(HCDHost&, uintptr_t baseAddress, uint8_t channelNumber, std::span<std::byte, 1024> dmaBuffer);

    void Reset();
    void SetMaxPacketSize(uint16_t size);
    void SetDeviceAddress(uint8_t address);
    void SetEndpointType(uint8_t type);
    void SetDirection(bool isIn);
    void SetInterval(uint8_t interval);
    
    bool StartTransfer(void const* data, size_t length);
    bool IsTransferComplete() const;
    size_t GetTransferredBytes() const;

    using InCallback = bool (*)(uintptr_t context, HCDChannel& channel);

    void Prepare(
        UsbPipe const&    pipe, // Endpoint information
        usb_transfer_type Type,
        UsbDirection      Direction,
        PacketId          packetId,
        uint32_t          transferSize,
        InCallback        callback, // Callback to call when transfer is complete
        uintptr_t         context
    );
    void StartInTransfer();

    void HandleInTransferInterrupt();

    uint32_t TransferIn (UsbPipe const& pipe, usb_transfer_type Type, std::span<std::byte      > buffer, PacketId packetId);
    uint32_t TransferOut(UsbPipe const& pipe, usb_transfer_type Type, std::span<std::byte const> buffer, PacketId packetId);

    uint32_t GetNumber() const noexcept { return m_Number; }

    HCDHost& GetHost() const noexcept { return m_Host; }

private:
    ChannelInterrupts WaitOnTransmissionResult(uint32_t timeout);

private:
    union Registers;

    Registers& registers;

    HCDHost& m_Host;

    uint32_t          m_Number;
    bool              m_Prepared;
    bool              m_InTransfer;
    bool              m_OutTransfer;
    bool              m_SplitEnabled;
    uint32_t          m_Size;
    UsbPipe           m_Pipe;
    usb_transfer_type m_Type;
    UsbDirection      m_Direction;
    InCallback        m_Callback;
    uintptr_t         m_Context;

    // Aligned buffer for DMA which need to also be multiple of 4 bytes
    // Fortunately max packet size under USB2 is 1024 so that is a given
    // Aligning to cache line size so we can flush/invalidate with impunity.
    std::span<std::byte, 1024> m_DmaBuffer;
};
