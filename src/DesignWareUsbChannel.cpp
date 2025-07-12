// DesignWare2 hardware driver for Raspberry Pi 3.
//
// Based on the code from: https://github.com/LdB-ECM/Raspberry-Pi/tree/master/Arm32_64_USB
//
// Original banner:
//    /***************************************************************}
//    {  Complete redux of CSUD (Chadderz's Simple USB Driver) by     }
//    {  Alex Chadwick by Leon de Boer(LdB) 2017, 2018                }
//    {                                                               }
//    {  Version 2.0  (AARCH64 & AARCH32 compilation supported)       }
//    {                                                               }
//    {  CSUD was overly complex in both it's coding and especially   }
//    {  implementation for what it actually did. At it's heart CSUD  }
//    {  simply provides the CONTROL pipe operation of a USB bus.That }
//    {  provides all the functionality to enumerate the USB bus and  }
//    {  control devices on the BUS. It is the start point for a real }
//    {  driver or access layer to the USB.                           }
//    {                                                               }
//    {******************[ THIS CODE IS FREEWARE ]********************}
//    {                                                               }
//    {     This sourcecode is released for the purpose to promote    }
//    {   programming on the Raspberry Pi. You may redistribute it    }
//    {   and/or modify with the following disclaimer.                }
//    {                                                               }
//    {   The SOURCE CODE is distributed "AS IS" WITHOUT WARRANTIES   }
//    {   AS TO PERFORMANCE OF MERCHANTABILITY WHETHER EXPRESSED OR   }
//    {   IMPLIED. Redistributions of source code must retain the     }
//    {   copyright notices.                                          }
//    {                                                               }
//    {++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include "DesignWareUsbChannel.h"

#include "DesignWareUsbHost.h"

#include <stdbool.h>            // C standard needed for bool
#include <stdlib.h>                // C standard needed for NULL
#include <stdint.h>                // C standard needed for uint8_t, uint32_t, uint64_t etc
#include <string.h>                // C standard needed for memset
#include <wchar.h>                // C standard needed for UTF for unicode descriptor support

#include <BootLib/RegisterProxy.h>

#include "Cpu.h"
#include "Mmio.h"
#include "Mailbox.h"
#include "Timer.h"
#include "Processor.h"
#include "Interrupts.h"

#include "emb-stdio.h"                // Needed for printf

#include <concepts>

#define LOG(...)
//#define LOG(...) printf2(__VA_ARGS__)
#define LOG_DEBUG(...)
//#define LOG_DEBUG(...) printf2(__VA_ARGS__)

// Explicitly packing bitfields of different types keeps VSCode's IntelliSense happier.
#define PACKED __attribute__((__packed__))

union HostChannelCharacteristic
{
    struct PACKED
    {
        unsigned max_packet_size    : 11; //  @0-10 Maximum packet size the endpoint is capable of sending or receiving
        unsigned endpoint_number    :  4; // @11-14 Endpoint number (low 4 bits of bEndpointAddress)
        unsigned endpoint_direction :  1; // @15    Endpoint direction 1=IN, 0=OUT
        unsigned _reserved          :  1; // @16
        unsigned low_speed          :  1; // @17    1 when the device being communicated with is at low speed, 0 otherwise
        unsigned endpoint_type      :  2; // @18-19 Endpoint type (low 2 bits of bmAttributes)
        unsigned packets_per_frame  :  2; // @20-21 Maximum number of transactions that can be executed per microframe
        unsigned device_address     :  7; // @22-28 USB device address of the device on which the endpoint is located
        unsigned odd_frame          :  1; // @29    Before enabling channel must be set to opposite of low bit of host_frame_number
        unsigned channel_disable    :  1; // @30    Software can set this to 1 to halt the channel
        unsigned channel_enable     :  1; // @31    Software can set this to 1 to enable the channel
    };
    uint32_t Raw32;
};

static_assert(sizeof(HostChannelCharacteristic) == 0x04, "Structure should be 32bits (4 bytes)");


union HostChannelSplitControl
{
    struct PACKED
    {
        unsigned port_address         :  7; //  @0-6    0-based index of the port on the high-speed hub Transaction Translator occurs
        unsigned hub_address          :  7; // @7-13    USB device address of the high-speed hub that acts as Transaction Translator
        unsigned transaction_position :  2; // @14-15   If we are processing isochronous OUT split the transation position Begin=2,End=1,Middle=0,All=3
        unsigned complete_split       :  1; // @16      1 to complete a Split transaction, 0 = normal transaction
        unsigned _reserved            : 14; // @17-30
        unsigned split_enable         :  1; // @31      Set to 1 to enable Split Transactions
    };
    uint32_t Raw32;
};

static_assert(sizeof(HostChannelSplitControl) == 0x04, "Structure should be 32bits (4 bytes)");


union ChannelInterrupts
{
    struct PACKED
    {
        bool TransferComplete        :  1; //  @0
        bool Halt                    :  1; //  @1
        bool AhbError                :  1; //  @2
        bool Stall                   :  1; //  @3
        bool NegativeAcknowledgement :  1; //  @4
        bool Acknowledgement         :  1; //  @5
        bool NotYet                  :  1; //  @6
        bool TransactionError        :  1; //  @7
        bool BabbleError             :  1; //  @8
        bool FrameOverrun            :  1; //  @9
        bool DataToggleError         :  1; // @10
        bool BufferNotAvailable      :  1; // @11
        bool ExcessiveTransmission   :  1; // @12
        bool FrameListRollover       :  1; // @13
        unsigned _reserved           : 18; // @14-31
    };
    uint32_t Raw32;
};

static_assert(sizeof(ChannelInterrupts) == 0x04, "Structure should be 32bits (4 bytes)");


union HostTransferSize
{
    struct PACKED
    {
        unsigned size         : 19; //  @0-18   Size of data to send or receive, in bytes and can be greater than maximum packet length
        unsigned packet_count : 10; // @19-28   Number of packets left to transmit or maximum number of packets left to receive
        PacketId packet_id    :  2; // @29      Various packet phase ID
        unsigned do_ping      :  1; // @31        
    };
    uint32_t Raw32;
};

static_assert(sizeof(HostTransferSize) == 0x04, "Structure should be 32bits (4 bytes)");


union UsbSendControl
{
    struct PACKED
    {
        unsigned SplitTries        : 8; //  @0 Count of attempts to send packet as a split
        unsigned PacketTries       : 8; //  @8 Count of attempts to send current packet
        unsigned GlobalTries       : 8; // @16 Count of global tries (more serious errors increment)
        unsigned reserved          : 3; // @24 Padding to make 32 bit
        bool     LongerDelay       : 1; // @27 Longer delay .. not yet was response
        bool     ActionResendSplit : 1; // @28 Resend split packet
        bool     ActionRetry       : 1; // @29 Retry sending 
        bool     ActionFatalError  : 1; // @30 Some fatal error occured ... so bail
        bool     Success           : 1; // @31 Success .. tansfer complete
    };
    uint32_t Raw32;
};

static_assert(sizeof(UsbSendControl) == 0x04, "Structure should be 32bits (4 bytes)");


union HCDChannel::Registers
{
    BootLib::Register<HostChannelCharacteristic, 0x00> Characteristic;
    BootLib::Register<HostChannelSplitControl  , 0x04> SplitCtrl     ;
    BootLib::Register<ChannelInterrupts        , 0x08> Interrupt     ;
    BootLib::Register<ChannelInterrupts        , 0x0C> InterruptMask ;
    BootLib::Register<HostTransferSize         , 0x10> TransferSize  ;
    BootLib::Register<uint32_t                 , 0x14> DmaAddr       ;
};


/*-INTERNAL: HCDCheckErrorAndAction -----------------------------------------
 Given a channel interrupt flags and whether packet was complete (not split)
 it will set sendControl structure with what to do next.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT HCDCheckErrorAndAction(ChannelInterrupts interrupts, bool packetSplit, UsbSendControl* sendCtrl) {
    sendCtrl->ActionResendSplit = false;
    sendCtrl->ActionRetry = false;
    sendCtrl->LongerDelay = false;
    // First deal with all the fatal errors .. no use dealing with trivial errors if these are set
    if (interrupts.AhbError) {
        sendCtrl->ActionFatalError = true;
        return DWCRESULT::ErrorDevice;
    }
    if (interrupts.DataToggleError) {
        sendCtrl->ActionFatalError = true;
        return DWCRESULT::ErrorTransmission;
    }
    // Next deal with the fully successful case
    if (interrupts.Acknowledgement) {
        if (interrupts.TransferComplete) sendCtrl->Success = true;
            else sendCtrl->ActionResendSplit = true;
        sendCtrl->GlobalTries = 0;
        return DWCRESULT::Ok;
    }
    // Everything else is minor error invoking a retry .. so first update counts
    if (packetSplit) {
        sendCtrl->SplitTries++;
        if (sendCtrl->SplitTries == 5) {
            // Ridiculous number of split resends reached .. fatal error
            sendCtrl->ActionFatalError = true;
            return DWCRESULT::ErrorTransmission;
        }
        sendCtrl->ActionResendSplit = true;
    } else {
        sendCtrl->PacketTries++;
        if (sendCtrl->PacketTries == 3) {
            // Ridiculous number of packet resends reached .. fatal error
            sendCtrl->ActionFatalError = true;
            return DWCRESULT::ErrorTransmission;
        }
        sendCtrl->ActionRetry = true;
    }
    // Check no transmission errors and if so deal with minor cases
    if (!interrupts.Stall && !interrupts.BabbleError &&
        !interrupts.FrameOverrun) {
        // If endpoint NAK nothing wrong just demanding a retry
        if (interrupts.NegativeAcknowledgement)
            return DWCRESULT::ErrorTransmission;
        if (interrupts.NotYet)
        {
            // Device is not yet ready for this
            // Note: This seems to be exactly what we need to do.
            // But the caller was putting on a 10 ms wait? :-P
            //sendCtrl->LongerDelay = true;
            return DWCRESULT::ErrorTransmission;
        }
        return DWCRESULT::ErrorTimeout;
    }
    // Everything else updates global count as it is serious
    sendCtrl->GlobalTries++;
    if (sendCtrl->GlobalTries == 3) {
        sendCtrl->ActionRetry = false;
        sendCtrl->ActionResendSplit = false;
        sendCtrl->ActionFatalError = true;
        return DWCRESULT::ErrorTransmission;
    }
    // Stall is usually recoverable with a wait and retry.
    if (interrupts.Stall)
    {
        return DWCRESULT::ErrorStall;
    }
    // Deal with true transmission errors
    if ((interrupts.BabbleError) ||
        (interrupts.FrameOverrun) ||
        (interrupts.TransactionError))
    {
        return DWCRESULT::ErrorTransmission;
    }
    return DWCRESULT::ErrorGeneral;
}

/*-INTERNAL: HCDWaitOnTransmissionResult------------------------------------
 When not using Interrupts, Timers or OS this is the good old polling wait
 around for transmission packet sucess or timeout. HCD supports multiple
 options on sending the packets this static polled is just one way.
 19Feb17 LdB
 --------------------------------------------------------------------------*/
ChannelInterrupts HCDChannel::WaitOnTransmissionResult(uint32_t timeout)
{
    uint64_t ticksTimeout = Cpu::GetPerformanceTicksForUs(timeout);
    uint64_t original_tick = Cpu::GetPerformanceCounter();
    for (;;) {
        Cpu::DelayInMicroseconds(100);
        ChannelInterrupts tempInt = registers.Interrupt;
        if (tempInt.Halt || Cpu::GetPerformanceCounter() - original_tick > ticksTimeout)
        {
            return tempInt;
        }
    }
}

void HCDChannel::Prepare(
    UsbPipe const&    pipe, // Endpoint information
    usb_transfer_type Type,
    UsbDirection      Direction,
    PacketId          packetId,
    uint32_t          transferSize,
    InCallback        callback, // Callback to call when transfer is complete
    uintptr_t         context
)
{
    LOG_DEBUG("HCD: Channel %u %s transfer, length %d, packetId %d, address %u, endpoint %u, type %u, speed %u\n",
        channel, Direction == USB_DIRECTION_IN ? "in" : "out", pipe.Number, pipe.EndPoint, Type, pipe.Speed);

    uint32_t offset = 0;

    // Program the channel.
    registers.Interrupt = 0xFFFFFFFF;
    registers.InterruptMask = 0x0;

    HostChannelCharacteristic tempChar = { 0 };
    tempChar.device_address = pipe.Number;
    tempChar.endpoint_number = pipe.EndPoint;
    tempChar.endpoint_direction = Direction;
    tempChar.low_speed = pipe.Speed == USB_SPEED_LOW ? true : false;
    tempChar.endpoint_type = Type;
    tempChar.max_packet_size = pipe.MaxPacketSizeInBits;
    tempChar.channel_enable = false;
    tempChar.channel_disable = false;
    registers.Characteristic = tempChar;

    // Clear and setup split control to low speed devices
    HostChannelSplitControl tempSplit = { 0 };
    if (pipe.Speed != USB_SPEED_HIGH) {
        LOG_DEBUG("Setting split control, addr: %i port: %i, packetSize: PacketSize: %u\n",
            pipe.splitNodePoint, pipe.splitNodePort, pipe.MaxPacketSizeInBits);
        tempSplit.split_enable = true;
        tempSplit.hub_address = pipe.splitNodePoint;
        tempSplit.port_address = pipe.splitNodePort;
        tempSplit.transaction_position = 0;//3;
    }
    registers.SplitCtrl = tempSplit;

    // Set transfer size
    HostTransferSize tempXfer{};
    tempXfer.size = transferSize;
    if (pipe.Speed == USB_SPEED_LOW) tempXfer.packet_count = (transferSize + 7) / 8;
    else                             tempXfer.packet_count = (transferSize + pipe.MaxPacketSizeInBits - 1) / pipe.MaxPacketSizeInBits;
    if (tempXfer.packet_count == 0) tempXfer.packet_count = 1;
    tempXfer.packet_id = packetId;
    registers.TransferSize = tempXfer;

    m_Prepared = true; // Mark channel as prepared
    m_InTransfer = false;
    m_OutTransfer = false;
    m_SplitEnabled = (pipe.Speed != USB_SPEED_HIGH);
    m_Size = transferSize;
    m_Pipe = pipe;
    m_Type = Type;
    m_Direction = Direction;
    m_Callback = callback;
    m_Context = context;
}

void HCDChannel::StartInTransfer()
{
    m_InTransfer = true;

    LOG_DEBUG("HCD: Channel %u transfer size set to %#08X bytes.\n", pipectrl.Channel, tempXfer.Raw32);

    // Clear any left over channel interrupts
    registers.Interrupt = 0xFFFFFFFF;
    registers.InterruptMask = ChannelInterrupts
    {
        .TransferComplete        = true,
        .Halt                    = true,
        .Stall                   = true,
        .NegativeAcknowledgement = true,
    };

    // TODO: DWC_HOST->INTERRUPTMASK = 1u << channel; // Enable channel interrupts

    // Clear any left over split
    registers.SplitCtrl = [](auto& reg){ reg.complete_split = false; };

    uint8_t* dmaBuffer  = Mailbox::AsGpuPointer(m_DmaBuffer);

    registers.DmaAddr = Mailbox::AsGpuAddress(dmaBuffer) | 0xC000'0000u;

    auto nextFrame = m_Host.GetCurrentFrame() + 1;

    /* Launch transmission */
    registers.Characteristic = [nextFrame](auto& reg)
    {
        reg.channel_enable    = true;
        reg.odd_frame         = nextFrame & 1;
        reg.packets_per_frame = 1;
    };
}

void HCDChannel::HandleInTransferInterrupt()
{
    ChannelInterrupts interrupts = registers.Interrupt;
    if (interrupts.TransferComplete)
    {
        HostTransferSize size = registers.TransferSize;
        if (size.packet_count > 0)
        {

            LOG_DEBUG("HCD: Channel %u transfer size is zero, no data transferred.\n", channel);
            return;
        }

        LOG_DEBUG("HCD: Channel %u transfer complete.\n", channel);
        if (m_Callback)
        {
            if (m_Callback(m_Context, *this))
            {
                LOG_DEBUG("HCD: Callback for channel %u returned true.\n", channel);
            }
            else
            {
                LOG_DEBUG("HCD: Callback for channel %u returned false.\n", channel);
            }
        }
    }
    else if (interrupts.Stall)
    {
        // Must retry later.
        LOG_DEBUG("HCD: Channel %u stalled.\n", channel);
    }
    else if (interrupts.NegativeAcknowledgement)
    {
        // Rejected by the device.
        LOG_DEBUG("HCD: Channel %u NAKed.\n", channel);
    }
    else if (interrupts.Halt)
    {
        if (m_SplitEnabled)
        {
            registers.SplitCtrl = [](auto& reg)
            {
                reg.complete_split = true; // Mark split as complete
            };
        }
        else
        {
            LOG_DEBUG("HCD: Channel %u halted.\n", channel);
        }
    }
    else
    {
        LOG_DEBUG("HCD: Channel %u unknown interrupt.\n", channel);
    }
}

/*-INTERNAL: HCDChannelTransfer----------------------------------------------
 Sends/recieves data from the given buffer and size directed by pipe settings.
 19Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT HCDChannel::Transfer(UsbPipe const& pipe, usb_transfer_type Type, UsbDirection Direction, uint8_t* buffer, uint32_t& bufferLength, PacketId packetId) 
{
    LOG_DEBUG("HCD: Channel %u %s transfer, length %d, packetId %d, address %u, endpoint %u, type %u, speed %u%s",
        channel.GetChannelNumber(), Direction == USB_DIRECTION_IN ? "in" : "out", bufferLength, packetId, pipe.Number, pipe.EndPoint, Type, pipe.Speed,
        Direction == USB_DIRECTION_IN ? "\n" : ", "
    );
    if (bufferLength >= 8 && Direction == USB_DIRECTION_OUT)
    {
        LOG_DEBUG("Data = 0x%08X'%08X\n", ((uint32_t*)buffer)[1], ((uint32_t*)buffer)[0]);
    }

    DWCRESULT result;
    ChannelInterrupts tempInt;
    UsbSendControl sendCtrl = { 0 };
    uint32_t offset = 0;

    // Program the channel.
    registers.Interrupt = 0xFFFFFFFF;
    registers.InterruptMask = 0x0;

    HostChannelCharacteristic tempChar = { 0 };
    tempChar.device_address = pipe.Number;
    tempChar.endpoint_number = pipe.EndPoint;
    tempChar.endpoint_direction = Direction;
    tempChar.low_speed = pipe.Speed == USB_SPEED_LOW ? true : false;
    tempChar.endpoint_type = Type;
    tempChar.max_packet_size = pipe.MaxPacketSizeInBits;
    tempChar.channel_enable = false;
    tempChar.channel_disable = false;
    registers.Characteristic = tempChar;

    // Clear and setup split control to low speed devices
    HostChannelSplitControl tempSplit = { 0 };
    if (pipe.Speed != USB_SPEED_HIGH) {
        LOG_DEBUG("Setting split control, addr: %i port: %i, packetSize: PacketSize: %u\n",
            pipe.splitNodePoint, pipe.splitNodePort, pipe.MaxPacketSizeInBits);
        tempSplit.split_enable = true;
        tempSplit.hub_address = pipe.splitNodePoint;
        tempSplit.port_address = pipe.splitNodePort;
        tempSplit.transaction_position = 0;//3;
    }
    registers.SplitCtrl = tempSplit;

    // Set transfer size
    HostTransferSize tempXfer = { 0 };
    tempXfer.size = bufferLength;
    if (pipe.Speed == USB_SPEED_LOW) tempXfer.packet_count = (bufferLength + 7) / 8;
    else tempXfer.packet_count = (bufferLength + pipe.MaxPacketSizeInBits - 1) / pipe.MaxPacketSizeInBits;
    if (tempXfer.packet_count == 0) tempXfer.packet_count = 1;
    tempXfer.packet_id = packetId;
    registers.TransferSize = tempXfer;

    LOG_DEBUG("HCD: Channel %u transfer size set to %#08X bytes.\n", channel.GetNumber(), tempXfer.Raw32);

    sendCtrl.PacketTries = 0;
    do {

        // Clear any left over channel interrupts
        registers.Interrupt = 0xFFFFFFFF;
        registers.InterruptMask = 0x0;

        // Clear any left over split
        tempSplit = *registers.SplitCtrl;
        tempSplit.complete_split = false;
        registers.SplitCtrl = tempSplit;

        uint8_t* dmaBuffer  = Mailbox::AsGpuPointer(m_DmaBuffer);

        // Since our buffer is unaligned for OUT endpoints, copy the data
        // From the buffer to the aligned buffer
        if (Direction == USB_DIRECTION_OUT)
        {
            for (int i = 0; i < bufferLength-offset; ++i)
            {
                dmaBuffer[i] = buffer[offset + i];
            }
            //memcpy(&m_DmaBuffer, &buffer[offset], bufferLength-offset);
        }

        //Processor::FlushDataCache(dmaBuffer, bufferLength - offset);

        registers.DmaAddr = Mailbox::AsGpuAddress(dmaBuffer) | 0xC000'0000u;

        auto nextFrame = m_Host.GetCurrentFrame() + 1;

        /* Launch transmission */
        tempChar = *registers.Characteristic;// Read host channel characteristic
        tempChar.odd_frame = nextFrame & 1;
        tempChar.packets_per_frame = 1;                                // Set 1 frame per packet
        tempChar.channel_enable = true;                                // Set enable channel
        tempChar.channel_disable = false;                            // Clear channel disable
        registers.Characteristic = tempChar;// Write channel characteristic

        // Polling wait on transmission only option right now .. other options soon :-)
        tempInt = WaitOnTransmissionResult(5000);
        if (!tempInt.Halt)
        {
            LOG("HCD: Request on channel %i has timed out.\n", m_Number);// Log the error
            return DWCRESULT::ErrorTimeout;                                    // Return timeout error
        }
        LOG_DEBUG("HCD: Channel %u transmission result: 0x%08X\n", m_Number, tempInt.Raw32);

        tempSplit = *registers.SplitCtrl;    // Fetch the split details
        result = HCDCheckErrorAndAction(tempInt,
            tempSplit.split_enable, &sendCtrl);                        // Check transmisson DWCRESULT and set action flags
        if (result != DWCRESULT::Ok) LOG_DEBUG("Result: %i Action: 0x%08x tempInt: 0x%08x tempSplit: 0x%08x Bytes sent: %i\n",
            result, (unsigned int)sendCtrl.Raw32, (unsigned int)tempInt.Raw32, 
            (unsigned int)tempSplit.Raw32, result != DWCRESULT::Ok ? 0 : (*registers.TransferSize).size);
        if (sendCtrl.ActionFatalError) return result;                // Fatal error occured we need to bail

        sendCtrl.SplitTries = 0;
        while (sendCtrl.ActionResendSplit) {                        // Decision was made to resend split
            Cpu::DelayInMicroseconds(250);
            // Clear channel interrupts
            registers.Interrupt = 0xFFFFFFFF;
            registers.InterruptMask = 0x0;

            // Set we are completing the split
            tempSplit = *registers.SplitCtrl;
            tempSplit.complete_split = true;                        // Set complete split flag
            registers.SplitCtrl = tempSplit;

            // Launch transmission
            tempChar = *registers.Characteristic;
            tempChar.channel_enable = true;
            tempChar.channel_disable = false;
            registers.Characteristic = tempChar;

            // Polling wait on transmission only option right now .. other options soon :-)
            tempInt = WaitOnTransmissionResult(5000);
            if (!tempInt.Halt)
            {
                LOG("HCD: Request split completion on channel:%i has timed out.\n", m_Number);// Log error
                return DWCRESULT::ErrorTimeout;                                // Return timeout error
            }
            LOG_DEBUG("HCD: Channel %u transmission result: 0x%08X\n", m_Number, tempInt.Raw32);

            tempSplit = *registers.SplitCtrl;// Fetch the split details again
            result = HCDCheckErrorAndAction(tempInt,
                tempSplit.split_enable, &sendCtrl);                    // Check DWCRESULT of split resend and set action flags
            LOG_DEBUG("Result: %i Action: 0x%08x tempInt: 0x%08x tempSplit: 0x%08x Bytes sent: %i\n",
                result, (unsigned int)sendCtrl.Raw32, (unsigned int)tempInt.Raw32,
                (unsigned int)tempSplit.Raw32, result != DWCRESULT::Ok ? 0 : (*registers.TransferSize).size);
            if (sendCtrl.ActionFatalError) return result;            // Fatal error occured bail
            if (sendCtrl.LongerDelay) Cpu::DelayInMicroseconds(10000);            // Not yet response slower delay
                else Cpu::DelayInMicroseconds(2500);                                // Small delay between split resends
        }

        if (sendCtrl.Success) {                                        // Send successful adjust buffer position
            // BUGBUG: In an out transfer, this_transfer doesn't mean what we think it means.
            uint32_t const this_transfer = (*registers.TransferSize).size;
            uint32_t const transferred = bufferLength - this_transfer - offset;
            LOG_DEBUG("Transferred %u bytes on channel %u. Remaining: %u\n", transferred, m_Number, this_transfer);

            // Since our buffer is unaligned for IN endpoints
            // Copy the data from the the aligned buffer to the buffer
            // We know the aligned buffer was used because it is unaligned
            if (Direction == USB_DIRECTION_IN)
            {
                //Processor::InvalidateDataCache(dmaBuffer, transferred);
                for (int i = 0; i < transferred; ++i)
                {
                    buffer[offset + i] = dmaBuffer[i];
                }
                //memcpy(&buffer[offset], m_DmaBuffer, this_transfer);
                if (transferred >= 8)
                {
                    LOG_DEBUG("Data = 0x%08X'%08X at %8p\n", ((uint32_t*)&buffer[offset])[1], ((uint32_t*)&buffer[offset])[0], &buffer[offset]);
                    LOG_DEBUG("Data = 0x%08X'%08X at %8p\n", ((uint32_t*)dmaBuffer)[1], ((uint32_t*)dmaBuffer)[0], dmaBuffer);
                }
            }

            offset += transferred;
        }

    } // Loop if packets remain.
    while ((*registers.TransferSize).packet_count > 0);

    if (Direction == USB_DIRECTION_IN)
    {
        bufferLength -= (*registers.TransferSize).size;
    }

    return DWCRESULT::Ok;
}

HCDChannel::HCDChannel(HCDHost& host, uintptr_t baseAddress, uint8_t channelNumber)
    : registers   { *reinterpret_cast<Registers*>(baseAddress) }
    , m_Host        { host                      }
    , m_Number      { channelNumber             }
    , m_Prepared    { false                     }
    , m_InTransfer  { false                     }
    , m_OutTransfer { false                     }
    , m_SplitEnabled{ false                     }
    , m_Size        { 0                         }
    , m_Pipe        {                           }
    , m_Type        { USB_TRANSFER_TYPE_CONTROL }
    , m_Direction   { USB_DIRECTION_OUT         }
    , m_Callback    { nullptr                   }
    , m_Context     { 0                         }
{
    registers.Characteristic = HostChannelCharacteristic
    {
        .device_address     = 0,
        .endpoint_number    = 0,
        .endpoint_direction = 0,
        .low_speed          = false,
        .endpoint_type      = USB_TRANSFER_TYPE_CONTROL,
        .max_packet_size    = 0,
        .channel_enable     = false,
        .channel_disable    = false
    };
}
