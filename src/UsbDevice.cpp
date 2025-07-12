// Basic USB device driver for Raspberry Pi 3.
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

#include "UsbDevice.h"

#include "DesignWareUsbHost.h"
#include "DesignWareUsbChannel.h"

#include "UsbSpec.h"

#include "emb-stdio.h"

#define LOG(...)
//#define LOG(...) printf2(__VA_ARGS__)
#define LOG_DEBUG(...)
//#define LOG_DEBUG(...) printf2(__VA_ARGS__)

enum RESULT : int;

RESULT HcdProcessRootHubMessageIn(std::span<std::byte> replyBuffer, UsbDeviceRequest const& request, uint32_t *bytesTransferred);
RESULT HcdProcessRootHubMessageOut(UsbDeviceRequest const& request);

namespace USB
{

extern std::shared_ptr<HCDHost> Host;

constexpr uint8_t RootHubDeviceNumber = 0;

// Sends an OUT control message to a device. Handles all necessary channel creation
// and other processing. The sequence of a control transfer is defined in the
// USB 2.0 manual section 5.5.
// Returns the number of bytes transferred or 0 on failure.
uint32_t Device::SubmitControlMessageOUT(
    std::span<std::byte const> buffer,
    UsbDeviceRequest const& request,
    uint32_t timeoutInUs
)
{
    auto const channel = Host->GetChannel();

    uint32_t bytesTransferred = 0;

    // Note: The root hub is special (fake).
    // It starts as device 0 and gets assigned the address 1 during enumeration.
    // TODO: A bit of inheritance for this.
    if (m_Number <= 1)
    {
        if (HcdProcessRootHubMessageOut(request) == 0)
        {
            LOG("HCD: Root hub message failed.\n");
        }
        return 0;
    }

    LOG_DEBUG("Setup phase\n");
    std::span requestSpan{ reinterpret_cast<std::byte const*>(&request), sizeof(request) };
    auto const setupSent = channel->TransferOut(m_ControlPipe, USB_TRANSFER_TYPE_CONTROL, requestSpan, USB_PID_SETUP);
    if (setupSent != sizeof(request))
    {
        LOG("HCD: SETUP packet to device: %#x req: %#x req Type: %#x Speed: %i PacketSize: %i LowNode: %i LowPort: %i Error: %i\n",
            m_ControlPipe.Number, request.Request, request.Type, m_ControlPipe.Speed, m_ControlPipe.MaxPacketSizeInBytes, m_ControlPipe.splitNodePoint, m_ControlPipe.splitNodePort, result);// Some parameter issue
        return 0;
    }

    if (!buffer.empty())
    {
        LOG_DEBUG("Transfer phase\n");
        bytesTransferred = channel->TransferOut(m_ControlPipe, USB_TRANSFER_TYPE_CONTROL, buffer, USB_PID_DATA1);
        if (bytesTransferred == 0)
        {
            LOG("HCD: Could not transfer DATA to device %i.\n", m_ControlPipe.Number);
            return 0;
        }
    }
    else
    {
        LOG_DEBUG("No data to transfer, skipping transfer phase.\n");
    }

    LOG_DEBUG("Status phase\n");
    channel->TransferIn(m_ControlPipe, USB_TRANSFER_TYPE_CONTROL, {}, USB_PID_DATA1);

    return bytesTransferred;
}

uint32_t Device::SubmitControlMessageIN(
    std::span<std::byte>    buffer,
    UsbDeviceRequest const& request,
    uint32_t                timeoutInUs
)
{
    auto const channel = Host->GetChannel();

    uint32_t bytesTransferred = 0;

    // Note: The root hub is special (fake).
    // It starts as device 0 and gets assigned the address 1 during enumeration.
    // TODO: A bit of inheritance for this.
    if (m_Number <= 1)
    {
        if (HcdProcessRootHubMessageIn(buffer, request, &bytesTransferred) == 0)
        {
            LOG("HCD: Root hub message failed.\n");
        }
        return 0;
    }

    LOG_DEBUG("Setup phase\n");
    std::span requestSpan{ reinterpret_cast<std::byte const*>(&request), sizeof(request) };
    auto const setupSent = channel->TransferOut(m_ControlPipe, USB_TRANSFER_TYPE_CONTROL, requestSpan, USB_PID_SETUP);
    if (setupSent != sizeof(request))
    {
        LOG("HCD: SETUP packet to device: %#x req: %#x req Type: %#x Speed: %i PacketSize: %i LowNode: %i LowPort: %i Error: %i\n",
            m_ControlPipe.Number, request.Request, request.Type, m_ControlPipe.Speed, m_ControlPipe.MaxPacketSizeInBytes, m_ControlPipe.splitNodePoint, m_ControlPipe.splitNodePort, result);// Some parameter issue
        return 0;
    }

    if (!buffer.empty())
    {
        LOG_DEBUG("Transfer phase\n");
        bytesTransferred = channel->TransferIn(m_ControlPipe, USB_TRANSFER_TYPE_CONTROL, buffer, USB_PID_DATA1);
        if (bytesTransferred == 0)
        {
            LOG("HCD: Could not transfer DATA to device %i.\n", m_ControlPipe.Number);
            return 0;
        }
        LOG_DEBUG("Status phase\n");
        channel->TransferOut(m_ControlPipe, USB_TRANSFER_TYPE_CONTROL, {}, USB_PID_DATA1);
    }
    else
    {
        LOG_DEBUG("No data to transfer, skipping transfer phase.\n");
        LOG_DEBUG("Status phase\n");
        channel->TransferIn(m_ControlPipe, USB_TRANSFER_TYPE_CONTROL, {}, USB_PID_DATA1);
    }

    return bytesTransferred;
}

Device::Device(uint32_t number)
    : m_Number(number)
{
    // Initialize the device with the given number.
}

}
// namespace USB
