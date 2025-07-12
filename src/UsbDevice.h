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
#pragma once

#include "UsbSpec.h"

#include <span>

#include <stdint.h>

namespace USB
{

class Device
{
public:
    Device(uint32_t number);

    uint32_t GetNumber() const { return m_Number; }

    // Driver-constructed description.
    const char* GetDescription() const;

    // get the device's descriptor.
    DeviceDescriptor GetDescriptor() const;

    size_t GetProductString     (std::span<char> buffer) const;
    size_t GetManufacturerString(std::span<char> buffer) const;
    size_t GetSerialNumberString(std::span<char> buffer) const;
    size_t GetConfigStringString(std::span<char> buffer) const;

    bool IsHub        () const;
    bool IsHid        () const;
    bool IsMassStorage() const;
    bool IsMouse      () const;
    bool IsKeyboard   () const;

    void* GetTypedDevice() const; // TODO: Remove.

    UsbInterfaceDescriptor GetInterfaceDescriptor(uint8_t interfaceIndex);
    UsbEndpointDescriptor  FindEndpoint          (uint8_t interfaceIndex, usb_transfer_type type, UsbDirection direction);

    bool GetDescriptor (
        usb_descriptor_type type,             // The type of descriptor
        uint8_t             index,            // The index of the type descriptor
        uint16_t            langId,           // The language id
        void*               buffer,           // Buffer to recieve descriptor
        uint32_t            length,           // Maximumlength of descriptor
        uint8_t             recipient,        // Recipient flags                                     
        uint32_t*           bytesTransferred, // Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)                                
        bool                runHeaderCheck    // Whether to run header check
    );

    void SetAddress(HCDChannel& channel, uint8_t address);

    uint32_t SubmitControlMessageOUT(
        std::span<std::byte const> buffer,    // Buffer of data to send.
        UsbDeviceRequest const&    request,
        uint32_t                   timeoutInUs
    );

    uint32_t SubmitControlMessageIN(
        std::span<std::byte>    buffer,          // Buffer to receive the data.
        UsbDeviceRequest const& request,
        uint32_t                timeoutInUs
    );

    // Sends/recieves data from/to the given buffer to/from the given endpoint.
    size_t EndpointTransfer(UsbEndpointDescriptor endpoint, std::span<std::byte> buffer);

private:
    uint32_t m_Number; // The device number (address) on the USB bus.

    UsbPipe m_ControlPipe;

    //DeviceDescriptor Descriptor; // The device descriptor.
    //UsbInterfaceDescriptor InterfaceDescriptors[16]; // Up to 16 interfaces per device.
    //UsbEndpointDescriptor Endpoints[16]; // Up to 16 endpoints per interface.
};

}
// namespace USB
