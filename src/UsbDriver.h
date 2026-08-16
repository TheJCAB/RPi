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
#include "UsbPipe.h"
#include "Async.h"

namespace PCIe {
    struct Bcm2711Driver;
    struct DeviceAddress;
}

#include <generator>
#include <memory>
#include <span>
#include <vector>

#include <stdint.h>
#include <wchar.h>


enum class RESULT : int
{
    Ok                  =   0,
    ErrorGeneral        =  -1,
    ErrorArgument       =  -2,
    ErrorRetry          =  -3,
    ErrorDevice         =  -4,
    ErrorIncompatible   =  -5,
    ErrorMemory         =  -7,
    ErrorTimeout        =  -8,
    ErrorHardware       =  -9,
    ErrorTransmission   = -10,
    ErrorDisconnected   = -11,
    ErrorDeviceNumber   = -12,
    ErrorTooManyRetries = -13,
    ErrorIndex          = -14,
    ErrorNotHID         = -15,
    ErrorStall          = -16,
};

class UsbDriver;

struct HubDevice;
struct HidDevice;
struct MassStorageDevice;

enum class PayLoadType {
    Error       = 0,
    None        = 1,
    Hub         = 2,
    Hid         = 3,
    MassStorage = 4,
};

struct __attribute__((__packed__)) UsbParent {
    unsigned Number : 8;
    unsigned PortNumber : 8;
    unsigned reserved : 16;
};

struct __attribute__((__packed__)) UsbConfigControl {
    uint8_t ConfigIndex;
    uint8_t ConfigStringIndex;
    UsbDeviceStatus Status;
    uint8_t reserved;
};

class UsbDevice;

// USB hub structure which is just extra data attached to a USB node
struct HubDevice {
    std::vector<UsbDevice*> Children;
    HubDescriptor Descriptor;
};

class UsbDevice
{
public:
    explicit UsbDevice(std::shared_ptr<UsbDriver> driver = {})
        : driver_(std::move(driver))
    {
    }

    UsbDriver const& GetDriver() const { return *driver_; }
    UsbDriver&       GetDriver()       { return *driver_; }

    std::shared_ptr<UsbDriver> const& GetSharedDriver() const { return driver_; }

    bool IsHub        () const { return PayLoadId == PayLoadType::Hub; }
    bool IsHid        () const { return PayLoadId == PayLoadType::Hid; }
    bool IsMassStorage() const { return PayLoadId == PayLoadType::MassStorage; }
    bool IsKeyboard   () const { return PayLoadId == PayLoadType::Hid && !Interfaces.empty() && Interfaces[0].Protocol == 1; }
    bool IsMouse      () const { return PayLoadId == PayLoadType::Hid && !Interfaces.empty() && Interfaces[0].Protocol == 2; }
    // TODO: These:
    //bool IsGamepad    () const { return false; }

    size_t GetDeviceProductString     (std::span<char> buffer);
    size_t GetDeviceManufacturerString(std::span<char> buffer);
    size_t GetDeviceSerialNumberString(std::span<char> buffer);
    size_t GetDeviceConfigStringString(std::span<char> buffer);

    uint8_t GetAddress() { return Pipe0.Number; }
    DeviceDescriptor const& GetDescriptor() const { return Descriptor; }
    UsbInterfaceDescriptor const& GetInterfaceDescriptor(uint8_t index) const { return index < Interfaces.size() ? Interfaces[index] : NullInterfaceDescriptor; }
    UsbEndpointDescriptor FindEndpoint(uint8_t interfaceIndex, usb_transfer_type type, UsbDirection direction) const;

    HidDevice* GetHidDevice() { return IsHid() ? HidPayload : nullptr; }

    UsbParent ParentHub{};
    UsbPipe Pipe0{};
    UsbConfigControl Config{};
    std::vector<UsbInterfaceDescriptor> Interfaces{};
    std::vector<std::vector<UsbEndpointDescriptor>> Endpoints{};
    alignas(4) DeviceDescriptor Descriptor{};

    PayLoadType PayLoadId = PayLoadType::Error;
    HubDevice* HubPayload = nullptr;
    HidDevice* HidPayload = nullptr;
    MassStorageDevice* MassPayload = nullptr;

private:
    std::shared_ptr<UsbDriver> driver_;
};

#define ControlMessageTimeout 10

class UsbDriver
{
protected:
    struct IoHandleDeleter
    {
        UsbDriver* Driver = nullptr;

        void operator()(void* ptr) const
        {
            if (Driver)
            {
                Driver->DeleteIoHandle(ptr);
            }
        }
    };

    virtual void DeleteIoHandle(void* ptr) = 0;

public:
    virtual ~UsbDriver() = default;

    virtual RESULT GetError() = 0;

    using IoHandle = std::unique_ptr<void, IoHandleDeleter>;

    virtual IoHandle GetIoHandle(uint8_t deviceAddress) = 0;

    virtual std::generator<UsbDevice&> EnumerateDevices() = 0;

    /*-UsbGetRootHub ------------------------------------------------------------
    On a Universal Serial Bus, there exists a root hub. This if often a virtual
    device, and typically represents a one port hub, which is the physical
    universal serial bus for this computer. It is always address 1. It is present 
    to allow uniform software manipulation of the universal serial bus itself.
    This will return that FAKE rootHub or NULL on failure. Reason for failure is
    generally not having called USBInitialize to start the USB system.         
    11Apr17 LdB
    --------------------------------------------------------------------------*/
    virtual UsbDevice *UsbGetRootHub() = 0;

    /*-UsbDeviceAtAddress -------------------------------------------------------
    Given the unique USB address this will return the pointer to the USB device
    structure. If the address is not actually in use it will return NULL.
    11Apr17 LdB
    --------------------------------------------------------------------------*/
    virtual UsbDevice* UsbDeviceAtAddress (uint8_t devNumber) = 0;


    /*--------------------------------------------------------------------------}
    {					 PUBLIC USB CHANGE CHECKING ROUTINES					}
    {---------------------------------------------------------------------------}*/

    /*-UsbCheckForChange --------------------------------------------------------
    Recursively calls HubCheckConnection on all ports on all hubs connected to
    the root hub. It will hence automatically change the device tree matching
    any physical changes. If we don't have interrupts turned on you will need
    to poll this from time to time.
    10Apr17 LdB
    --------------------------------------------------------------------------*/
    virtual Async::task<void> UsbCheckForChange () = 0;

    /*--------------------------------------------------------------------------}
    {					 PUBLIC DISPLAY USB INTERFACE ROUTINES					}
    {---------------------------------------------------------------------------}*/

    /*-UsbGetDescription --------------------------------------------------------
    Returns a description for a device. This is not read from the device, this
    is just generated given by the driver.
    Unchanged from Alex Chadwick
    --------------------------------------------------------------------------*/
    virtual const char* UsbGetDescription (UsbDevice *device) = 0;

    /*--------------------------------------------------------------------------}
    {						 PUBLIC USB DESCRIPTOR ROUTINES						}
    {--------------------------------------------------------------------------*/

    /*-HCDGetDescriptor ---------------------------------------------------------
    Has the ability to fetches all the different descriptors from the device if
    you provide the right parameters. It is a marshal call that many internal
    descriptor reads will use and it has no checking on parameters. So if you
    provide invalid parameters it will most likely fail and return with error.
    The descriptor is read in two calls first the header is read to check the
    type matches and it provides the descriptor size. If the buffer length is
    longer than the descriptor the second call shortens the length to just the
    descriptor length. So the call provides the length of data requested or
    shorter if the descriptor is shorter than the buffer space provided.
    24Feb17 LdB
    --------------------------------------------------------------------------*/
    virtual Async::task<RESULT> HCDGetDescriptor (UsbDevice* device,
                            usb_descriptor_type type,				// The type of descriptor
                            uint8_t index,								// The index of the type descriptor
                            uint16_t langId,							// The language id
                            void* buffer,								// Buffer to recieve descriptor
                            uint32_t length,							// Maximumlength of descriptor
                            uint8_t recipient,							// Recipient flags									 
                            uint32_t *bytesTransferred,     			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)								
                            bool runHeaderCheck) = 0;       			// Whether to run header check

    /*-HCDSubmitControlMessage --------------------------------------------------
    Sends a control message to a device. Handles all necessary channel creation
    and other processing. The sequence of a control transfer is defined in the
    USB 2.0 manual section 5.5.  Success is indicated by return of Ok (0) all
    other codes indicate an error.
    24Feb17 LdB
    --------------------------------------------------------------------------*/
    virtual Async::task<RESULT> HCDSubmitControlMessageOUT(
        UsbDevice* device,
        std::byte* buffer,					// Data buffer both send and recieve				 
        uint32_t bufferLength,				// Buffer length for send or recieve
        UsbDeviceRequest request,	// USB request message
        uint32_t timeout,					// Timeout in microseconds on message
        uint32_t* bytesTransferred			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
    ) = 0;

    virtual Async::task<RESULT> HCDSubmitControlMessageIN(
        UsbDevice* device,
        std::byte* buffer,					// Data buffer both send and recieve				 
        uint32_t bufferLength,				// Buffer length for send or recieve
        UsbDeviceRequest request,	// USB request message
        uint32_t timeout,					// Timeout in microseconds on message
        uint32_t* bytesTransferred			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
    ) = 0;

    // Sends/recieves data from/to the given buffer to/from the given endpoint.
    virtual Async::task<RESULT> HCDEndpointTransfer(UsbDevice* device, UsbEndpointDescriptor endpoint, std::byte* buffer, uint32_t& bufferLength) = 0;

    void LOG(const char* format, ...) {}

    /*-INTERNAL: HCDReadStringDescriptor-----------------------------------------
    Reads the string descriptor at the given string index returning an ascii of
    the descriptor. Internally the descriptor is unicode so the raw descriptor
    is not returned. The code is setup to US English language support (0x409),
    and if a string does not have a valid English language string the default
    language is use to read blindly to satisfy enumeration. Non english speakers
    if you want to choose a different language you need to change 0x409 in the
    code below to your standard USB language ID you want.
    21Mar17 LdB
    --------------------------------------------------------------------------*/
    Async::task<RESULT> HCDReadStringDescriptor (UsbDevice& device,
                                    uint8_t stringIndex,				// String index to be returned
                                    char* buffer,						// Pointer to a buffer
                                    size_t& length);					// The size of that buffer

    // Shows the USB tree as ascii art using the Printf command
    void UsbShowTree();
};

Async::task<std::shared_ptr<UsbDriver>> UsbInitializeDesignWare();

namespace Usb::Xhci {
    Async::task<std::shared_ptr<UsbDriver>> UsbInitializeXhci(PCIe::Bcm2711Driver& pcie, PCIe::DeviceAddress const& deviceAddress);
} // namespace Usb::Xhci
