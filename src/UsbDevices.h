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
#include "Async.h"

#include <span>

#include <stdint.h>


enum RESULT {
    Ok = 0,
    ErrorGeneral = -1,
    ErrorArgument = -2,
    ErrorRetry = -3,
    ErrorDevice = -4,
    ErrorIncompatible = -5,
    ErrorCompiler = -6,
    ErrorMemory = -7,
    ErrorTimeout = -8,
    ErrorHardware = -9,
    ErrorTransmission = -10,
    ErrorDisconnected = -11,
    ErrorDeviceNumber = -12,
    ErrorTooManyRetries = -13,
    ErrorIndex = -14,
    ErrorNotHID = -15,
    ErrorStall = -16,
};

struct UsbDevice;
struct HidDevice;

#define MaximumDevices 32											// Max number of devices with a USB node we will allow 

#define ControlMessageTimeout 10

/***************************************************************************}
{					      PUBLIC INTERFACE ROUTINES			                }
****************************************************************************/

/*--------------------------------------------------------------------------}
{					 PUBLIC GENERIC USB INTERFACE ROUTINES					}
{---------------------------------------------------------------------------}*/

/*-UsbInitialize-------------------------------------------------------------
 Initializes the USB driver by performing necessary interfactions with the
 host controller driver, and enumerating the initial device tree.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
Async::task<RESULT> UsbInitialize();

DeviceDescriptor GetDeviceDescriptor(uint8_t devNumber);

size_t GetDeviceProductString(uint8_t devNumber, std::span<char> buffer);
size_t GetDeviceManufacturerString(uint8_t devNumber, std::span<char> buffer);
size_t GetDeviceSerialNumberString(uint8_t devNumber, std::span<char> buffer);
size_t GetDeviceConfigStringString(uint8_t devNumber, std::span<char> buffer);

/*-IsHub---------------------------------------------------------------------
 Will return if the given usbdevice is infact a hub and thus has hub payload
 data available. Remember the gateway node of a hub is a normal usb device.
 You should always call this first up in any routine that accesses the hub
 payload to make sure the payload pointers are valid. If it returns true it
 is safe to proceed and do things with the hub payload via it's pointer.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
bool IsHub (uint8_t devNumber);

/*-IsHid---------------------------------------------------------------------
 Will return if the given usbdevice is infact a hid and thus has hid payload
 data available. Remember a hid device is a normal usb device which takes
 human input (like keyboard, mouse etc). You should always call this first
 in any routine that accesses the hid payload to make sure the pointers are
 valid. If it returns true it is safe to proceed and do things with the hid
 payload via it's pointer.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
bool IsHid (uint8_t devNumber);

/*-IsMassStorage------------------------------------------------------------
 Will return if the given usbdevice is infact a mass storage device and thus
 has a mass storage payload data available. You should always call this first
 in any routine that accesses the storage payload to make sure the pointers
 are valid. If it returns true it is safe to proceed and do things with the
 storage payload via it's pointer.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
bool IsMassStorage (uint8_t devNumber);

/*-IsMouse-------------------------------------------------------------------
 Will return if the given usbdevice is infact a mouse. This initially checks
 the device IsHid and then refines that down to looking at the interface and
 checking it is defined as a mouse.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
bool IsMouse (uint8_t devNumber);

/*-IsKeyboard----------------------------------------------------------------
 Will return if the given usbdevice is infact a keyboard. This initially will
 check the device IsHid and then refines that down to looking at the interface 
 and checking it is defined as a keyboard.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
bool IsKeyboard (uint8_t devNumber);

/*-UsbGetRootHub ------------------------------------------------------------
 On a Universal Serial Bus, there exists a root hub. This if often a virtual
 device, and typically represents a one port hub, which is the physical
 universal serial bus for this computer. It is always address 1. It is present 
 to allow uniform software manipulation of the universal serial bus itself.
 This will return that FAKE rootHub or NULL on failure. Reason for failure is
 generally not having called USBInitialize to start the USB system.         
 11Apr17 LdB
 --------------------------------------------------------------------------*/
UsbDevice *UsbGetRootHub (void);

/*-UsbDeviceAtAddress -------------------------------------------------------
 Given the unique USB address this will return the pointer to the USB device
 structure. If the address is not actually in use it will return NULL.
 11Apr17 LdB
 --------------------------------------------------------------------------*/
UsbDevice* UsbDeviceAtAddress (uint8_t devNumber);

uint32_t   GetDeviceNumber(UsbDevice*);
HidDevice* GetHidDevice   (UsbDevice*);

UsbInterfaceDescriptor GetInterfaceDescriptor(UsbDevice* device, uint8_t interfaceIndex);
UsbEndpointDescriptor  FindEndpoint(UsbDevice* device, uint8_t interfaceIndex, usb_transfer_type type, UsbDirection direction);


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
void UsbCheckForChange (void);

/*--------------------------------------------------------------------------}
{					 PUBLIC DISPLAY USB INTERFACE ROUTINES					}
{---------------------------------------------------------------------------}*/

/*-UsbGetDescription --------------------------------------------------------
 Returns a description for a device. This is not read from the device, this
 is just generated given by the driver.
 Unchanged from Alex Chadwick
 --------------------------------------------------------------------------*/
const char* UsbGetDescription (UsbDevice *device);

/*-UsbShowTree --------------------------------------------------------------
 Shows the USB tree as ascii art using the Printf command. The normal command
 to show from roothub up is UsbShowTree(UsbGetRootHub(), 1, '+');
 14Mar17 LdB
 --------------------------------------------------------------------------*/
void UsbShowTree (UsbDevice *root, const int level, const char tee);

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
RESULT HCDGetDescriptor (UsbDevice* device,
                         usb_descriptor_type type,				// The type of descriptor
                         uint8_t index,								// The index of the type descriptor
                         uint16_t langId,							// The language id
                         void* buffer,								// Buffer to recieve descriptor
                         uint32_t length,							// Maximumlength of descriptor
                         uint8_t recipient,							// Recipient flags									 
                         uint32_t *bytesTransferred,     			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)								
                         bool runHeaderCheck);						// Whether to run header check

/*-HCDSumbitControlMessage --------------------------------------------------
 Sends a control message to a device. Handles all necessary channel creation
 and other processing. The sequence of a control transfer is defined in the
 USB 2.0 manual section 5.5.  Success is indicated by return of Ok (0) all
 other codes indicate an error.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT HCDSumbitControlMessageOUT(
    UsbDevice* device,
    std::byte* buffer,					// Data buffer both send and recieve				 
    uint32_t bufferLength,				// Buffer length for send or recieve
    UsbDeviceRequest&& request,	// USB request message
    uint32_t timeout,					// Timeout in microseconds on message
    uint32_t* bytesTransferred			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
);

RESULT HCDSumbitControlMessageIN(
    UsbDevice* device,
    std::byte* buffer,					// Data buffer both send and recieve				 
    uint32_t bufferLength,				// Buffer length for send or recieve
    UsbDeviceRequest&& request,	// USB request message
    uint32_t timeout,					// Timeout in microseconds on message
    uint32_t* bytesTransferred			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
);

// Sends/recieves data from/to the given buffer to/from the given endpoint.
RESULT HCDEndpointTransfer(UsbDevice* device, UsbEndpointDescriptor endpoint, std::byte* buffer, uint32_t& bufferLength);
