#pragma once

#include <stdint.h>


enum RESULT {
    OK = 0,
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

/***************************************************************************}
{					      PUBLIC INTERFACE ROUTINES			                }
****************************************************************************/

/*--------------------------------------------------------------------------}
{					 PUBLIC GENERIC USB INTERFACE ROUTINES					}
{---------------------------------------------------------------------------}*/

/*-UsbInitialise-------------------------------------------------------------
 Initialises the USB driver by performing necessary interfactions with the
 host controller driver, and enumerating the initial device tree.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT UsbInitialise ();

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
UsbDevice *UsbDeviceAtAddress (uint8_t devNumber);


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
{						 PUBLIC HID INTERFACE ROUTINES						}
{---------------------------------------------------------------------------}*/

/*- HIDReadDescriptor ------------------------------------------------------
 Reads the HID descriptor from the given device. The call will error if the
 device is not a HID device, you can always check that by the use of IsHID.
 23Mar17 LdB
 --------------------------------------------------------------------------*/
RESULT HIDReadDescriptor (uint8_t devNumber,						// Device number (address) of the device to read 
                           uint8_t hidIndex,							// Which hid configuration information is requested from
                          uint8_t* Buffer,							// Pointer to a buffer to receive the descriptor
                          uint16_t Length);							// Maxium length of the buffer 

/*- HIDReadReport ----------------------------------------------------------
 Reads the HID report from the given device. The call will error if device
 is not a HID device, you can always check that by the use of IsHID.
 23Mar17 LdB
 --------------------------------------------------------------------------*/
RESULT HIDReadReport (uint8_t devNumber,							// Device number (address) of the device to read
                      uint8_t hidIndex,								// Which hid configuration information is requested from
                      uint16_t reportValue,							// Hi byte = enum HidReportType  Lo Byte = Report Index (0 = default)  
                      uint8_t* Buffer,								// Pointer to a buffer to recieve the report
                      uint16_t Length);								// Length of the report

/*- HIDWriteReport ----------------------------------------------------------
 Writes the HID report located in buffer to the given device. This call will 
 error if device is not a HID device, you can always check that by the use of 
 IsHID.
 23Mar17 LdB
 --------------------------------------------------------------------------*/
RESULT HIDWriteReport (uint8_t devNumber,							// Device number (address) of the device to write report to
                       uint8_t hidIndex,							// Which hid configuration information is writing to
                       uint16_t reportValue,						// Hi byte = enum HidReportType  Lo Byte = Report Index (0 = default) 
                       uint8_t* Buffer,								// Pointer to a buffer containing the report
                       uint16_t Length);							// Length of the report

/*- HIDSetProtocol ----------------------------------------------------------
 Many USB HID devices support multiple low level protocols. For example most
 mice and keyboards have a BIOS Boot mode protocol that makes them look like
 an old DOS keyboard. They also have another protocol which is more advanced.
 This call enables the switch between protocols. What protocols are available
 and what interface is retrieved and parsed from Descriptors from the device.
 23Mar17 LdB
 --------------------------------------------------------------------------*/
RESULT HIDSetProtocol (uint8_t devNumber,							// Device number (address) of the device
                       uint8_t interface,							// Interface number to change protocol on
                       uint16_t protocol);							// The protocol number request

RESULT HIDSetIdle (uint8_t devNumber, uint8_t hidIndex);
