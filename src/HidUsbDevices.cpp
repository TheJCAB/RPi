// Basic HID USB device driver for Raspberry Pi 3.
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
#include "HidUsbDevices.h"

#include "UsbDevices.h"
#include "DesignWareUsb.h"

#include "emb-stdio.h"				// Needed for printf

#define LOG(...)
//#define LOG(...) printf2(__VA_ARGS__)
#define LOG_DEBUG(...)
//#define LOG_DEBUG(...) printf2(__VA_ARGS__)

#define MaxHIDPerDevice 4
#define MaximumHids 16												// Maximum number of HID payloads we will allow

/*--------------------------------------------------------------------------}
{ 		 USB HID 1.11 descriptor structure as per manual in 6.2.1		    }
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) HidDescriptor {
    struct UsbDescriptorHeader Header;								// +0x0 Length of this descriptor, +0x1 DEVICE descriptor type (enum DescriptorType)
    uint16_t HidVersion;										// (bcd version) +0x2 
    enum HidCountry {
        CountryNotSupported = 0,
        Arabic = 1,
        Belgian = 2,
        CanadianBilingual = 3,
        CanadianFrench = 4,
        CzechRepublic = 5,
        Danish = 6,
        Finnish = 7,
        French = 8,
        German = 9,
        Greek = 10,
        Hebrew = 11,
        Hungary = 12,
        International = 13,
        Italian = 14,
        Japan = 15,
        Korean = 16,
        LatinAmerican = 17,
        Dutch = 18,
        Norwegian = 19,
        Persian = 20,
        Poland = 21,
        Portuguese = 22,
        Russian = 23,
        Slovakian = 24,
        Spanish = 25,
        Swedish = 26,
        SwissFrench = 27,
        SwissGerman = 28,
        Switzerland = 29,
        Taiwan = 30,
        TurkishQ = 31,
        EnglishUk = 32,
        EnglishUs = 33,
        Yugoslavian = 34,
        TurkishF = 35,
    } Countrycode : 8;												// +0x4
    uint8_t DescriptorCount;										// +0x5
    enum usb_descriptor_type Type : 8;								// +0x6
    uint16_t Length;											    // +0x7 
};

/*--------------------------------------------------------------------------}
{	 USB hid structure which is just extra data attached to a USB node	    }
{---------------------------------------------------------------------------}*/
struct HidDevice {
    HidDescriptor Descriptor[MaxHIDPerDevice];	// HID descriptor of this device
    uint8_t HIDInterface[MaxHIDPerDevice];		// The interface the HID descriptor is on
    uint8_t MaxHID;
};

HidDevice HidTable[MaximumHids] = {};						// Usb hid device allocation table


/*==========================================================================}
{      INTERNAL FUNCTIONS THAT ADD AND REMOCE HID PAYLOADS TO DEVICES	    }
{==========================================================================*/

HidDevice* AllocateHidPayload()
{
    for (int number = 0; number < MaximumHids; number++)
    {
        // Find first free entry
        if (HidTable[number].MaxHID == 0)
        {
            HidTable[number].MaxHID = MaxHIDPerDevice;
            return &HidTable[number];
        }
    }
    return nullptr;
}

void FreeHidPayload(HidDevice* device)
{
    *device = {};
}

bool SetHidDescriptor(HidDevice* hidDevice, uint8_t hidIndex, uint8_t interface, uint8_t const* buffer, uint8_t size)
{
    if (sizeof(struct HidDescriptor) != size)
    {
        LOG("HID Entry wrong size\n");
        return false;
    }
    if (hidDevice->MaxHID < MaxHIDPerDevice && hidDevice->MaxHID != hidIndex)
    {
        LOG_DEBUG("HID Index %u out of order. Expected: %u\n", hidIndex, hidDevice->MaxHID);
        return false;
    }
    // We can hold a limited sane number of HID descriptors
    if (hidIndex >= MaxHIDPerDevice)
    {
        LOG("Too many HID entries\n");
        return false;
    }

    hidDevice->Descriptor[hidIndex] = *reinterpret_cast<HidDescriptor const*>(buffer);
    hidDevice->HIDInterface[hidIndex] = interface; // Hold the interface the HID is on
    hidDevice->MaxHID = hidIndex + 1;
    return true;
}

uint8_t GetHidCount(HidDevice* device)
{
    if (device == nullptr) return 0;
    return device->MaxHID;
}

void PrintHid(HidDevice* device, uint8_t hidIndex, char const* indent)
{
    printf("%s    - HID Record Version %X Country %u DescriptorCount %u Type %u Length %u Interface %u\n",
        indent,
        device->Descriptor[hidIndex].HidVersion,
        device->Descriptor[hidIndex].Countrycode,
        device->Descriptor[hidIndex].DescriptorCount,
        device->Descriptor[hidIndex].Type,
        device->Descriptor[hidIndex].Length,
        device->HIDInterface[hidIndex]
    );
}

void describe_hid_descriptor(const uint8_t* data, size_t length)
{
    uint32_t i = 0;
    while (i < length) {
        uint8_t prefix = data[i++];
        uint8_t size = prefix & 0x03;
        uint8_t type = (prefix >> 2) & 0x03;
        uint8_t tag  = (prefix >> 4) & 0x0F;

        if (prefix == 0xFE) { // Long item (rare)
            printf2("Long item not supported\n");
            break;
        }

        uint32_t value = 0;
        for (uint8_t j = 0; j < size; ++j) {
            if (i < length)
                value |= static_cast<uint32_t>(data[i++]) << (8 * j);
        }

        char const* type_str;
        char const* tag_str;

        switch (type) {
            case 0: type_str = "Main"; break;
            case 1: type_str = "Global"; break;
            case 2: type_str = "Local"; break;
            default: type_str = "Reserved"; break;
        }

        switch (type) {
            case 0: // Main
                switch (tag) {
                    case 8: tag_str = "Input"; break;
                    case 9: tag_str = "Output"; break;
                    case 11: tag_str = "Feature"; break;
                    case 10: tag_str = "Collection"; break;
                    case 12: tag_str = "End Collection"; break;
                    default: tag_str = "Unknown Main"; break;
                }
                break;
            case 1: // Global
                switch (tag) {
                    case 0: tag_str = "Usage Page"; break;
                    case 1: tag_str = "Logical Minimum"; break;
                    case 2: tag_str = "Logical Maximum"; break;
                    case 7: tag_str = "Report Size"; break;
                    case 8: tag_str = "Report ID"; break;
                    case 9: tag_str = "Report Count"; break;
                    default: tag_str = "Unknown Global"; break;
                }
                break;
            case 2: // Local
                switch (tag) {
                    case 0: tag_str = "Usage"; break;
                    case 1: tag_str = "Usage Minimum"; break;
                    case 2: tag_str = "Usage Maximum"; break;
                    default: tag_str = "Unknown Local"; break;
                }
                break;
            default:
                tag_str = "Unknown";
                break;
        }

        printf("[%02u] %10s %20s Value: 0x%X (%u)\n", i - 1, type_str, tag_str, value, value);
    }
}

/*-INTERNAL: EnumerateHID------------------------------------------------------
 If normal device enumeration detects a hid device, after normal single node
 enumeration it will call this procedure to enumerate connected HID devices.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT EnumerateHID (UsbDevice* device)
{
    auto const hidDevice = GetHidDevice(device);

    uint8_t Buf[1024];
    for (int i = 0; i < hidDevice->MaxHID; i++) {
        auto const& descriptor = hidDevice->Descriptor[i];
        auto const interface = GetInterfaceDescriptor(device, hidDevice->HIDInterface[i]);
        LOG("HID details: Version: %4x, Language: %i Descriptions: %i, Type: %i, Protocol: %i, NumInterface: %i\n",
            descriptor.HidVersion,
            descriptor.Countrycode,
            descriptor.DescriptorCount,
            descriptor.Type,
            interface.Protocol,
            interface.Number);

        if (HIDReadDescriptor(GetDeviceNumber(device), i, &Buf[0], sizeof(Buf)) == Ok) {
            LOG_DEBUG("HID REPORT> Page usage: 0x%02x%02x, Usage: 0x%02x%02x, Collection: 0x%02x%02x\n",
                Buf[0], Buf[1], Buf[2], Buf[3], Buf[4], Buf[5]);

            describe_hid_descriptor(Buf + 6, hidDevice->Descriptor[i].Length - 6);

            LOG_DEBUG("Bytes: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                Buf[6], Buf[7], Buf[8], Buf[9], Buf[10], Buf[11], Buf[12], Buf[13], Buf[14], Buf[15], Buf[16], Buf[17], Buf[18], Buf[19], Buf[20], Buf[21],
                Buf[22], Buf[23], Buf[24], Buf[25], Buf[26], Buf[27], Buf[28], Buf[29], Buf[30], Buf[31], Buf[32], Buf[33], Buf[34], Buf[35], Buf[36], Buf[37]);
            LOG_DEBUG("Bytes: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                Buf[38], Buf[39], Buf[40], Buf[41], Buf[42], Buf[43], Buf[44], Buf[45], Buf[46], Buf[47], Buf[48], Buf[49], Buf[50], Buf[51]);
        }
    }
    return Ok;														// Return success
}



/*--------------------------------------------------------------------------}
{						 PUBLIC HID INTERFACE ROUTINES						}
{--------------------------------------------------------------------------*/

/*- HIDReadDescriptor ------------------------------------------------------
 Reads the HID descriptor from the given device. The call will error if the
 device is not a HID device, you can always check that by the use of IsHID.
 23Mar17 LdB
 --------------------------------------------------------------------------*/
RESULT HIDReadDescriptor (uint8_t devNumber,						// Device number (address) of the device to read 
                          uint8_t hidIndex,							// Which hid configuration information is requested from
                          uint8_t* Buffer,							// Pointer to a buffer to receive the descriptor
                          uint16_t Length)							// Maxium length of the buffer 
{
    RESULT result;
    uint32_t transfer = 0;											// Preset transfer to zero
    volatile uint8_t Hi;
    volatile uint8_t Lo;

    if ((Buffer == NULL) || (Length == 0))	return ErrorArgument;	// Check buffer and length is valid
    if ((devNumber == 0) || (devNumber > MaximumDevices))
        return ErrorDeviceNumber;									// Device number not valid
    auto const device = UsbDeviceAtAddress(devNumber);				// Fetch pointer to device number requested
    if (device == nullptr)
    {
        return ErrorDeviceNumber;
    }
    auto const hidDevice = GetHidDevice(device);			// Fetch pointer to device number requested
    if (hidDevice == nullptr)
    {
        return ErrorNotHID;
    }
    if (hidIndex > hidDevice->MaxHID) return ErrorIndex;	// Invalid HID descriptor index requested
                                                                    // Calculate HID descriptor size
    uint16_t sizeToRead = hidDevice->Descriptor[hidIndex].Length;	// Total size we need to read

    /* Okay read the HID descriptor */
    result = HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_HID_REPORT, 0,
        hidDevice->HIDInterface[hidIndex],					// Index number of HID index
        Buffer, sizeToRead, 0x81, &transfer, false);				// Read the HID report descriptor 	
    if ((result != Ok) || (transfer != sizeToRead)) {				// Read/transfer failed
        LOG("HCD: Fetch HID descriptor %u for device: %u failed.\n",
            hidDevice->HIDInterface[hidIndex], 
            GetDeviceNumber(device));									// Log the error
        return ErrorDevice;											// No idea what problem is so bail
    }

    // We buffered for DMA alignment .. Now transfer to user pointer
    if (Length < sizeToRead) sizeToRead = Length;					// Insufficient buffer size for descriptor
    return Ok;														// Return success
}


/*- HIDReadReport ----------------------------------------------------------
 Reads the HID report from the given device. The call will error if device
 is not a HID device, you can always check that by the use of IsHID.
 23Mar17 LdB
 --------------------------------------------------------------------------*/
RESULT HIDReadReport (uint8_t devNumber,							// Device number (address) of the device to read
                      uint8_t hidIndex,								// Which hid configuration information is requested from
                      uint16_t reportValue,							// Hi byte = enum HidReportType  Lo Byte = Report Index (0 = default) 
                      uint8_t* Buffer,								// Pointer to a buffer to recieve the report
                      uint16_t Length)								// Length of the report
{
    RESULT result;
    uint32_t transfer = 0;											// Preset transfer to zero
    
    if ((Buffer == NULL) || (Length == 0))	return ErrorArgument;	// Check buffer and length is valid
    auto const device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr)
    {
        return ErrorDeviceNumber;
    }
    auto const hidDevice = GetHidDevice(device);
    if (hidDevice == nullptr)
    {
        return ErrorNotHID;
    }
    
    result = HCDSumbitControlMessageIN(
        device,												// Control pipe
        Buffer,														// Pass buffer pointer
        Length,														// Read length requested
        UsbDeviceRequest {
            .Type = 0xa1,											// D7 = Device to Host, D5 = Vendor, D0 = Interface = 1010 0001 = 0xA1	
            .Request = GetReport,									// Get report
            .Value = reportValue,									// Report value requested
            .Index = hidDevice->HIDInterface[hidIndex],	// HID interface
            .Length = Length,
        },
        ControlMessageTimeout,										// The standard timeout for any control message
        &transfer);													// Monitor transfer byte count
    if (result != Ok) return result;								// Return error
    return Ok;														// Return success
}

RESULT HIDSetIdle (uint8_t devNumber, uint8_t hidIndex)
{
    RESULT result;
    uint32_t transfer = 0;											// Preset transfer to zero

    auto const device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr)
    {
        return ErrorDeviceNumber;
    }
    auto const hidDevice = GetHidDevice(device);
    if (hidDevice == nullptr)
    {
        return ErrorNotHID;
    }

    result = HCDSumbitControlMessageOUT(
        device,												// Control pipe
        nullptr,													// Pass buffer pointer
        0,															// Read length requested
        UsbDeviceRequest {
            .Type = 0xa1,											// D7 = Device to Host, D5 = Vendor, D0 = Interface = 1010 0001 = 0xA1	
            .Request = SetIdle,
            .Value = 0,									// Report value requested
            .Index = hidDevice->HIDInterface[hidIndex],	// HID interface
            .Length = 0,
        },
        ControlMessageTimeout,										// The standard timeout for any control message
        &transfer);													// Monitor transfer byte count
    if (result != Ok) return result;								// Return error
    return Ok;														// Return success
}



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
                       uint16_t Length)								// Length of the report
{
    RESULT result;
    uint32_t transfer = 0;											// Preset transfer to zero
    if ((Buffer == NULL) || (Length == 0))	return ErrorArgument;	// Check buffer and length is valid

    auto const device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr)
    {
        return ErrorDeviceNumber;
    }
    auto const hidDevice = GetHidDevice(device);
    if (hidDevice == nullptr)
    {
        return ErrorNotHID;
    }

    result = HCDSumbitControlMessageOUT(
        device,												// Control pipe
        Buffer,														// Transfer buffer pointer
        Length,														// Write length requested
        UsbDeviceRequest {
            .Type = 0x21,											// D7 = Host to Device  D5 = Vendor, D0 = Interface = 0010 0001 = 0x21	
            .Request = SetReport,									// Set report
            .Value = reportValue,									// Report value requested
            .Index = hidDevice->HIDInterface[hidIndex],	// HID interface
            .Length = Length,										// Length of report
        },
        ControlMessageTimeout,										// The standard timeout for any control message
        &transfer);													// Monitor transfer byte count
    if (result != Ok) return result;								// Return error
    if (transfer != Length) return ErrorGeneral;					// Device didn't accept all the data
    return Ok;														// Return success
}

///*- HIDSetProtocol ----------------------------------------------------------
//Many USB HID devices support multiple low level protocols. For example most
//mice and keyboards have a BIOS Boot mode protocol that makes them look like
//an old DOS keyboard. They also have another protocol which is more advanced.
//This call enables the switch between protocols. What protocols are available
//and what interface is retrieved and parsed from Descriptors from the device.
//23Mar17 LdB
//--------------------------------------------------------------------------*/
//RESULT HIDSetInterface (uint8_t devNumber,							// Device number (address) of the device
//                       uint8_t interface)							// The protocol number request
//{
//    RESULT result;
//
//    auto const device = UsbDeviceAtAddress(devNumber);
//    if (device == nullptr)
//    {
//        return ErrorDeviceNumber;
//    }
//    auto const hidDevice = GetHidDevice(device);
//    if (hidDevice == nullptr)
//    {
//        return ErrorNotHID;
//    }
//
//    result = HCDSumbitControlMessageOUT(
//        device,												// Use the control pipe
//        NULL,														// No buffer for command
//        0,															// No buffer length because of above
//        UsbDeviceRequest {
//            .Type = 0x21,											// D7 = Host to Device  D5 = Vendor D0 = Interface = 0010 0001 = 0x21	
//            .Request = SetInterface,
//            .Value = protocol,										// Protocol
//            .Index = interface,										// Interface
//            .Length = 0,											// No data for command
//        },
//        ControlMessageTimeout,										// Standard control message timeout
//        NULL);														// No data so can ignore transfer bytes
//    return result;
//}

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
                       uint16_t protocol)							// The protocol number request
{
    RESULT result;

    auto const device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr)
    {
        return ErrorDeviceNumber;
    }
    auto const hidDevice = GetHidDevice(device);
    if (hidDevice == nullptr)
    {
        return ErrorNotHID;
    }

    result = HCDSumbitControlMessageOUT(
        device,												// Use the control pipe
        NULL,														// No buffer for command
        0,															// No buffer length because of above
        UsbDeviceRequest {
            .Type = 0x21,											// D7 = Host to Device  D5 = Vendor D0 = Interface = 0010 0001 = 0x21	
            .Request = SetProtocol,									// Set protocol request
            .Value = protocol,										// Protocol
            .Index = interface,										// Interface
            .Length = 0,											// No data for command
        },
        ControlMessageTimeout,										// Standard control message timeout
        NULL);														// No data so can ignore transfer bytes
    return result;
}

/*==========================================================================}
{                   HID INTERRUPT IN TRANSFER FUNCTIONS                    }
{==========================================================================*/

/*
 * HID Interrupt IN Example Usage:
 * 
 * // Traditional polling approach - inefficient, high CPU usage
 * void KeyboardPollingLoop(uint8_t keyboardDevice) {
 *     uint8_t keyboardData[8];
 *     while (true) {
 *         if (HIDReadReport(keyboardDevice, 0, 0x0100, keyboardData, 8) == Ok) {
 *             ProcessKeyboardData(keyboardData);
 *         }
 *         Timer::Delay(10000); // Poll every 10ms - wastes CPU cycles
 *     }
 * }
 * 
 * // New interrupt IN approach - efficient, event-driven
 * void KeyboardInterruptLoop(uint8_t keyboardDevice) {
 *     uint8_t keyboardData[8];
 *     uint8_t pollInterval;
 *     uint32_t bytesRead;
 *     
 *     // Get the device's preferred polling interval
 *     if (HIDGetInterruptInterval(keyboardDevice, 0, &pollInterval) == Ok) {
 *         LOG("Keyboard polling interval: %d ms\n", pollInterval);
 *     }
 *     
 *     while (true) {
 *         // This will only return when the device has new data
 *         if (HIDReadInterruptReport(keyboardDevice, 0, keyboardData, 8, &bytesRead) == Ok) {
 *             ProcessKeyboardData(keyboardData, bytesRead);
 *         }
 *         // Small delay based on device's preferred interval
 *         Timer::Delay(pollInterval * 1000); // Convert ms to microseconds
 *     }
 * }
 */

/*- HIDStartInterruptIN -----------------------------------------------------
 Starts an interrupt IN transfer for the specified HID device. This allows
 for asynchronous reading of HID reports without the need for constant polling.
 The function identifies the interrupt IN endpoint and sets up a transfer.
 --------------------------------------------------------------------------*/
RESULT HIDStartInterruptIN (uint8_t devNumber,                      // Device number (address) of the HID device
                           uint8_t hidIndex,                        // Which HID configuration to use
                           uint8_t* Buffer,                         // Buffer to receive interrupt data
                           uint16_t BufferLength,                   // Length of the buffer
                           uint32_t* BytesTransferred)             // Pointer to store actual bytes transferred
{
    uint32_t transfer = 0;

    // Validate parameters
    if ((Buffer == NULL) || (BufferLength == 0))
        return ErrorArgument;

    auto const device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr)
    {
        return ErrorDeviceNumber;
    }
    auto const hidDevice = GetHidDevice(device);
    if (hidDevice == nullptr)
    {
        return ErrorNotHID;
    }

    if (hidIndex >= hidDevice->MaxHID)
    {
        return ErrorIndex; // Invalid HID index
    }

    // Find the interrupt IN endpoint for this interface
    uint8_t interfaceIndex = hidDevice->HIDInterface[hidIndex];
    auto const endpoint = FindEndpoint(device, interfaceIndex, USB_TRANSFER_TYPE_INTERRUPT, USB_DIRECTION_IN);
    if (endpoint.Header.DescriptorLength == 0) 
    {
        LOG("HID: No interrupt IN endpoint found for device %d, interface %d\n", 
            devNumber, interfaceIndex);
        return ErrorDevice;
    }

    LOG_DEBUG("HID: Starting interrupt IN transfer on device %d, endpoint %d, interval %dms\n",
        devNumber, endpoint.EndpointAddress.Number, endpoint.Interval);

    // Start the interrupt transfer
    uint32_t transferLength = BufferLength;
    auto const result = HCDEndpointTransfer(device, endpoint, Buffer, transferLength);
    
    if (result == RESULT::Ok) {
        if (BytesTransferred != nullptr) *BytesTransferred = transferLength;
        LOG_DEBUG("HID: Interrupt IN transfer completed, %d bytes received\n", transferLength);
    } else {
        LOG("HID: Interrupt IN transfer failed for device %d, endpoint %u error: %d\n", devNumber, 
            endpoint.EndpointAddress.Number, result);
        if (BytesTransferred != nullptr) *BytesTransferred = 0;
    }

    return result;
}

/*- HIDStopInterruptIN ------------------------------------------------------
 Stops an active interrupt IN transfer for the specified HID device. This
 function would typically abort any ongoing transfers and cleanup resources.
 Note: The current implementation is a placeholder as the underlying HCD
 layer would need specific abort functionality.
 --------------------------------------------------------------------------*/
RESULT HIDStopInterruptIN (uint8_t devNumber,                       // Device number (address) of the HID device
                          uint8_t hidIndex)                         // Which HID configuration to stop
{
    auto const device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr)
    {
        return ErrorDeviceNumber;
    }
    auto const hidDevice = GetHidDevice(device);
    if (hidDevice == nullptr)
    {
        return ErrorNotHID;
    }

    if (hidIndex >= hidDevice->MaxHID) return ErrorIndex; // Invalid HID index

    LOG_DEBUG("HID: Stopping interrupt IN transfer for device %d\n", devNumber);
    
    // Note: In a full implementation, this would:
    // 1. Cancel any pending interrupt transfers
    // 2. Clean up allocated channels/resources
    // 3. Reset endpoint state if needed
    // For now, we just return Ok as the transfers are synchronous
    
    return Ok;
}

/*- HIDReadInterruptReport --------------------------------------------------
 Convenience function that performs a single interrupt IN transfer to read
 a HID report. This is useful for applications that want to use interrupt
 mode but still handle transfers synchronously.
 
 Example usage for polling keyboard state with interrupt IN:
 
    // Traditional polling approach:
    // while (true) {
    //     if (HIDReadReport(keyboardDevice, 0, 0x0100, buffer, 8) == Ok) {
    //         // Process keyboard data
    //     }
    //     Timer::Delay(10000); // 10ms delay
    // }
    
    // New interrupt IN approach:
    // uint32_t bytesRead;
    // if (HIDReadInterruptReport(keyboardDevice, 0, buffer, 8, &bytesRead) == Ok) {
    //     // Process keyboard data - only called when device has new data
    // }
 --------------------------------------------------------------------------*/
RESULT HIDReadInterruptReport (uint8_t devNumber,                   // Device number (address) of the HID device
                              uint8_t hidIndex,                     // Which HID configuration to use
                              uint8_t* Buffer,                      // Buffer to receive the report
                              uint16_t BufferLength,                // Length of the buffer
                              uint32_t* BytesTransferred)          // Pointer to store actual bytes transferred
{
    return HIDStartInterruptIN(devNumber, hidIndex, Buffer, BufferLength, BytesTransferred);
}

/*- HIDGetInterruptInterval -------------------------------------------------
 Gets the polling interval for the interrupt IN endpoint of a HID device.
 This value indicates how often the device should be polled for new data.
 The interval is specified in frames (1ms for full/high speed, 1-255ms for low speed).
 --------------------------------------------------------------------------*/
RESULT HIDGetInterruptInterval (uint8_t devNumber,                  // Device number (address) of the HID device
                               uint8_t hidIndex,                    // Which HID configuration to use
                               uint8_t* Interval)                  // Pointer to store the interval in milliseconds
{
    // Validate parameters
    if (Interval == NULL) return ErrorArgument;

    auto const device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr)
    {
        return ErrorDeviceNumber;
    }
    auto const hidDevice = GetHidDevice(device);
    if (hidDevice == nullptr)
    {
        return ErrorNotHID;
    }

    if (hidIndex >= hidDevice->MaxHID) return ErrorIndex; // Invalid HID index

    // Find the interrupt IN endpoint for this interface
    uint8_t interfaceIndex = hidDevice->HIDInterface[hidIndex];
    auto const endpoint = FindEndpoint(device, interfaceIndex, USB_TRANSFER_TYPE_INTERRUPT, USB_DIRECTION_IN);
    if (endpoint.Header.DescriptorLength == 0)
    {
        LOG("HID: No interrupt IN endpoint found for device %d, interface %d\n", 
            devNumber, interfaceIndex);
        return ErrorDevice;
    }

    // Return the polling interval
    *Interval = endpoint.Interval;
    
    LOG_DEBUG("HID: Device %d interrupt IN endpoint interval: %d ms\n", devNumber, *Interval);
    
    return Ok;
}

/*==========================================================================}
{                   HID INTERRUPT IN ENABLE FUNCTIONS                     }
{==========================================================================*/

/*- HIDEnableInterruptIN ----------------------------------------------------
 Enables interrupt IN communication for a HID device. This function performs
 any necessary setup steps to ensure the device is ready for interrupt IN
 transfers. For most standard HID devices, interrupt IN endpoints are enabled
 automatically after USB configuration, but some devices may require specific
 protocol or idle settings.
 
 This function:
 1. Verifies the device has an interrupt IN endpoint
 2. Optionally sets the HID protocol (boot vs report protocol)
 3. Optionally configures the idle rate to reduce unnecessary transfers
 
 Parameters:
 - setProtocol: If true, sets the device to report protocol (1) vs boot protocol (0)
 - protocolValue: 0 = boot protocol, 1 = report protocol
 - setIdleRate: If true, sets the idle rate to reduce polling frequency
 - idleRate: 0 = infinite (only send on change), >0 = duration in 4ms units
 
 Returns RESULT::Ok if successful, error code otherwise.
 --------------------------------------------------------------------------*/
RESULT HIDEnableInterruptIN (uint8_t devNumber,                     // Device number (address) of the HID device
                            uint8_t hidIndex,                       // Which HID configuration to enable
                            bool setProtocol,                       // Whether to set the protocol
                            uint8_t protocolValue,                  // Protocol value (0=boot, 1=report)
                            bool setIdleRate,                       // Whether to set idle rate
                            uint8_t idleRate)                       // Idle rate (0=infinite, >0=4ms units)
{
    RESULT result;

    // Validate parameters
    auto const device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr)
    {
        return ErrorDeviceNumber;
    }
    auto const hidDevice = GetHidDevice(device);
    if (hidDevice == nullptr)
    {
        return ErrorNotHID;
    }

    if (hidIndex >= hidDevice->MaxHID)
    {
        return ErrorIndex; // Invalid HID index
    }

    // Verify that the device has an interrupt IN endpoint
    uint8_t interfaceIndex = hidDevice->HIDInterface[hidIndex];
    auto const endpoint = FindEndpoint(device, interfaceIndex, USB_TRANSFER_TYPE_INTERRUPT, USB_DIRECTION_IN);
    if (endpoint.Header.DescriptorLength == 0) 
    {
        LOG("HID: No interrupt IN endpoint found for device %d, interface %d\n", 
            devNumber, interfaceIndex);
        return ErrorDevice;
    }

    LOG("HID: Enabling interrupt IN for device %d, interface %d, endpoint %d\n",
        devNumber, interfaceIndex, endpoint.EndpointAddress.Number);

    // Step 1: Set protocol if requested
    if (setProtocol) {
        LOG_DEBUG("HID: Setting protocol to %s for device %d\n", 
            protocolValue == 0 ? "boot" : "report", devNumber);
        
        result = HIDSetProtocol(devNumber, interfaceIndex, protocolValue);
        if (result != RESULT::Ok) {
            LOG("HID: Warning - Failed to set protocol for device %d: %d\n", 
                devNumber, result);
            // Continue anyway - some devices don't support SetProtocol
        }
    }

    // Step 2: Set idle rate if requested
    if (setIdleRate) {
        LOG_DEBUG("HID: Setting idle rate to %d (x4ms) for device %d\n", idleRate, devNumber);
        
        // Custom SetIdle implementation with configurable idle rate
        result = HCDSumbitControlMessageOUT(
            device,
            nullptr,
            0,
            UsbDeviceRequest {
                .Type = 0x21,                                       // Host to device, Class, Interface
                .Request = SetIdle,
                .Value = (uint16_t)(idleRate << 8),                // Idle rate in high byte, report ID in low byte (0 for all)
                .Index = interfaceIndex,                           // Interface number
                .Length = 0,
            },
            ControlMessageTimeout,
            nullptr);
            
        if (result != RESULT::Ok) {
            LOG("HID: Warning - Failed to set idle rate for device %d: %d\n", 
                devNumber, result);
            // Continue anyway - some devices don't support SetIdle
        }
    }

    LOG("HID: Interrupt IN enabled for device %d, endpoint %d, interval %dms\n",
        devNumber, endpoint.EndpointAddress.Number, endpoint.Interval);

    return RESULT::Ok;
}

/*- HIDEnableInterruptINSimple ----------------------------------------------
 Simplified version of HIDEnableInterruptIN that uses sensible defaults:
 - Sets report protocol (more feature-rich than boot protocol)
 - Sets idle rate to 0 (only send reports on state change)
 
 This is suitable for most keyboard and mouse applications where you want
 efficient, event-driven input handling.
 --------------------------------------------------------------------------*/
RESULT HIDEnableInterruptINSimple (uint8_t devNumber,               // Device number (address) of the HID device
                                  uint8_t hidIndex)                 // Which HID configuration to enable
{
    return HIDEnableInterruptIN(devNumber, hidIndex, 
                               true,  // Set protocol
                               0,     // boot protocol // Report protocol (more features than boot protocol)
                               true,  // Set idle rate  
                               10);    // Idle rate 0 = only send on change (most efficient)
}
