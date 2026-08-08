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
#pragma once

#include <stdint.h>

#include "UsbSpec.h"
#include "UsbDriver.h"


/***************************************************************************}
{          PUBLIC HID 1.11 STRUCTURE DEFINITIONS AS PER THE MANUAL          }
****************************************************************************/

/*--------------------------------------------------------------------------}
{ 					 USB HID 1.11 defined report types						}
{---------------------------------------------------------------------------}*/
enum HidReportType {
    USB_HID_REPORT_TYPE_INPUT = 1,									// Input HID report
    USB_HID_REPORT_TYPE_OUTPUT = 2,									// Output HID report
    USB_HID_REPORT_TYPE_FEATURE = 3,								// Feature HID report
};


/*--------------------------------------------------------------------------}
{						 PUBLIC HID INTERFACE ROUTINES						}
{---------------------------------------------------------------------------}*/

/*- HIDReadDescriptor ------------------------------------------------------
 Reads the HID descriptor from the given device. The call will error if the
 device is not a HID device, you can always check that by the use of IsHID.
 23Mar17 LdB
 --------------------------------------------------------------------------*/
Async::task<RESULT> HIDReadDescriptor(UsbDriver&, uint8_t devNumber,						// Device number (address) of the device to read 
                           uint8_t hidIndex,							// Which hid configuration information is requested from
                          uint8_t* Buffer,							// Pointer to a buffer to receive the descriptor
                          uint16_t Length);							// Maxium length of the buffer 

/*- HIDReadReport ----------------------------------------------------------
 Reads the HID report from the given device. The call will error if device
 is not a HID device, you can always check that by the use of IsHID.
 23Mar17 LdB
 --------------------------------------------------------------------------*/
Async::task<RESULT> HIDReadReport(UsbDriver&, uint8_t devNumber,							// Device number (address) of the device to read
                      uint8_t hidIndex,								// Which hid configuration information is requested from
                      uint16_t reportValue,							// Hi byte = enum HidReportType  Lo Byte = Report Index (0 = default)  
                      std::byte* Buffer,								// Pointer to a buffer to recieve the report
                      uint16_t Length);								// Length of the report

/*- HIDWriteReport ----------------------------------------------------------
 Writes the HID report located in buffer to the given device. This call will 
 error if device is not a HID device, you can always check that by the use of 
 IsHID.
 23Mar17 LdB
 --------------------------------------------------------------------------*/
Async::task<RESULT> HIDWriteReport(UsbDriver&, uint8_t devNumber,							// Device number (address) of the device to write report to
                       uint8_t hidIndex,							// Which hid configuration information is writing to
                       uint16_t reportValue,						// Hi byte = enum HidReportType  Lo Byte = Report Index (0 = default) 
                       std::byte* Buffer,								// Pointer to a buffer containing the report
                       uint16_t Length);							// Length of the report

/*- HIDSetProtocol ----------------------------------------------------------
 Many USB HID devices support multiple low level protocols. For example most
 mice and keyboards have a BIOS Boot mode protocol that makes them look like
 an old DOS keyboard. They also have another protocol which is more advanced.
 This call enables the switch between protocols. What protocols are available
 and what interface is retrieved and parsed from Descriptors from the device.
 23Mar17 LdB
 --------------------------------------------------------------------------*/
Async::task<RESULT> HIDSetProtocol(UsbDriver&, uint8_t devNumber,							// Device number (address) of the device
                       uint8_t interface,							// Interface number to change protocol on
                       uint16_t protocol);							// The protocol number request

Async::task<RESULT> HIDSetIdle(UsbDriver&, uint8_t devNumber, uint8_t hidIndex);

/*- HIDStartInterruptIN -----------------------------------------------------
 Starts an interrupt IN transfer for the specified HID device and endpoint.
 This allows for asynchronous reading of HID reports without polling.
 
 This function replaces the need for constant polling using HIDReadReport.
 Instead of polling every few milliseconds, this function uses the device's
 interrupt IN endpoint to receive data only when the device has new data
 to report (e.g., when a key is pressed on a keyboard).
 
 Note: This is a synchronous call that performs one interrupt transfer.
 For true asynchronous operation, call this function in a loop or timer.
 --------------------------------------------------------------------------*/
Async::task<RESULT> HIDStartInterruptIN(UsbDriver&, uint8_t devNumber,                      // Device number (address) of the HID device
                           uint8_t hidIndex,                        // Which HID configuration to use
                           std::byte* Buffer,                         // Buffer to receive interrupt data
                           uint16_t BufferLength,                   // Length of the buffer
                           uint32_t* BytesTransferred);             // Pointer to store actual bytes transferred

/*- HIDStopInterruptIN ------------------------------------------------------
 Stops an active interrupt IN transfer for the specified HID device.
 --------------------------------------------------------------------------*/
Async::task<RESULT> HIDStopInterruptIN(UsbDriver&, uint8_t devNumber,                       // Device number (address) of the HID device
                          uint8_t hidIndex);                        // Which HID configuration to stop

/*- HIDReadInterruptReport --------------------------------------------------
 Convenience function that performs a single interrupt IN transfer to read
 a HID report. This is useful for applications that want to use interrupt
 mode but still handle transfers synchronously.
 --------------------------------------------------------------------------*/
Async::task<RESULT> HIDReadInterruptReport(UsbDriver&, uint8_t devNumber,                   // Device number (address) of the HID device
                              uint8_t hidIndex,                     // Which HID configuration to use
                              std::byte* Buffer,                      // Buffer to receive the report
                              uint16_t BufferLength,                // Length of the buffer
                              uint32_t* BytesTransferred);         // Pointer to store actual bytes transferred

/*- HIDGetInterruptInterval -------------------------------------------------
 Gets the polling interval for the interrupt IN endpoint of a HID device.
 This value indicates how often the device should be polled for new data.
 --------------------------------------------------------------------------*/
RESULT HIDGetInterruptInterval(UsbDriver&, uint8_t devNumber,                  // Device number (address) of the HID device
                               uint8_t hidIndex,                    // Which HID configuration to use
                               uint8_t* Interval);                 // Pointer to store the interval in milliseconds

/*--------------------------------------------------------------------------}
{                     HID INTERRUPT IN ENABLE FUNCTIONS                    }
{--------------------------------------------------------------------------}*/

/*- HIDEnableInterruptIN ----------------------------------------------------
 Enables interrupt IN communication for a HID device. This function performs
 any necessary setup steps to ensure the device is ready for interrupt IN
 transfers. For most standard HID devices, interrupt IN endpoints are enabled
 automatically after USB configuration, but some devices may require specific
 protocol or idle settings.
 --------------------------------------------------------------------------*/
Async::task<RESULT> HIDEnableInterruptIN(UsbDriver&, uint8_t devNumber,                     // Device number (address) of the HID device
                            uint8_t hidIndex,                       // Which HID configuration to enable
                            bool setProtocol,                       // Whether to set the protocol
                            uint8_t protocolValue,                  // Protocol value (0=boot, 1=report)
                            bool setIdleRate,                       // Whether to set idle rate
                            uint8_t idleRate);                      // Idle rate (0=infinite, >0=4ms units)

/*- HIDEnableInterruptINSimple ----------------------------------------------
 Simplified version of HIDEnableInterruptIN that uses sensible defaults.
 Sets report protocol and idle rate of 0 (only send on state change).
 --------------------------------------------------------------------------*/
Async::task<RESULT> HIDEnableInterruptINSimple(UsbDriver&, uint8_t devNumber,               // Device number (address) of the HID device
                                  uint8_t hidIndex);                // Which HID configuration to enable

