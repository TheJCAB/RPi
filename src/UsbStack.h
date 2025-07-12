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


enum RESULT : int {
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

namespace USB
{
class Device;
}

struct HidDevice;

#define MaximumDevices 32                                            // Max number of devices with a USB node we will allow 

#define ControlMessageTimeout 10

/***************************************************************************}
{                          PUBLIC INTERFACE ROUTINES                            }
****************************************************************************/

/*--------------------------------------------------------------------------}
{                     PUBLIC GENERIC USB INTERFACE ROUTINES                    }
{---------------------------------------------------------------------------}*/

/*-UsbInitialize-------------------------------------------------------------
 Initializes the USB driver by performing necessary interfactions with the
 host controller driver, and enumerating the initial device tree.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT UsbInitialize ();

struct DeviceHandler
{
    virtual ~DeviceHandler();

    virtual void NewDeviceAdded(USB::Device& device) = 0;
    virtual void DeviceRemoved (USB::Device& device) = 0;
};

void RegisterHidDeviceHandler(DeviceHandler& handler);

/*-UsbGetRootHub ------------------------------------------------------------
 On a Universal Serial Bus, there exists a root hub. This if often a virtual
 device, and typically represents a one port hub, which is the physical
 universal serial bus for this computer. It is always address 1. It is present 
 to allow uniform software manipulation of the universal serial bus itself.
 This will return that FAKE rootHub or NULL on failure. Reason for failure is
 generally not having called USBInitialize to start the USB system.         
 11Apr17 LdB
 --------------------------------------------------------------------------*/
USB::Device& UsbGetRootHub();

/*-UsbDeviceAtAddress -------------------------------------------------------
 Given the unique USB address this will return the pointer to the USB device
 structure. If the address is not actually in use it will return NULL.
 11Apr17 LdB
 --------------------------------------------------------------------------*/
USB::Device& UsbDeviceAtAddress (uint8_t devNumber);

uint32_t   GetDeviceNumber(USB::Device&);

/*-UsbCheckForChange --------------------------------------------------------
 Recursively calls HubCheckConnection on all ports on all hubs connected to
 the root hub. It will hence automatically change the device tree matching
 any physical changes. If we don't have interrupts turned on you will need
 to poll this from time to time.
 10Apr17 LdB
 --------------------------------------------------------------------------*/
void UsbCheckForChanges();

/*-UsbShowTree --------------------------------------------------------------
 Shows the USB tree as ascii art using the Printf command. The normal command
 to show from roothub up is UsbShowTree(UsbGetRootHub(), 1, '+');
 14Mar17 LdB
 --------------------------------------------------------------------------*/
void UsbShowTree (USB::Device& root, const int level, const char tee);
