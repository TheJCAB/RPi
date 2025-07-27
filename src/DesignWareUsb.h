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
#pragma once

#include "UsbSpec.h"
#include "Async.h"

#include <stdint.h>
#include <stddef.h>

#include <array>
#include <memory>
#include <expected>





/***************************************************************************}
{             PUBLIC USB STRUCTURES DEFINITIONS DEFINED BY US                }
****************************************************************************/

/*--------------------------------------------------------------------------}
{     USB pipe our own special structure encompassing a pipe in the USB spec    }
{---------------------------------------------------------------------------}*/
struct PACKED UsbPipe
{
    uint16_t MaxPacketSizeInBytes; // Maximum packet size in bits
    UsbSpeed Speed;               // Speed of device
    uint8_t  EndPoint;            // Endpoint address
    uint8_t  Number;              // Unique device number sometimes called address or id
    uint8_t  splitNodePort;       // In low speed transfers it is port device is on closest parent high speed hub
    uint8_t  splitNodePoint;      // In low speed transfers it is closest parent high speed hub
};


// Operating functions

enum class DWCRESULT : uint8_t
{
    Ok = 0,
    ErrorGeneral      = 1,
    ErrorArgument     = 2,
    ErrorDevice       = 3,
    ErrorIncompatible = 4,
    ErrorTimeout      = 5,
    ErrorTransmission = 6,
    ErrorStall        = 7,
};

class HCDHost;

// Starts the HCD system once completed this routiune the system is operational.
Async::task<std::expected<std::shared_ptr<HCDHost>, DWCRESULT>> HCDInitialize();
