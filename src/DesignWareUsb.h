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
#include "UsbPipe.h"
#include "Async.h"

#include <stdint.h>
#include <stddef.h>

#include <array>
#include <memory>
#include <expected>





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
