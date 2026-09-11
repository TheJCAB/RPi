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

#include "UsbDriver.h"

#include "UsbSpec.h"

#include <fmt/format.h>

