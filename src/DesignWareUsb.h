// Taken from: https://github.com/LdB-ECM/Raspberry-Pi/tree/master/Arm32_64_USB
/***************************************************************}
{  Complete redux of CSUD (Chadderz's Simple USB Driver) by		}
{  Alex Chadwick by Leon de Boer(LdB) 2017, 2018				}
{																}
{  Version 2.0  (AARCH64 & AARCH32 compilation supported)		}
{																}
{  CSUD was overly complex in both it's coding and especially   }
{  implementation for what it actually did. At it's heart CSUD  }
{  simply provides the CONTROL pipe operation of a USB bus.That }
{  provides all the functionality to enumerate the USB bus and  }
{  control devices on the BUS. It is the start point for a real }
{  driver or access layer to the USB.							}
{                                                               }
{******************[ THIS CODE IS FREEWARE ]********************}
{																}
{     This sourcecode is released for the purpose to promote	}
{   programming on the Raspberry Pi. You may redistribute it    }
{   and/or modify with the following disclaimer.                }
{																}
{   The SOURCE CODE is distributed "AS IS" WITHOUT WARRANTIES	}
{   AS TO PERFORMANCE OF MERCHANTABILITY WHETHER EXPRESSED OR   }
{   IMPLIED. Redistributions of source code must retain the     }
{   copyright notices.                                          }
{																}
{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#ifndef _RPI_USB_					// Check RPI_USB guard
#define _RPI_USB_

#include "UsbSpec.h"

#include <array>

#include <stdint.h>




/***************************************************************************}
{             PUBLIC USB STRUCTURES DEFINITIONS DEFINED BY US				}
****************************************************************************/

/*--------------------------------------------------------------------------}
{ 	USB pipe our own special structure encompassing a pipe in the USB spec	}
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) UsbPipe {
    uint16_t MaxPacketSizeInBits;								// Maximum packet size in bits
    UsbSpeed Speed;		    									// Speed of device
    uint8_t EndPoint;   										// Endpoint address
    uint8_t Number; 											// Unique device number sometimes called address or id
    uint8_t lowSpeedNodePort;									// In low speed transfers it is port device is on closest parent high speed hub
    uint8_t lowSpeedNodePoint;									// In low speed transfers it is closest parent high speed hub
};

/*--------------------------------------------------------------------------}
{ 			USB pipe control used mainly by internal routines				}
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) UsbPipeControl {
    unsigned _reserved : 14;										// @0-13	
    enum usb_transfer_type	Type : 2;								// @14-15	Packet type
    unsigned Channel : 8;											// @16-23   Channel to use
    unsigned Direction : 1;											// @24		Direction  1=IN, 0=OUT
    unsigned _reserved1 : 7;										// @25-31	
};

// Finds and reserves an unused DWC USB host channel. This is blocking and
// will wait until a channel is available if all in use.
// RETURN: Index of the free channel
unsigned int dwc_get_free_channel();

// Releases the given DWC USB host channel that was in use and marks as free.
void dwc_release_channel(unsigned int chan);


void DwcClearEnable();
void DwcResume();
void DwcPowerOff();
void DwcConnectionChange();
void DwcEnableChange();
void DwcOverCurrentChange();
void DwcReset();
void DwcPowerOn();
HubPortFullStatus DwcGetPortStatus();

enum class DWCRESULT : uint8_t
{
    Ok = 0,
    ErrorGeneral,
    ErrorArgument,
    ErrorDevice,
    ErrorIncompatible,
    ErrorTimeout,
    ErrorTransmission,
    ErrorStall,
};



// Initialises the hardware that is in use. This usually means powering up that
// hardware and it may therefore need a set delay between this call and  the
// HCDStart routine after which you can use the system.
DWCRESULT HCDInitialise();

// Starts the HCD system once completed this routiune the system is operational.
DWCRESULT HCDStart();

// Sends/recieves data from the given buffer and size directed by pipe settings.
DWCRESULT HCDChannelTransfer(const struct UsbPipe pipe, const UsbPipeControl pipectrl, uint8_t* buffer, uint32_t& bufferLength, PacketId packetId);

#endif						// end RPI_USB guard
