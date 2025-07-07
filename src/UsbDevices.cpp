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
#include "UsbDevices.h"

#include "DesignWareUsb.h"

#include "Cpu.h"
#include "Timer.h"

#include "emb-stdio.h"				// Needed for printf

#include <string.h>
#include <wchar.h>

#include <span>

#define LOG(...)
//#define LOG(...) printf2(__VA_ARGS__)
#define LOG_DEBUG(...)
//#define LOG_DEBUG(...) printf2(__VA_ARGS__)

static RESULT ResultFromDwcResult(DWCRESULT dwcResult)
{
    switch (dwcResult) {
        case DWCRESULT::Ok                 : return RESULT::Ok;
        case DWCRESULT::ErrorGeneral       : return RESULT::ErrorGeneral;
        case DWCRESULT::ErrorArgument      : return RESULT::ErrorArgument;
        case DWCRESULT::ErrorDevice        : return RESULT::ErrorDevice;
        case DWCRESULT::ErrorIncompatible  : return RESULT::ErrorIncompatible;
        case DWCRESULT::ErrorTimeout       : return RESULT::ErrorTimeout;
        case DWCRESULT::ErrorTransmission  : return RESULT::ErrorTransmission;
        case DWCRESULT::ErrorStall         : return RESULT::ErrorStall;
    }
    LOG("Unknown DWCRESULT: %d\n", static_cast<int>(dwcResult));
    return RESULT::ErrorGeneral; // Fallback for unknown results
}

    /**
    \brief The maximum number of children a device could have, by implication, this is
    the maximum number of ports a hub supports.

    This is theoretically 255, as 8 bits are used to transfer the port count in
    a hub descriptor. Practically, no hub has more than 10, so we instead allow
    that many. Increasing this number will waste space, but will not have
    adverse consequences up to 255. Decreasing this number will save a little
    space in the HubDevice structure, at the risk of removing support for an
    otherwise valid hub.
    */
#define MaxChildrenPerDevice 10

    /**
    \brief The maximum number of interfaces a device configuration could have.

    This is theoretically 255 as one byte is used to transfer the interface
    count in a configuration descriptor. In practice this is unlikely, so we
    allow an arbitrary 8. Increasing this number wastes (a lot) of space in
    every device structure, but should not have other consequences up to 255.
    Decreasing this number reduces the overheads of the UsbDevice structure, at
    the cost of possibly rejecting support for an otherwise supportable device.
    */
#define MaxInterfacesPerDevice 8

    /**
    \brief The maximum number of endpoints a device could have (per interface).

    This is theoretically 16, as four bits are used to transfer the endpoint
    number in certain device requests. This is possible in practice, so we
    allow that many. Decreasing this number reduces the space in each device
    structure considerably, while possible removing support for otherwise valid
    devices. This number should not be greater than 16.
    */
#define MaxEndpointsPerDevice 16



/*--------------------------------------------------------------------------}
{ 	USB parent used mainly by internal routines (details of parent hub)		}
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) UsbParent {
    unsigned Number : 8;											// @0	Unique device number of our parent sometimes called address or id
    unsigned PortNumber : 8;										// @8	This is the port we are connected to on our parent hub
    unsigned reserved : 16;											// @16  Reserved 16 bits
};

/*--------------------------------------------------------------------------}
{ 			USB config control used mainly by internal routines				}
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) UsbConfigControl {
    uint8_t ConfigIndex;										// @0 Current set config index
    uint8_t ConfigStringIndex;									// @8 Current config string index
    UsbDeviceStatus Status;     								// @16 Device enumeration status .. USB_ATTACHED, USB_POWERED, USB_ADDRESSED, etc
    uint8_t reserved;											// @24-31
};

/*--------------------------------------------------------------------------}
{	  Forward declare our USB device types which form our device tree		}
{---------------------------------------------------------------------------}*/
struct UsbDevice;			// Single device endpoint
struct HubDevice;			// Hub connects to multiple other devices so we get a tree as well as being an endpoint itself
struct HidDevice;			// Single device endpoint which is a human interface 
struct MassStorageDevice;	// Single device endpoint which is a mass storage device 

/*--------------------------------------------------------------------------}
{	  To a standard USB device we can add a payload this is the type id		}
{---------------------------------------------------------------------------}*/
enum PayLoadType {
    ErrorPayload = 0,								// Device is not even active so can't have a payload							
    NoPayload = 1,									// Device is active but no payload attached
    HubPayload = 2,									// Device has hub payload attached
    HidPayload = 3,									// Device has Hid payload attached
    MassStoragePayload = 4,							// Device has Mass storage payload attached
};

#define ALIGN4 __attribute__((aligned(4)))			// Alignment attribute shortcut macro .. I hate the attribute text length nothing tricky

/*--------------------------------------------------------------------------}
{  Our structure that hold details about any USB device we have detected    }
{---------------------------------------------------------------------------}*/
struct UsbDevice {
    UsbParent ParentHub;						// Details of our parent hub
    UsbPipe Pipe0;							// Usb device pipe AKA pipe0	
    UsbConfigControl Config;					// Usb config control
    uint8_t MaxInterface ALIGN4;					// Maxiumum interface in array (varies with config and usually a lot less than the max array size) 
    UsbInterfaceDescriptor Interfaces[MaxInterfacesPerDevice] ALIGN4; // These are available interfaces on this device
    UsbEndpointDescriptor Endpoints[MaxInterfacesPerDevice][MaxEndpointsPerDevice] ALIGN4; // These are available endpoints on this device
    DeviceDescriptor Descriptor ALIGN4;	// Device descriptor it's accessed a bit so we have a copy to save USB bus ... align it for ARM7/8

    PayLoadType PayLoadId;						// Payload type being carried
    union {											// It can only be any of the different payloads
        HubDevice* HubPayload;				// If this is a USB gateway node of a hub this pointer will be set to the hub data which is about the ports
        HidDevice* HidPayload;				// If this node has a HID function this pointer will be to the HID payload
        MassStorageDevice* MassPayload;		// If this node has a MASS STORAGE function this pointer will be to the Mass Storage payload
    };
};

/*--------------------------------------------------------------------------}
{	 USB hub structure which is just extra data attached to a USB node	    }
{---------------------------------------------------------------------------}*/
struct HubDevice {
    uint32_t MaxChildren;
    UsbDevice *Children[MaxChildrenPerDevice];
    HubDescriptor Descriptor ALIGN4;				// Hub descriptor it's accessed a bit so we have a copy to save USB bus ... align it for ARM7/8
};

/*--------------------------------------------------------------------------}
{	USB mass storage structure which is extra data attached to a USB node   }
{---------------------------------------------------------------------------}*/
struct MassStorageDevice {
    uint8_t SCSI;
};


uint8_t RootHubDeviceNumber = 0;

UsbDevice DeviceTable[MaximumDevices] = { 0 };				// Usb node device allocation table
#define MaximumHubs	16												// Maximum number of HUB payloads we will allow
HubDevice HubTable[MaximumHubs] = { 0 };						// Usb hub device allocation table

/*--------------------------------------------------------------------------}
{			USB2.0 DEVICE DESCRIPTOR BLOCK FOR OUR "FAKED" ROOTHUB 			}
{--------------------------------------------------------------------------*/
alignas(4) struct DeviceDescriptor RootHubDeviceDescriptor = {
    .bLength = sizeof(struct DeviceDescriptor),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = DeviceClassHub,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0,
    .idProduct = 0,
    .bcdDevice = 0x0100,
    .iManufacturer = 0,
    .iProduct = 1,									// String 1 see below .. says "FAKED Root Hub (tm)"
    .iSerialNumber = 0,
    .bNumConfigurations = 1,
};

/*--------------------------------------------------------------------------}
{  Hard-coded configuration descriptor, along with an associated interface  }
{  descriptor and endpoint descriptor, for the "faked" root hub.			}
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__)) RootHubConfig {
    struct ConfigurationDescriptor Configuration;
    struct UsbInterfaceDescriptor Interface;
    struct UsbEndpointDescriptor Endpoint;
};

alignas(4) struct RootHubConfig root_hub_configuration = {
    .Configuration = []() constexpr {
        ConfigurationDescriptor desc = {
            .bLength = sizeof(ConfigurationDescriptor),
            .bDescriptorType = USB_DESCRIPTOR_TYPE_CONFIGURATION,
            .wTotalLength = sizeof(root_hub_configuration),
            .bNumInterfaces = 1,
            .bConfigurationValue = 1,
            .iConfiguration = 2,
            .bMaxPower = 0,
        };
        // These are in unnamed structs/unions, so not reachable via designated initializers.
        desc.RemoteWakeup = false;
        desc.SelfPowered = true;
        desc._reserved7 = 1;
        return desc;
    }(),
    .Interface = {
        .Header = {
            .DescriptorLength = sizeof(struct UsbInterfaceDescriptor),
            .DescriptorType = USB_DESCRIPTOR_TYPE_INTERFACE,
        },
        .Number = 0,
        .AlternateSetting = 0,
        .EndpointCount = 1,
        .Class = InterfaceClass::Hub,
        .SubClass = 0,
        .Protocol = 0,
        .StringIndex = 0,
    },
    .Endpoint = {
        .Header = {
            .DescriptorLength = sizeof(struct UsbEndpointDescriptor),
            .DescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
        },
        .EndpointAddress = {
            .Number = 1,
            .Direction = USB_DIRECTION_IN,
        },
        .Attributes = {
            .Type = USB_TRANSFER_TYPE_INTERRUPT,
        },
        .Packet = {
            .MaxSize = 64,
        },
        .Interval = 0xff,
    },
};

/*--------------------------------------------------------------------------}
{		  USB2.0 DESCRIPTION STRING0 FOR OUR "FAKED" ROOTHUB 				}
{--------------------------------------------------------------------------*/
constexpr char16_t RootHubString0Buffer[] = { char16_t{0x0409} };
alignas(4) constexpr auto RootHubString0 = LiteralUsbStringDescriptor(RootHubString0Buffer);

#define RootHubString u"FAKED Root Hub (tm)"				// UTF string
/*--------------------------------------------------------------------------}
{		  USB2.0 DESCRIPTION STRING1 FOR OUR "FAKED" ROOTHUB				}
{--------------------------------------------------------------------------*/
alignas(4) constexpr auto RootHubString1 = LiteralUsbStringDescriptor(RootHubString);

#define RootHubConfigString u"FAKE config string"				// UTF string
/*--------------------------------------------------------------------------}
{		  USB2.0 DESCRIPTION STRING3 FOR OUR "FAKED" ROOTHUB				}
{--------------------------------------------------------------------------*/
alignas(4) constexpr auto RootHubString2 = LiteralUsbStringDescriptor(RootHubConfigString);

/*--------------------------------------------------------------------------}
{			USB2.0 HUB DESCRIPTION FOR OUR "FAKED" ROOTHUB 					}
{--------------------------------------------------------------------------*/
alignas(4) struct HubDescriptor RootHubDescriptor = {
    .Header = {
        .DescriptorLength = sizeof(struct HubDescriptor),
        .DescriptorType = USB_DESCRIPTOR_TYPE_HUB,
    },
    .PortCount = 1,
    .Attributes = {
        .PowerSwitchingMode = Global,
        .Compound = false,
        .OverCurrentProtection = Global,
        .ThinkTime = 0,
        .Indicators = false,
    },
    .PowerGoodDelay = 0,
    .MaximumHubPower = 0,
    .DeviceRemovable = { .Port1 = true },
    .PortPowerCtrlMask = 0xff,
};

/*==========================================================================}
{	 MY MEMORY COPY .. YEAH I AM OVER THE ARM MEMCOPY ALIGNMENT	ISSUES	    }
{==========================================================================*/
void myMemCopy (uint8_t* dest, uint8_t* source, uint32_t size){
    while (size) {													// While data to copy
        *dest++ = *source++;										// Copy 1 byte from source to dest and increment pointers
        size--;														// Decerement size
    }
}

/*==========================================================================}
{			    INTERNAL FAKE ROOT HUB MESSAGE HANDLER					    }
{==========================================================================*/
RESULT HcdProcessRootHubMessage (uint8_t* buffer, uint32_t bufferLength, struct UsbDeviceRequest *request, uint32_t *bytesTransferred)
{
    RESULT result = Ok;
    uint32_t replyLength = 0;
    union {										// Place a union over these to stop having to mess around .. its a 4 bytes whatever the case .. look carefully
        uint8_t* replyBytes;					// Pointer to bytes to return can be anything 
        uint8_t	reply8;							// 8 bit return
        uint16_t reply16;						// 16 bit return
        uint32_t reply32;						// 32 bit return
        struct HubFullStatus replyHub;			// Hub status return
        struct HubPortFullStatus replyPort;		// Port status return
    } replyBuf;
    bool ptrTransfer = false;					// Default is not a pointer transfer
    switch (request->Request) {
    /* Details on GetStatus from here http://www.beyondlogic.org/usbnutshell/usb6.shtml */
    case GetStatus:
        switch (request->Type) {
        case bmREQ_DEVICE_STATUS  /*0x80*/: 						// Device status request .. returns a 16 bit device status
            replyBuf.reply16 = 1;									// Only two bits in D0 = Self Powered, D1 = Remote Wakeup .. So 1 just self powered 
            replyLength = 2;										// Two byte response
            break;
        case bmREQ_INTERFACE_STATUS /* 0x81 */:						// Interface status request .. returns a 16 bit status
            replyBuf.reply16 = 0;									// Spec says two bytes of 0x00, 0x00. (Both bytes are reserved for future use)
            replyLength = 2;										// Two byte response
        case bmREQ_ENDPOINT_STATUS /* 0x82 */:						// Endpoint status request .. return a 16 bit status 
            replyBuf.reply16 = 0;									// Two bytes indicating the status (Halted/Stalled) of a endpoint. D0 = Stall .. 0 No stall for us 
            replyLength = 2;										// Two byte response
            break;
        case bmREQ_HUB_STATUS  /*0xa0*/:							// We are a hub class so we need a standard hub class get status return
            replyBuf.replyHub.Raw32 = 0;							// Zero all the status bits
            replyBuf.replyHub.Status.LocalPower = true;				// So we will return a HubFullStatus ... Just set LocalPower bit
            replyLength = 4;										// 4 bytes in size .. remember we checked all that in static asserts
            break;
        case bmREQ_PORT_STATUS /* 0xa3 */:							// PORT request .. Remember we have 1 port which is the actual physical hardware
            if (request->Index == 1) {								// Remember we have only one port so any other port is an error
                replyBuf.replyPort = DwcGetPortStatus();
                replyLength = sizeof(replyBuf.replyPort);			// 4 bytes in size .. remember we checked all that in static asserts
            } else result = ErrorArgument;							// Any other port than number 1 means the arguments are garbage
            break;
        default:
            result = ErrorArgument;									// Unknown argument provided on request GetStatus
            break;
        };
        break;
    /* Details on ClearFeature from here http://www.beyondlogic.org/usbnutshell/usb6.shtml */
    case ClearFeature:
        replyLength = 0;
        switch (request->Type) {
        case bmREQ_INTERFACE_FEATURE /*0x01*/:						// Interface clear feature requet
            break;													// Current USB Specification Revision 2 specifies no interface features.
        case bmREQ_ENDPOINT_FEATURE /*0x02*/:						// Endpoint set feature request
            break;													// 16 bits only option is Halt on D0 which we dont support
        case bmREQ_HUB_FEATURE      /*0x20*/:						// Hub clear feature request
            break;													// Only options DEVICE_REMOTE_WAKEUP and TEST_MODE neither which we support
        case  bmREQ_PORT_FEATURE /*0x23*/:							// Port clear feature request
            if (request->Index == 1) {								// Remember we have only one port so any other port is an error
                switch ((enum HubPortFeature)request->Value) {		// Check what request to clear is
                case FeatureEnable:
                    DwcClearEnable();
                    break;
                case FeatureSuspend:
                    DwcResume();
                    break;
                case FeaturePower:
                    LOG("Physical host power off\n");
                    DwcPowerOff();
                    break;
                case FeatureConnectionChange:
                    DwcConnectionChange();
                    break;
                case FeatureEnableChange:
                    DwcEnableChange();
                    break;
                case FeatureOverCurrentChange:
                    DwcOverCurrentChange();
                    break;
                default:
                    break;											// Any other clear feature rtequest just ignore
                }
            } else result = ErrorArgument;							// Any other port than number 1 means the arguments are garbage
            break;
        default:
            result = ErrorArgument;									// If it's not a device/interface/classor endpoint ClearFeature message is garbage
            break;
        }
        break;
    /* Details on SetFeature from here http://www.beyondlogic.org/usbnutshell/usb6.shtml */
    case SetFeature:
        replyLength = 0;
        switch (request->Type) {
        case bmREQ_INTERFACE_FEATURE /*0x01*/:						// Interface set feature requet
            break;													// Current USB Specification Revision 2 specifies no interface features.
        case bmREQ_ENDPOINT_FEATURE /*0x02*/:						// Endpoint set feature request
            break;													// 16 bits only option is Halt on D0 which we dont support
        case bmREQ_HUB_FEATURE      /*0x20*/:						// Hub set feature request
            break;													// 16 bits only options DEVICE_REMOTE_WAKEUP and TEST_MODE neither which we support
        case bmREQ_PORT_FEATURE /* 0x23 */:							// Port set feature request
            if (request->Index == 1) {								// Remember we have only one port so any other port is an error
                switch ((enum HubPortFeature)request->Value) {
                case FeatureReset:			
                    DwcReset();
                    LOG_DEBUG("Reset physical port .. rootHub %i\n", RootHubDeviceNumber);
                    break;
                case FeaturePower:
                    LOG("Physical host power on\n");
                    DwcPowerOn();
                    break;
                default:
                    break;
                }
            } else result = ErrorArgument;							// Any other port than number 1 means the argument are garbage .. remember 1 port on this hub
            break;
        default:
            result = ErrorArgument;									// If it's not a device/interface/class or endpoint SetFeature message is garbage
            break;
        }
        break;
    case SetAddress:
        replyLength = 0;
        RootHubDeviceNumber = request->Value;						// Move the roothub to address requested .. should always be from zero to 1
        break;
    /* Details on GetDescriptor from here http://www.beyondlogic.org/usbnutshell/usb5.shtml#DeviceDescriptors */
    case GetDescriptor:
        replyLength = 0;											// Preset no return data length	
        switch (request->Type) {
        case bmREQ_GET_DEVICE_DESCRIPTOR /*0x80*/:					// Device descriptor request
            switch ((request->Value >> 8) & 0xff) {
            case USB_DESCRIPTOR_TYPE_DEVICE:
                replyLength = sizeof(RootHubDeviceDescriptor);		// Size of our fake hub descriptor
                replyBuf.replyBytes = (uint8_t*)&RootHubDeviceDescriptor;// Pointer to our fake roothub descriptor
                ptrTransfer = true;									// Set pointer transfer flag
                break;
            case USB_DESCRIPTOR_TYPE_CONFIGURATION:
                replyLength = sizeof(root_hub_configuration);		// Size of our fake config descriptor
                replyBuf.replyBytes = (uint8_t*)&root_hub_configuration;// Pointer to our fake roothub configuration
                ptrTransfer = true;									// Set pointer transfer flag
                break;
            case USB_DESCRIPTOR_TYPE_STRING:
                switch (request->Value & 0xff) {
                case 0x0:											
                    replyLength = RootHubString0.Header.DescriptorLength;// Length of string decriptor 0
                    replyBuf.replyBytes = (uint8_t*)&RootHubString0;// Pointer to string 0
                    ptrTransfer = true;								// Set pointer transfer flag
                    break;
                case 0x1:				
                    replyLength = RootHubString1.Header.DescriptorLength;// Length of string descriptor 1
                    replyBuf.replyBytes = (uint8_t*)&RootHubString1;// Pointer to string 1
                    ptrTransfer = true;								// Set pointer transfer flag
                    break;
                case 0x2:											// Return our fake roothub string2				
                    replyLength = RootHubString2.Header.DescriptorLength;// Length of string descriptor 2
                    replyBuf.replyBytes = (uint8_t*)&RootHubString2;// Pointer to string 2
                    ptrTransfer = true;								// Set pointer transfer flag
                    break;
                default:
                    break;
                }
                break;
            default:
                LOG("Unknown get descriptor type %x\n", (request->Value >> 8) & 0xff);
                result = ErrorArgument;								// Unknown get descriptor type
            }
            break;
        case  bmREQ_GET_HUB_DESCRIPTOR /*0xa0*/:					// RootHub descriptor requested
            replyLength = RootHubDescriptor.Header.DescriptorLength;// Length of our descriptor for our fake hub
            replyBuf.replyBytes = (uint8_t*)&RootHubDescriptor;		// Pointer to our fake roothu descriptor
            ptrTransfer = true;										// Set pointer transfer flag
            break;
        default:
            result = ErrorArgument;									// Besides HUB and DEVICE descriptors our fake hub doesn't know
            break;
        }
        break;
    case GetConfiguration:											// Get configuration message
        replyBuf.reply8 = 0x1;										// Return 1 rememeber we only have 1 config so can't be anything else
        replyLength = 1;											// Reply is a byte
        break;
    case SetConfiguration:											// Set configuration message
        replyLength = 0;											// Just ignore it we have 1 fixed config
        break;
    default:														// Any other message is unknown
        result = ErrorArgument;										// Return error with argument
        break;
    }
    if (replyLength > bufferLength) replyLength = bufferLength;		// The buffer length does not have enough room so truncate our respone to fit
    if (ptrTransfer) myMemCopy(&buffer[0], replyBuf.replyBytes, replyLength); // For a pointer transfer replyBuf has the pointer
       else myMemCopy(&buffer[0], (uint8_t*)&replyBuf, replyLength);// Otherwise we want the raw data in the replyBuf
    if (bytesTransferred) *bytesTransferred = replyLength;			// If bytes transferred return requested provide it
    return result;													// Return result
}

/*-HCDSumbitControlMessage --------------------------------------------------
 Sends a control message to a device. Handles all necessary channel creation
 and other processing. The sequence of a control transfer is defined in the
 USB 2.0 manual section 5.5.  Success is indicated by return of Ok (0) all
 other codes indicate an error.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT HCDSumbitControlMessage (UsbDevice* device,
                                UsbPipeControl const& pipectrl,// Pipe control structure 					
                                uint8_t* buffer,					// Data buffer both send and recieve				 
                                uint32_t bufferLength,				// Buffer length for send or recieve
                                UsbDeviceRequest&& request,	// USB request message
                                uint32_t timeout,					// Timeout in microseconds on message
                                uint32_t* bytesTransferred)			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
{
    DWCRESULT result;

    LOG_DEBUG("HCD: SETUP packet to device: %#x req: %#x req Type: %#x Speed: %i PacketSize: %i LowNode: %i LowPort: %i\n",
        device->Pipe0.Number, request.Request, request.Type, device->Pipe0.Speed, device->Pipe0.MaxPacketSizeInBits, device->Pipe0.splitNodePoint, device->Pipe0.splitNodePort);

    if (device->Pipe0.Number == RootHubDeviceNumber) {
        return HcdProcessRootHubMessage(buffer, bufferLength, &request, bytesTransferred);
    }
    uint32_t lastTransfer = 0;

    LOG_DEBUG("Setup phase\n");
    // Setup phase
    UsbPipeControl intPipeCtrl = pipectrl;					// Copy the pipe control (We want channel really)										
    intPipeCtrl.Type = USB_TRANSFER_TYPE_CONTROL;					// Set pipe to control	
    intPipeCtrl.Direction = USB_DIRECTION_OUT;						// Set pipe to out
    uint32_t transferLength = 8;
    if ((result = HCDChannelTransfer(device->Pipe0, intPipeCtrl,
        (uint8_t*)&request, transferLength, USB_PID_SETUP)) != DWCRESULT::Ok) {				// Send the 8 byte setup request packet
        LOG("HCD: SETUP packet to device: %#x req: %#x req Type: %#x Speed: %i PacketSize: %i LowNode: %i LowPort: %i Error: %i\n",
            device->Pipe0.Number, request.Request, request.Type, device->Pipe0.Speed, device->Pipe0.MaxPacketSizeInBits, device->Pipe0.splitNodePoint, device->Pipe0.splitNodePort, result);// Some parameter issue
        return ResultFromDwcResult(result);
    }
    LOG_DEBUG("Transfer phase\n");
    // Data transfer phase
    if (buffer != NULL) {											// Buffer must be valid for any transfer to occur
        intPipeCtrl.Direction = pipectrl.Direction;					// Set pipe direction as requested	
        lastTransfer = bufferLength;
        if ((result = HCDChannelTransfer(device->Pipe0, intPipeCtrl,
            buffer,	lastTransfer, USB_PID_DATA1)) != DWCRESULT::Ok) {		// Send or recieve the data
            LOG("HCD: Could not transfer DATA to device %i.\n",
                device->Pipe0.Number);										// Log error
        return ResultFromDwcResult(result);
        }
    }

    LOG_DEBUG("Status phase\n");
    // Status phase		
    intPipeCtrl.Direction = ((bufferLength == 0) || pipectrl.Direction == USB_DIRECTION_OUT) ? USB_DIRECTION_IN : USB_DIRECTION_OUT;
    transferLength = 0;
    if ((result = HCDChannelTransfer(device->Pipe0, intPipeCtrl, buffer, transferLength, USB_PID_DATA1)) != DWCRESULT::Ok)	// Send or recieve the status
    {
        LOG("HCD: Could not transfer STATUS to device %i.\n",
            device->Pipe0.Number);											// Log error
        return ResultFromDwcResult(result);
    }
    //if ((*DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel]).size != 0)
    //	LOG_DEBUG("HCD: Warning non zero status transfer! %u.\n", (*DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel]).size);

    if (bytesTransferred) *bytesTransferred = lastTransfer;
    //LOG("\n");
    return Ok;
}

RESULT HCDSumbitControlMessageOUT(
    UsbDevice* device,
    uint8_t* buffer,					// Data buffer both send and recieve				 
    uint32_t bufferLength,				// Buffer length for send or recieve
    UsbDeviceRequest&& request,	// USB request message
    uint32_t timeout,					// Timeout in microseconds on message
    uint32_t* bytesTransferred			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
)
{
    UsbPipeControl const pipectrl
    {
        .Type      = USB_TRANSFER_TYPE_CONTROL,
        .Channel   = dwc_get_free_channel(),
        .Direction = USB_DIRECTION_OUT,
    };
    RESULT result = HCDSumbitControlMessage(
        device,
        pipectrl,
        buffer,
        bufferLength,
        std::move(request),
        timeout,
        bytesTransferred
    );
    dwc_release_channel(pipectrl.Channel);
    return result;
}

RESULT HCDSumbitControlMessageIN(
    UsbDevice* device,
    uint8_t* buffer,					// Data buffer both send and recieve				 
    uint32_t bufferLength,				// Buffer length for send or recieve
    UsbDeviceRequest&& request,	// USB request message
    uint32_t timeout,					// Timeout in microseconds on message
    uint32_t* bytesTransferred			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
)
{
    UsbPipeControl const pipectrl
    {
        .Type      = USB_TRANSFER_TYPE_CONTROL,
        .Channel   = dwc_get_free_channel(),
        .Direction = USB_DIRECTION_IN,
    };
    RESULT result = HCDSumbitControlMessage(
        device,
        pipectrl,
        buffer,
        bufferLength,
        std::move(request),
        timeout,
        bytesTransferred
    );
    dwc_release_channel(pipectrl.Channel);
    return result;
}


/*-HCDSetAddress ------------------------------------------------------------
 Sets the address of the device with control endpoint given by the pipe. Zero
 is a restricted address for the rootHub and will return if attempted.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT HCDSetAddress (UsbDevice* device,
                      uint8_t channel,								// Channel to use
                      uint8_t address)								// Address to set
{
    RESULT result;
    UsbPipeControl pipectrl = {
        .Type = USB_TRANSFER_TYPE_CONTROL,							// Control packet
        .Channel = channel,											// Use given channel channel
        .Direction = USB_DIRECTION_OUT,								// We are writing to host
    };
    if (address == 0) return ErrorArgument;							// You can't set address zero that is strictly reserved for roothub
    result = HCDSumbitControlMessage(
        device,														// Pipe which points to current device endpoint
        pipectrl,													// Pipe control
        NULL,														// No data its a command
        0,															// Zero size transfer as no data
        UsbDeviceRequest {
            .Type = 0,
            .Request = SetAddress,									// Set address request
            .Value = address,										// Address to set
        },
        ControlMessageTimeout, NULL);
    return result;													// Return the result
}

/*-INTERNAL: HCDSetConfiguration---------------------------------------------
 Sets a given USB device configuration to the config index number requested.
 28Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT HCDSetConfiguration (UsbDevice* device, uint8_t channel, uint8_t configuration) {
    RESULT result;
    UsbPipeControl pipectrl = {
        .Type = USB_TRANSFER_TYPE_CONTROL,							// Control packet
        .Channel = channel,											// Use given channel
        .Direction = USB_DIRECTION_OUT,								// We are writing to host
    };
    result = HCDSumbitControlMessage(
        device,
        pipectrl,
        NULL,
        0,
        UsbDeviceRequest {
            .Type = 0,
            .Request = SetConfiguration,							// Set configuration
            .Value = configuration,									// Config index
        },
        ControlMessageTimeout,
        NULL);														// Read the requested configuration
    return result;													// Return result
}

/*==========================================================================}
{		 INTERNAL HCD MESSAGE ROUTINES SPECIFICALLY FOR HUB DEVICES		    }
{==========================================================================*/

/*-INTERNAL: HCDReadHubPortStatus--------------------------------------------
 Reads the given port status on a hub device. Port input is index 1 and so
 requesting port 0 is interpretted as you want the port gateway node status.
 When reading a port the return is really a HubPortFullStatus, while for
 port = 0 the return will be a struct HubFullStatus. There are uint32_t unions
 on those two structures to pass the raw 32 bits in/out.
 21Mar17 LdB
 --------------------------------------------------------------------------*/
RESULT HCDReadHubPortStatus (UsbDevice* device,
                             uint8_t port,							// Port to get status  OR  0 = Gateway node
                             uint8_t *Status)						// HubPortFullStatus or HubFullStatus .. use Raw union  
{
    RESULT result;
    uint32_t transfer = 0;
    UsbPipeControl pipectrl = {
        .Type = USB_TRANSFER_TYPE_CONTROL,							// Control packet
        .Channel = dwc_get_free_channel(),							// Find first free channel
        .Direction = USB_DIRECTION_IN,								// We are reading to host
    };
    if (Status == NULL) return ErrorArgument;						// Make sure return pointer is valid
    if ((result = HCDSumbitControlMessage(
        device,														// Pass control pipe thru unchanged
        pipectrl,
        (uint8_t*)Status,											// Pass in pointer to status
        sizeof(uint32_t),											// We want full structure for either call which is 32 bits
        UsbDeviceRequest {								// Construct a USB request
            .Type = port ? bmREQ_PORT_STATUS : bmREQ_HUB_STATUS,	// Request bit mask is for hub if port = 0, hub port otherwise 
            .Request = GetStatus,									// Get status id
            .Index = port,											// Port number is index 1 so we add one
            .Length = sizeof(uint32_t),								// We want full structure size
        },
        ControlMessageTimeout,										// Standard control message timeouts
        &transfer)) != Ok)											// We will check transfer size so pass in pointer to our local
    {
        dwc_release_channel(pipectrl.Channel);						// Release the channel
        LOG("HCD Hub read status failed on device: %i, port: %i, Result: %#x, Pipe Speed: %#x, Pipe MaxPacket: %u\n",
            device->Pipe0.Number, port, result, device->Pipe0.Speed, device->Pipe0.MaxPacketSizeInBits);	// Log any error
        return result;												// Return error result
    }
    dwc_release_channel(pipectrl.Channel);							// Release the channel
    if (transfer < sizeof(uint32_t)) {								// Hub did not read amount requested
        LOG("HUB: Failed to read hub device:%i port:%i status\n",
            device->Pipe0.Number, port);										// Log error
        return ErrorDevice;											// Some quirk in enumeration usually
    }
    return Ok;														// Return success
}

/*-INTERNAL: HCDChangeHubPortFeature-----------------------------------------
 Changes a feature setting on the given port on a hub device. Port input is
 index 1 and so requesting port 0 is interpretted as you are changing the
 feature on the port gateway node.
 21Mar17 LdB
 --------------------------------------------------------------------------*/
RESULT HCDChangeHubPortFeature (UsbDevice* device,
                                enum HubPortFeature feature,		// Which feature to change
                                uint8_t port,						// Port to change feature  OR  0 = Gateway node
                                bool set)							// Set or clear the feature
{
    RESULT result;
    UsbPipeControl pipectrl = {
        .Type = USB_TRANSFER_TYPE_CONTROL,							// Control packet
        .Channel = dwc_get_free_channel(),							// Find first free channel
        .Direction = USB_DIRECTION_OUT,								// We are writing to host
    };
    if ((result = HCDSumbitControlMessage(
        device,														// Pipe settings passed thru as is
        pipectrl,
        NULL,														// No buffer as no data
        0,															// Length zero as no data
        UsbDeviceRequest {
            .Type = port ? bmREQ_PORT_FEATURE : bmREQ_HUB_FEATURE,	// Request bit mask is for hub if port = 0, hub port otherwise
            .Request = set ? SetFeature : ClearFeature,				// Set or clear feature as requested
            .Value = (uint16_t)feature,								// Feature we are changing
            .Index = port,											// Port (index 1 so add one)
        },
        ControlMessageTimeout,										// Standard control message timeouts
        NULL)) != Ok)												// Ignore transfer pointer as zero data
    {
        dwc_release_channel(pipectrl.Channel);						// Release the channel
        LOG("HUB: Failed to change port feature for device: %i, Port:%d feature:%d set:%d.\n",
            device->Pipe0.Number, port, feature, set);						// Log any error
        return result;												// Return error result
    }
    dwc_release_channel(pipectrl.Channel);							// Release the channel
    return Ok;														// Return success
}


/*==========================================================================}
{      INTERNAL FUNCTIONS THAT OPERATE TO GET DESCRIPTORS FROM DEVICES	    }
{==========================================================================*/

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
RESULT HCDReadStringDescriptor (UsbDevice* device,
                                uint8_t stringIndex,				// String index to be returned
                                char* buffer,						// Pointer to a buffer
                                size_t& length)					// The size of that buffer
{
    RESULT result;
    uint32_t transfer = 0;
    struct UsbDescriptorHeader Header __attribute__((aligned(4)));	// aligned for DMA transfer a discriptor header is two bytes
    char descBuffer[256] __attribute__((aligned(4)));				// aligned for DMA transfer a descriptor is max 256 bytes (uint8_t size in header definition)
    uint16_t langIds[96] __attribute__((aligned(4))) = { 0 };		// aligned for DMA transfer a descriptors
    bool NoEnglishSupport = false;									// Preset no english support false

    if (buffer == NULL || stringIndex == 0) return ErrorArgument;	// Make sure values valid
    result = HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_STRING, 0, 0, &langIds, 2,
        bmREQ_GET_DEVICE_DESCRIPTOR, &transfer, true);				// Get language support header
    if ((result != RESULT::Ok) && (transfer < 2)) {							// Could not read language support data
        LOG("HCD: Could not read language support for device: %i\n",
            device->Pipe0.Number);											// Log the error
        return ErrorArgument;										// I am lost what is going on bail
    }

    // langIds 0 actually has 0x03 (string descriptor) and size of language support words .. if it doesn't bail
    if ((langIds[0] >> 8) != 0x03) {								// The top byte has to be 0x03
        LOG("HCD: Not a valid language support descriptor on device: %i\n",
            device->Pipe0.Number);											// Log the error
        return ErrorArgument;										// I am lost what is going on bail
    }
    // So we have size to read for all the language support pairs
    result = HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_STRING, 0, 0, &langIds, langIds[0] & 0xFF,
        bmREQ_GET_DEVICE_DESCRIPTOR, &transfer, true);				// Get all language support pair data
    if ((result != RESULT::Ok) && (transfer < (langIds[0] & 0xFF))) {		// We failed to read all the support data
        LOG("HCD: Could not read all the language support data on device: %i\n",
            device->Pipe0.Number);											// Log the error		
        return ErrorArgument;										// I am lost what is going on bail
    }

    // Okay lets see if 0x409 is supported .. Sorry I am only interested in english
    // Non speaking people feel free to choose you own language id for your language 
    int i;
    int lastEntry = (langIds[0] & 0xFF) >> 1;						// So from header size we can work last pair entry 
    for (i = 1; i < lastEntry; i++) {								// Remember langIds[0] is header so start at 1
        if (langIds[i] == 0x409) break;								// English id pair exists yipee
    }
    if (i == lastEntry) {											// No search all pairs no english support available
        LOG("No english language string available on device: %i\n",
            device->Pipe0.Number);											// Log the error
        NoEnglishSupport = true;									// Set that flag
    }

    // Pull header of string descriptor so we get size. If no english available use lang pair at position 1
    // We have to read string descriptor for enumeration .. but we don't have to put it in buffer
    result = HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_STRING, stringIndex,
        NoEnglishSupport ? langIds[1] : 0x409, &Header,
        sizeof(struct UsbDescriptorHeader), bmREQ_GET_DEVICE_DESCRIPTOR, 
        &transfer, true);											// Read string descriptor header only
    if ((result != RESULT::Ok) || (transfer != sizeof(struct UsbDescriptorHeader))) {
        LOG("HCD: Could not fetch string descriptor header (%i) for device: %i\n",
            stringIndex, device->Pipe0.Number);								// Log the error
        return ErrorDevice;											// No idea what problem is so bail										
    }

    // Okay we got the size of the string so now read the entire size
    result = HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_STRING, stringIndex,
        NoEnglishSupport ? langIds[1] : 0x409, &descBuffer,
        Header.DescriptorLength, bmREQ_GET_DEVICE_DESCRIPTOR, 
        &transfer, true);											// Read the full string 	
    if ((result != RESULT::Ok) || (transfer != Header.DescriptorLength)) {
        LOG("HCD: Could not fetch string descriptor (%i) for device: %i\n",
            stringIndex, device->Pipe0.Number);								// Log the error
        return ErrorArgument;										// No idea what problem is so bail
    }

    // Finally we need to turn the UTF16 string back to ascii for caller
    i = 0;															// Set i to zero in case no english support
    if (NoEnglishSupport == false) {								// Yipee we have english support				
        uint16_t* p = (uint16_t*)&descBuffer[2];					// Start of unicode text .. 2 bytes at top are descriptor header
        for (i = 0; i < ((Header.DescriptorLength - 2) >> 1)
            && (i < length - 1); i++) buffer[i] = wctob(*p++);		// Narrow character from unicode to ascii
        length = i;
    }
    buffer[i] = '\0';												// Make asciiz

    return Ok;														// Return success
}

/*==========================================================================}
{      INTERNAL FUNCTIONS THAT ADD AND REMOCE HID PAYLOADS TO DEVICES	    }
{==========================================================================*/

HidDevice* AllocateHidPayload();
void FreeHidPayload(HidDevice* device);
uint8_t GetHidCount(HidDevice* device);
bool SetHidDescriptor(HidDevice* hidDevice, uint8_t hidIndex, uint8_t interface, uint8_t const* buffer, uint8_t size);
void PrintHid(HidDevice* device, uint8_t hidIndex, char const* indent);

/*-INTERNAL: AddHidPayload---------------------------------------------------
 Makes sure the device has no other sorts of payload AKA it's simple node
 and if so will find the first free hid storage area and attach it as a hid
 payload.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT AddHidPayload (UsbDevice *device)
{
    if (device == nullptr || device->PayLoadId != NoPayload)
    {
        return ErrorArgument;
    }

    device->HidPayload = AllocateHidPayload();
    if (device->HidPayload == nullptr)
    {
        return ErrorMemory;
    }

    device->PayLoadId = HidPayload;
    return Ok;
}

/*-INTERNAL: RemoveHidPayload------------------------------------------------
 Makes sure the hid payload is free from device will make it free again in the
 hid table to be allocated again.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
void RemoveHidPayload(UsbDevice *device)
{
    if (device && device->PayLoadId == HidPayload && device->HidPayload != nullptr)
    {
        FreeHidPayload(device->HidPayload);
        device->HidPayload = nullptr;
        device->PayLoadId = NoPayload;
    }
}

/*==========================================================================}
{      INTERNAL FUNCTIONS THAT ADD AND REMOCE HUB PAYLOADS TO DEVICES	    }
{==========================================================================*/

/*-INTERNAL: AddHubPayload---------------------------------------------------
 Makes sure the device has no other sorts of payload AKA it's simple node
 and if so will find the first free hub storage area and attach it as a hub
 payload.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT AddHubPayload(struct UsbDevice *device) {
    if (device && device->PayLoadId == NoPayload) {					// Check device is valid and not already assigned a payload
        for (int number = 0; number < MaximumHubs; number++) {		// Search each entry in hub data payload array
            if (HubTable[number].MaxChildren == 0) {				// Find first free entry
                device->HubPayload = &HubTable[number];				// Place pointer to the device payload pointer
                device->PayLoadId = HubPayload;						// Set the payload id
                HubTable[number].MaxChildren = MaxChildrenPerDevice;// Max children starts out as set by us (hub may shorten up itself) .. non zero means entry in use
                return Ok;											// Return success
            }
        }
        return ErrorMemory;											// Too many hubs ... no free hub table entries 
    }
    return ErrorArgument;											// Passed an invalid device ... programming error 
}

/*-INTERNAL: RemoveHubPayload------------------------------------------------
 Makes sure the hub payload is free of all children and then clears payload
 which will make it free again in the hub table to be allocated again.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
void UsbDeallocateDevice(struct UsbDevice *device);					// UsbDeallocate and RemoveHubPayload call each other so we need a forward declare
void RemoveHubPayload(struct UsbDevice *device) {
    if (device && device->PayLoadId == HubPayload && device->HubPayload) {// Check device is valid, is assigned a hub payload and the hubpayload is valid
        for (int i = 0; i < device->HubPayload->MaxChildren; i++) {	// Check each of the children (we would hope already done but check)
            if (device->HubPayload->Children[i])					// If a child is valid
                UsbDeallocateDevice(device->HubPayload->Children[i]);// Any valid children need to be deallocated
        }
        memset(device->HubPayload, 0, sizeof(struct HubDevice));	// Clear all the hub payload data which will mark it unused
        device->HubPayload = NULL;									// Payload removed from device
        device->PayLoadId = NoPayload;								// Clear payload ID its gone
    }
}

/*==========================================================================}
{       INTERNAL FUNCTIONS THAT ADD/DETACH AND DEALLOCATE DEVICES		    }
{==========================================================================*/

/*-INTERNAL: UsbAllocateDevice-----------------------------------------------
 Find first free device entry table and return that pointer as our device.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT UsbAllocateDevice(struct UsbDevice **device) {
    if (device) {
        for (int number = 0; number < MaximumDevices; number++) {	// Search device table entries
            if (DeviceTable[number].PayLoadId == 0) {				// Find first free entry (PayloadId goes to non zero when in use)
                *device = &DeviceTable[number];						// Return that entry area as device
                (*device)->Pipe0.Number = number + 1;				// Our device Id is the table entry we found
                (*device)->Config.Status = USB_STATUS_ATTACHED;		// Set status to attached
                (*device)->ParentHub.PortNumber = 0;				// Start on port 0
                (*device)->ParentHub.Number = 0xFF;					// At this stage we have no parent
                (*device)->PayLoadId = NoPayload;					// Set PayLoadId to no payload attached (PayloadId goes non zero indicating in use)
                (*device)->HubPayload = NULL;						// Make sure payload pointer is NULL
                return Ok;											// Return success
            }
        }
        return ErrorMemory;											// All device table entries are in use .. no free table
    }
    return ErrorArgument;											// The device pointer was invalid .. serious programming error								
}

/*-INTERNAL: UsbDeallocateDevice---------------------------------------------
 Deallocate a device releasing all memory associated to the device
 11Feb17 LdB
 --------------------------------------------------------------------------*/
void UsbDeallocateDevice (struct UsbDevice *device) {
    if (IsHub(device->Pipe0.Number)) {								// If this device is a hub we will need to deal with the children
        /* A hub must deallocate all its children first */
        for (int i = 0; i < device->HubPayload->MaxChildren; i++) {	// For each child
            if (device->HubPayload->Children[i] != NULL)			// If that child is valid
                UsbDeallocateDevice(device->HubPayload->Children[i]);// Iterate deallocating each child
        }
        RemoveHubPayload(device);									// Having disposed of the children we need to get rid of the hub payload	
    }
    if (device->ParentHub.Number < MaximumDevices) {				// Check we have a valid parent
        struct UsbDevice* parent;
        parent = &DeviceTable[device->ParentHub.Number-1];			// Fetch the parent hub device
        /* Now remove this device from any parent .. check everything to make sure it is a child */
        if (parent->PayLoadId == HubPayload && parent->HubPayload &&// Check we have a valid parent and it is a hub
            device->ParentHub.PortNumber < parent->HubPayload->MaxChildren && // Check we are on a valid port
            parent->HubPayload->Children[device->ParentHub.PortNumber] == device)// Check we are the child pointer on that port
            parent->HubPayload->Children[device->ParentHub.PortNumber] = NULL;// Yes we really are the child so clear our entry
    }
    memset(device, 0, sizeof(struct UsbDevice));					// Clear the device entry area which will mark it unused
}

/*==========================================================================}
{			    NON HCD INTERNAL HUB FUNCTIONS ON PORTS						}
{==========================================================================*/
RESULT HubPortReset(struct UsbDevice *device, uint8_t port) {
    RESULT result;
    struct HubPortFullStatus portStatus;
    uint32_t retry, timeout;
    if (!IsHub(device->Pipe0.Number)) return ErrorDevice;			// If device is not a hub then bail
    LOG_DEBUG("HUB: Reseting device: %u Port: %u. source: %i\n", device->Pipe0.Number, port, 0/*source*/);
    for (retry = 0; retry < 3; retry++) {
        if ((result = HCDChangeHubPortFeature(device,
            FeatureReset, port + 1, true)) != Ok) 					// Issue a setfeature of reset
        {
            LOG("HUB: Device %i Failed to reset Port%d.\n",
                device->Pipe0.Number, port + 1);					// Log any failure
            return result;											// Return result that is causing failure
        }
        timeout = 0;
        do {
            Cpu::DelayInMicroseconds(20000);
            if ((result = HCDReadHubPortStatus(device, port + 1, (uint8_t*)&portStatus.Raw32)) != Ok) {
                LOG("HUB: Hub failed to get status (4) for %s.Port%d.\n", UsbGetDescription(device), port + 1);
                return result;
            }
            timeout++;
        } while (!portStatus.Change.ResetChanged && !portStatus.Status.Enabled && timeout < 10);

        if (timeout == 10) continue;

        LOG_DEBUG("HUB: %s.Port%d Status %x:%x.\n", UsbGetDescription(device), port + 1, portStatus.RawStatus, portStatus.RawChange);

        if (portStatus.Change.ConnectedChanged || !portStatus.Status.Connected)
            return ErrorDevice;

        if (portStatus.Status.Enabled)
            break;
    }

    if (retry == 3) {
        LOG("HUB: Cannot enable %s.Port%d. Please verify the hardware is working.\n", UsbGetDescription(device), port + 1);
        return ErrorDevice;
    }

    if ((result = HCDChangeHubPortFeature(device, FeatureResetChange, port + 1, false)) != Ok) {
        LOG("HUB: Failed to clear reset on %s.Port%d.\n", UsbGetDescription(device), port + 1);
    }
    return Ok;
}

/*-INTERNAL: HubPortConnectionChanged ---------------------------------------
 If a connection on a port on a hub as changed this routine is called to deal
 with the change. This will involve it enumerating an added new device or the
 deallocation of a removed or detached device.
 21Mar17 LdB
 --------------------------------------------------------------------------*/
RESULT EnumerateDevice (struct UsbDevice *device, struct UsbDevice* ParentHub, uint8_t PortNum); // We need to forward declare
RESULT HubPortConnectionChanged(struct UsbDevice *device, uint8_t port) {
    RESULT result;
    struct HubDevice *data;
    struct HubPortFullStatus portStatus;
    if (!IsHub(device->Pipe0.Number)) return ErrorDevice;

    data = device->HubPayload;

    if ((result = HCDReadHubPortStatus(device, port + 1, (uint8_t*)&portStatus.Raw32)) != Ok) {
        LOG("HUB: Hub failed to get status (2) for %s.Port%d.\n", UsbGetDescription(device), port + 1);
        return result;
    }
    LOG_DEBUG("HUB: %s.Port%d Status %x:%x.\n", UsbGetDescription(device), port + 1, portStatus.RawStatus, portStatus.RawChange);

    if ((result = HCDChangeHubPortFeature(device, FeatureConnectionChange, port + 1, false)) != Ok) {
        LOG("HUB: Failed to clear change on %s.Port%d.\n", UsbGetDescription(device), port + 1);
    }

    if ((!portStatus.Status.Connected && !portStatus.Status.Enabled) || data->Children[port] != NULL) {
        LOG("HUB: Disconnected %s.Port%d - %s.\n", UsbGetDescription(device), port + 1, UsbGetDescription(data->Children[port]));
        UsbDeallocateDevice(data->Children[port]);
        data->Children[port] = NULL;
        if (!portStatus.Status.Connected) return Ok;
    }

    if ((result = HubPortReset(device, port)) != Ok) {
        LOG("HUB: Could not reset %s.Port%d for new device.\n", UsbGetDescription(device), port + 1);
        return result;
    }

    if ((result = UsbAllocateDevice(&data->Children[port])) != Ok) {
        LOG("HUB: Could not allocate a new device entry for %s.Port%d.\n", UsbGetDescription(device), port + 1);
        return result;
    }

    if ((result = HCDReadHubPortStatus(device, port + 1, (uint8_t*)&portStatus.Raw32)) != Ok) {
        LOG("HUB: Hub failed to get status (3) for %s.Port%d.\n", UsbGetDescription(device), port + 1);
        return result;
    }

    LOG("HUB: %s. Device:%i Port:%d Status %04x:%04x.\n", UsbGetDescription(device), device->Pipe0.Number, port, portStatus.RawStatus, portStatus.RawChange);

    if (portStatus.Status.HighSpeedAttatched)
    {
        data->Children[port]->Pipe0.Speed = USB_SPEED_HIGH;
    }
    else if (portStatus.Status.LowSpeedAttatched)
    {
        data->Children[port]->Pipe0.Speed = USB_SPEED_LOW;
        data->Children[port]->Pipe0.splitNodePoint = device->Pipe0.Number;
        data->Children[port]->Pipe0.splitNodePort = port;
    }
    else
    {
        data->Children[port]->Pipe0.Speed = USB_SPEED_FULL;
        data->Children[port]->Pipe0.splitNodePoint = device->Pipe0.Number;
        data->Children[port]->Pipe0.splitNodePort = port;
    }
    data->Children[port]->ParentHub.Number = device->Pipe0.Number;
    data->Children[port]->ParentHub.PortNumber = port;
    if ((result = EnumerateDevice(data->Children[port], device, port)) != Ok) {
        LOG("HUB: Could not connect to new device in %s.Port%d. Disabling.\n", UsbGetDescription(device), port + 1);
        UsbDeallocateDevice(data->Children[port]);
        data->Children[port] = NULL;
        if (HCDChangeHubPortFeature(device, FeatureEnable, port + 1, false) != Ok) {
            LOG("HUB: Failed to disable %s.Port%d.\n", UsbGetDescription(device), port + 1);
        }
        return result;
    }
    return Ok;
}


/*-HubCheckConnection -------------------------------------------------------
 Checks device is a hub and if a valid hub checks connection status of given
 port on the hub. If it has changed performs necessary actions such as the
 enumerating of a new device or deallocating an old one.
 10Apr17 LdB
 --------------------------------------------------------------------------*/
RESULT HubCheckConnection(struct UsbDevice *device, uint8_t port) {
    RESULT result;
    struct HubPortFullStatus portStatus;
    struct HubDevice *data;

    if (!IsHub(device->Pipe0.Number)) return ErrorDevice;
    data = device->HubPayload;

    if ((result = HCDReadHubPortStatus(device, port + 1, (uint8_t*)&portStatus.Raw32)) != Ok) {
        if (result != ErrorDisconnected)
            LOG("HUB: Failed to get hub port status (1) for %s.Port%d.\n", UsbGetDescription(device), port + 1);
        return result;
    }

    if (portStatus.Change.ConnectedChanged) {
        LOG_DEBUG("Device %i, Port: %i changed\n", device->Pipe0.Number, port);
        HubPortConnectionChanged(device, port);
    }

    if (portStatus.Change.EnabledChanged) {
        if (HCDChangeHubPortFeature(device, FeatureEnableChange, port + 1, false) != Ok) {
            LOG("HUB: Failed to clear enable change %s.Port%d.\n", UsbGetDescription(device), port + 1);
        }

        // This may indicate EM interference.
        if (!portStatus.Status.Enabled && portStatus.Status.Connected && data->Children[port] != NULL) {
            LOG("HUB: %s.Port%d has been disabled, but is connected. This can be cause by interference. Reenabling!\n", UsbGetDescription(device), port + 1);
            HubPortConnectionChanged(device, port);
        }
    }

    if (portStatus.Status.Suspended) {
        if (HCDChangeHubPortFeature(device, FeatureSuspend, port + 1, false) != Ok) {
            LOG("HUB: Failed to clear suspended port - %s.Port%d.\n", UsbGetDescription(device), port + 1);
        }
    }

    if (portStatus.Change.OverCurrentChanged) {
        if (HCDChangeHubPortFeature(device, FeatureOverCurrentChange, port + 1, false) != Ok) {
            LOG("HUB: Failed to clear over current port - %s.Port%d.\n", UsbGetDescription(device), port + 1);
        }
    }

    if (portStatus.Change.ResetChanged) {
        if (HCDChangeHubPortFeature(device, FeatureResetChange, port + 1, false) != Ok) {
            LOG("HUB: Failed to clear reset port - %s.Port%d.\n", UsbGetDescription(device), port + 1);
        }
    }

    return Ok;
}

/*-INTERNAL: HubCheckForChange ----------------------------------------------
 This performs an iteration loop to check each port on each hub to see if any
 device has been added or removed.
 21Mar17 LdB
 --------------------------------------------------------------------------*/
void HubCheckForChange(struct UsbDevice *device) {
    if (IsHub(device->Pipe0.Number)) {
        for (int i = 0; i < device->HubPayload->MaxChildren; i++) {
            if (HubCheckConnection(device, i) != Ok) continue;		// If port is not connected move to next port
            if (device->HubPayload->Children[i] != NULL)			// If child device is valid
                HubCheckForChange(device->HubPayload->Children[i]);	// Iterate this call
        }
    }
}

/*==========================================================================}
{						 INTERNAL ENUMERATION ROUTINES						}
{==========================================================================*/

/*-INTERNAL: EnumerateHID------------------------------------------------------
 If normal device enumeration detects a hid device, after normal single node
 enumeration it will call this procedure to enumerate connected HID devices.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT EnumerateHID (UsbDevice* device);

/*-INTERNAL: EnumerateHub ---------------------------------------------------
 Continues enumeration of each port if an enumerated detected device is a hub
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT EnumerateHub (struct UsbDevice *device) {
    RESULT result;
    uint32_t transfer;
    struct HubDevice *data;
    struct HubFullStatus status;

    if (auto const thisResult = AddHubPayload(device); thisResult != Ok) {					// We are a hub so we need a hub payload
        LOG("Could not allocate hub payload, Error ID %i\n", thisResult);
        return thisResult;												// We must have to fouled up device allocation code
    }

    data = device->HubPayload;										// Hub payload data added grab pointer to it we will be using it a fair bit

    for (int i = 0; i < MaxChildrenPerDevice; i++)
        data->Children[i] = NULL;									// For safety make sure all children pointers are NULL

    result = HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_HUB, 
        0, 0, &data->Descriptor, sizeof(struct HubDescriptor),
        bmREQ_GET_HUB_DESCRIPTOR, &transfer, true);					// Fetch the HUB descriptor and hold in the hub payload, we use it a bit so saves USB bus
    if ((result != RESULT::Ok) || (transfer != sizeof(struct HubDescriptor)))
    {
        LOG("HCD: Could not fetch hub descriptor for device: %i\n",
            device->Pipe0.Number);									// Log the error
        return ErrorDevice;											// No idea what problem is so bail
    }
    LOG_DEBUG("Hub device %i has %i ports\n", device->Pipe0.Number, data->Descriptor.PortCount);
    LOG_DEBUG("HUB: Hub power to good: %dms.\n", data->Descriptor.PowerGoodDelay * 2);
    LOG_DEBUG("HUB: Hub current required: %dmA.\n", data->Descriptor.MaximumHubPower * 2);

    if (data->Descriptor.PortCount > MaxChildrenPerDevice) {		// Check number of ports on hub vs maxium number we allow on a hub payload
        LOG("HUB device:%i is too big for this driver to handle. Only the first %d ports will be used.\n",
            device->Pipe0.Number, MaxChildrenPerDevice);			// Log error			
    }
    else data->MaxChildren = data->Descriptor.PortCount;			// Reduce number of children down to same as hub supports

    if (auto const thisResult = HCDReadHubPortStatus(device, 0, (uint8_t*)&status.Raw32); thisResult != Ok) // Gateway node status
    {
        LOG("HUB device:%i failed to get hub status.\n", device->Pipe0.Number);
        return thisResult;
    }

    LOG("HUB: Hub powering ports on.\n");
    for (int i = 0; i < data->MaxChildren; i++) {					// For each port
        if (HCDChangeHubPortFeature(device, FeaturePower,
            i + 1, true) != Ok)										// Power the port							
            LOG("HUB: device: %i could not power Port%d.\n",
                device->Pipe0.Number, i + 1);						// Log error
    }
    Cpu::DelayInMicroseconds(data->Descriptor.PowerGoodDelay * 2000);				// Every hub has a different power stability delay
    Cpu::DelayInMicroseconds(1'000'000);									// Wait 1 second to allow power to stabilize

    for (int port = 0; port < data->MaxChildren; port++) {			// Now check for new device to enumerate on each port
        HubCheckConnection(device, port);							// Run connection check on each port
    }

    return Ok;														// Return success
}


/*-INTERNAL: EnumerateDevice ------------------------------------------------
 All detected devices start enumeration here. We recover critical information
 of every USB device and hold those details in the device data block. Finally 
 if the device is recognized as any of the sepcial specific class then it will
 call extended enumeration for those specific classes.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT EnumerateDevice(struct UsbDevice *device, struct UsbDevice* ParentHub, uint8_t PortNum) {
    RESULT result;
    DWCRESULT dwcResult;
    uint8_t address;
    uint32_t transferred;
    DeviceDescriptor desc = { 0 };
    char buffer[256] __attribute__((aligned(4)));					// Text buffer

    /* Store the unique address until it is actually assigned. */
    address = device->Pipe0.Number;									// Hold unique address we will set device to
    device->Pipe0.Number = 0;										// Initially it starts as zero
    LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 1 = Read first 8 Bytes of Device Descriptor\n");
    device->Pipe0.MaxPacketSizeInBits = 8;							// Set max packet size to 8 ( So exchange will be exactly 1 packet)
    UsbPipeControl pipectrl = {
        .Type = USB_TRANSFER_TYPE_CONTROL,							// Control packet
        .Channel = dwc_get_free_channel(),							// Find first free channel
        .Direction = USB_DIRECTION_IN,								// We are reading to host
    };

    result = HCDSumbitControlMessage(
        device,												// Pipe as given to us
        pipectrl,													// Pipe control structure
        (uint8_t*)&desc,											// Pointer to descriptor
        8,															// Ask for first 8 bytes as per USB specification
        UsbDeviceRequest {								// We will build a request structure
            .Type = bmREQ_GET_DEVICE_DESCRIPTOR,					// Recipient is a flag usually 0x0 for normal device, 0x20 for a hub
            .Request = GetDescriptor,								// We want a descriptor obviously
            .Value = (uint16_t)USB_DESCRIPTOR_TYPE_DEVICE << 8,		// Type and the index (0) get compacted as the value
            .Index = 0,												// We want descriptor 0
            .Length = 8,											// 8 bytes as per USB enumeration by the book
        },
        ControlMessageTimeout,										// The standard timeout for any control message
        &transferred);												// Pass in pointer to get bytes transferred back
    if ((result != Ok) || (transferred != 8)) {						// This should pass on any valid device
        dwc_release_channel(pipectrl.Channel);						// Release the channel we are exiting
        LOG("Enumeration: Step 1 on device %i failed, Result: %#x, transferred: %u.\n",
            address, result, transferred);										// Log any error
        return result != Ok ? result : RESULT::ErrorTransmission;	// Fatal enumeration error of this device
    }
    LOG_DEBUG("Max packet size: %u\n", desc.bMaxPacketSize0);
    device->Pipe0.MaxPacketSizeInBits = desc.bMaxPacketSize0;	    // Set the maximum endpoint packet size to pipe from response
    device->Config.Status = USB_STATUS_DEFAULT;						// Move device enumeration to default

    LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 2 = Reset Port (old device support)\n");
    if (ParentHub != NULL) {										// Roothub is the only one who will have a NULL parent and you can't reset a FAKE hub
        // Reset the port for what will be the second time.
        if ((result = HubPortReset(ParentHub, PortNum)) != Ok) {
            dwc_release_channel(pipectrl.Channel);					// Release the channel we are exiting
            LOG("HCD: Failed to reset port again for new device %s.\n", UsbGetDescription(device));
            device->Pipe0.Number = address;
            return result;
        }
    }
    
    LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 3 = Set Device Address %u\n", address);
    if ((result = HCDSetAddress(device, pipectrl.Channel, address)) != Ok) {
        dwc_release_channel(pipectrl.Channel);					   // Release the channel we are exiting
        LOG("Enumeration: Failed to assign address to %#x.\n", address);// Log the error
        device->Pipe0.Number = address;								// Set device number just so it stays valid
        return result;												// Fatal enumeration error of this device
    }
    device->Pipe0.Number = address;									// Device successfully addressed so put it back to control pipe								
    Cpu::DelayInMicroseconds(10000);												// Allows time for address to propagate.
    device->Config.Status = USB_STATUS_ADDRESSED;					// Our enumeration status in now addressed

    LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 4 = Read Device Descriptor At Address\n");
    result = HCDGetDescriptor(
        device,												// Device control 0 pipe
        USB_DESCRIPTOR_TYPE_DEVICE,							        // Fetch device descriptor 
        0,															// Index 0
        0,															// Language 0
        &device->Descriptor,										// Pointer to buffer in device structure 
        sizeof(device->Descriptor),									// Ask for entire descriptor
        bmREQ_GET_DEVICE_DESCRIPTOR,								// Recipient device
        &transferred, true);										// Pass in pointer to get bytes transferred back
    if (result == RESULT::Ok && transferred != sizeof(device->Descriptor))
    {
        // This should pass on any valid device
        dwc_release_channel(pipectrl.Channel);						// Release the channel we are exiting
        LOG("Enumeration: Step 4 on device %i failed, Got %u bytes != %u.\n",
            device->Pipe0.Number, transferred, (uint32_t)sizeof(device->Descriptor));
        return RESULT::ErrorTransmission;
    }
    if (result != RESULT::Ok)
    {
        dwc_release_channel(pipectrl.Channel);						// Release the channel we are exiting
        LOG("Enumeration: Step 4 on device %i failed, Result: %#x.\n",
            device->Pipe0.Number, result);						// Log any error
        return result;
    }
    LOG_DEBUG("Device: %u, Class: %u, Subclass: %u\n", device->Pipe0.Number, device->Descriptor.bDeviceClass, device->Descriptor.bDeviceSubClass);


    LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 5 = Read Device Configurations\n");
    // Read the master Config at index 0 ... this is not really a config but an index to avail configs
    uint32_t transfer;
    ConfigurationDescriptor configDesc;
    result = HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_CONFIGURATION, 0, 0,
        &configDesc, sizeof(configDesc), bmREQ_GET_DEVICE_DESCRIPTOR,
        &transfer, true);											// Read the config descriptor 	
    if (result == RESULT::Ok && transfer != sizeof(configDesc))
    {
        dwc_release_channel(pipectrl.Channel);						// Release the channel we are exiting
        LOG("HCD: Got %u bytes != %u reading configuration descriptor for device: %i\n",
            transfer, (uint32_t)sizeof(device->Descriptor), device->Pipe0.Number);
        return RESULT::ErrorTransmission;
    }
    if (result != RESULT::Ok) {
        dwc_release_channel(pipectrl.Channel);						// Release the channel we are exiting
        LOG("HCD: Error: %i, reading configuration descriptor for device: %i\n",
            result, device->Pipe0.Number);
        return RESULT::ErrorDevice;											// No idea what problem is so bail
    }
    device->Config.ConfigStringIndex = configDesc.iConfiguration;	// Grab string index while here

    // Most devices I played with only have 1 config .. regardless we will take first
    // The index to call is given as at offset 5 bConfigurationValue
    // Read it by that index it's probably the same but just do it
    uint8_t configNum = configDesc.bConfigurationValue;
    // Okay we have the total length of config so we will read it in entirity
    uint8_t configBuffer[1024];										// Largest config I have ever seen is few hundred bytes this is 1K buffer
    result = HCDSumbitControlMessage(
        device,												// Device 
        pipectrl,											        // Create pipe control structure 
        &configBuffer[0],											// Buffer pointer passed in as is
        configDesc.wTotalLength,									// Length of whole config descriptor
        UsbDeviceRequest {								// We will build a request structure
            .Type = bmREQ_GET_DEVICE_DESCRIPTOR,					// We want normal device descriptor
            .Request = GetDescriptor,								// We want a descriptor obviously
            .Value = (uint16_t)USB_DESCRIPTOR_TYPE_CONFIGURATION << 8,// Type and the index get compacted as the value
            .Index = 0,												// Language ID is the index
            .Length = configDesc.wTotalLength,						// Duplicate the length
        },
        ControlMessageTimeout,										// The standard timeout for any control message
        &transfer);													// Set pointer to fetch transfer bytes
    if ((result != Ok) || (transfer != configDesc.wTotalLength)) {	// Check if anything went wrong
        dwc_release_channel(pipectrl.Channel);						// Release the channel
        LOG("HCD: Failed to read configuration descriptor for device %i, %u bytes read, Error: %i.\n",
            device->Pipe0.Number, (unsigned int)transfer, result);				// Log error
        if (result != Ok) return result;							// Return error result
        return ErrorDevice;											// Something went badly wrong .. bail
    }

    // So now we need to search for interfaces and endpoints
    uint8_t EndPtCnt = 0;											// Preset endpoint count to zero
    uint8_t hidCount = 0;											// Preset hid count to zero
    uint32_t i = 0;													// Start array search at zero
    while (i < configDesc.wTotalLength - 1) {						// So while we havent reached end of config data
        switch (configBuffer[i + 1]) {								// i will be on a descriptor header i+1 is decsriptor type 
        case USB_DESCRIPTOR_TYPE_INTERFACE: {						// Ok we have an interface descriptor we need to add it
            myMemCopy((uint8_t*)&device->Interfaces[device->MaxInterface],
                &configBuffer[i], 
                sizeof(struct UsbInterfaceDescriptor));				// configBuffer[i] is descriptor size as well as first byte
            device->MaxInterface++;									// One interface added
            EndPtCnt = 0;											// Reset endpoint count to zero (we are on new interface now)
            break;
        }
        case USB_DESCRIPTOR_TYPE_ENDPOINT: {						// Ok we have an endpoint descriptor we need to add it
            myMemCopy((uint8_t*)&device->Endpoints[device->MaxInterface - 1][EndPtCnt], 
                &configBuffer[i],
                sizeof(struct UsbEndpointDescriptor));				// configBuffer[i] is descriptor size as well as first byte
            EndPtCnt++;												// One endpoint added so move index
            break;
        }
        case USB_DESCRIPTOR_TYPE_HID: {								// HID Interface found
            if (hidCount == 0) {									// First HID descriptor found
                if ((result = AddHidPayload(device)) != RESULT::Ok) {		// Ok so we need to add a hid payload to device
                    dwc_release_channel(pipectrl.Channel);			// Release the channel we are exiting
                    LOG("Could not allocate hid payload, Error ID %i\n", result);
                    return result;									// We must have to fouled up device allocation code
                };
            }
            // Set the HID descriptor in the payload
            if (SetHidDescriptor(device->HidPayload, hidCount, device->MaxInterface - 1, &configBuffer[i], configBuffer[i]))
            {
                hidCount++;
            }
            break;
        }
        default:
            break;
        }
        i = i + configBuffer[i];									// Add config descriptor size .. which moves us to next descriptor
    }

    LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 6 = Set Configuration to Device\n");
    if (auto const thisResult = HCDSetConfiguration(device, pipectrl.Channel, configNum); thisResult != Ok) {
        dwc_release_channel(pipectrl.Channel);					   // Release the channel we are exiting
        LOG("HCD: Failed to set configuration %#x for device %i.\n",
            configNum, device->Pipe0.Number);
        return thisResult;
    }
    device->Config.ConfigIndex = configNum;							// Hold the configuration index
    device->Config.Status = USB_STATUS_CONFIGURED;					// Set device status to configured

    LOG("HCD: Attach Device %s. Address:%d Class:%d USB:%x.%x, %d configuration(s), %d interface(s).\n",
        UsbGetDescription(device), address, device->Descriptor.bDeviceClass, (device->Descriptor.bcdUSB >> 8) & 0xFF,
        device->Descriptor.bcdUSB & 0xFF, device->Descriptor.bNumConfigurations, device->MaxInterface);
    
    if (device->Descriptor.iProduct != 0) {
        size_t length = sizeof(buffer);
        if (HCDReadStringDescriptor(device, device->Descriptor.iProduct, &buffer[0], length) == Ok)
        {
            LOG("HCD:  -Product:       %s.\n", buffer);
        }
    }
    
    if (device->Descriptor.iManufacturer != 0) {
        size_t length = sizeof(buffer);
        if (HCDReadStringDescriptor(device, device->Descriptor.iManufacturer, &buffer[0], length) == Ok)
        {
            LOG("HCD:  -Manufacturer:  %s.\n", buffer);
        }
    }
    if (device->Descriptor.iSerialNumber != 0) {
        size_t length = sizeof(buffer);
        if (HCDReadStringDescriptor(device, device->Descriptor.iSerialNumber, &buffer[0], length) == Ok)
        {
            LOG("HCD:  -SerialNumber:  %s.\n", buffer);
        }
    }


    if (device->Config.ConfigStringIndex != 0) {
        size_t length = sizeof(buffer);
        if (HCDReadStringDescriptor(device, device->Config.ConfigStringIndex, &buffer[0], length) == Ok)
        {
            LOG("HCD:  -Configuration: %s.\n", buffer);
        }
    }


    LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 7 = ENUMERATE SPECIAL DEVICES\n");
    if (device->Descriptor.bDeviceClass == DeviceClassHub) {		// If device is a hub then enumerate it
        LOG_DEBUG("Device is a hub, enumerating ports.\n");
        if ((result = EnumerateHub(device)) != RESULT::Ok) {				// Run hub enumeration
            dwc_release_channel(pipectrl.Channel);					// Release the channel we are exiting
            LOG("Could not enumerate HUB device %i, Error ID %i\n",
                device->Pipe0.Number, result);						// Log error
            return result;											// Return the error
        }
    } else if (hidCount > 0) {										// HID interface on the device
        LOG_DEBUG("Device hidCount: %u, enumerating ports.\n", hidCount);
        if ((result = EnumerateHID(device)) != RESULT::Ok) {	// Ok so enumerate the HID device
            dwc_release_channel(pipectrl.Channel);					// Release the channel we are exiting
            LOG("Could not enumerate HID device %i, Error ID %i\n",
                device->Pipe0.Number, result);
            return result;											// return the error
        }
    }
    else {														// If not a hub or HID then just log the device
        LOG_DEBUG("Device is not a hub or HID, skipping enumeration.\n");
    }
    dwc_release_channel(pipectrl.Channel);							// Release the channel we are exiting
    return Ok;
}

/*-INTERNAL: EnumerateDevice ------------------------------------------------
 This is called from USBInitialize and will allocate our fake rootHub device 
 and then begin enumeration of the whole USB bus.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT UsbAttachRootHub(void) {
    RESULT result;
    struct UsbDevice *rootHub = NULL;
    LOG_DEBUG("Allocating RootHub\n");
    if (DeviceTable[0].PayLoadId != 0)								// If RootHub is already in use
        UsbDeallocateDevice(&DeviceTable[0]);						// We will need to deallocate it and every child
    result = UsbAllocateDevice(&rootHub);							// Try allocating the root hub now
    if (rootHub != &DeviceTable[0]) result = ErrorCompiler;			// Somethign really wrong .. 1st allocation should always be DeviceList[0]
    if (result != Ok) return result;								// Return error result somethging fatal happened
    DeviceTable[0].Pipe0.Speed = USB_SPEED_FULL;					// Set our fake hub to full speed .. as it's fake we cant really ask it speed can we :-)
    DeviceTable[0].Pipe0.MaxPacketSizeInBits = 64;					// Set our fake hub to 64 byte packets .. as it's fake we need to do it manually
    DeviceTable[0].Config.Status = USB_STATUS_POWERED;				// Set our fake hub status to configured .. as it's fake we need to do manually
    RootHubDeviceNumber = 0;										// Roothub number is zero
    Cpu::DelayInMicroseconds(1'000'000);								// Wait 1 second to allow the USB bus to stabilize
    return EnumerateDevice(&DeviceTable[0], NULL, 0);				// Ok start enumerating the USB bus as roothub port 1 is the physical bus
}

/***************************************************************************}
{					      PUBLIC INTERFACE ROUTINES			                }
****************************************************************************/

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
                         enum usb_descriptor_type type,				// The type of descriptor
                         uint8_t index,								// The index of the type descriptor
                         uint16_t langId,							// The language id
                         void* buffer,								// Buffer to recieve descriptor
                         uint32_t length,							// Maximumlength of descriptor
                         uint8_t recipient,							// Recipient flags									 
                         uint32_t *bytesTransferred,     			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)								
                         bool runHeaderCheck)						// Whether to run header check
{
    RESULT result;
    uint32_t transfer;
    alignas(4) struct UsbDescriptorHeader header  = { 0 };
    UsbPipeControl pipectrl = {
        .Type = USB_TRANSFER_TYPE_CONTROL,							// This is a control request
        .Channel = dwc_get_free_channel(),							// Find first free channel
        .Direction = USB_DIRECTION_IN,								// In to host as we are getting
    };
    if (runHeaderCheck) {
        result = HCDSumbitControlMessage(
            device,													// Pipe passed in as is
            pipectrl,											    // Pipe control structure 
            (uint8_t*)&header,										// Buffer to description header
            sizeof(header),											// Size of the header
            UsbDeviceRequest {							// We will build a request structure
                .Type = recipient,									// Recipient is a flag usually bmREQ_GET_DEVICE_DESCRIPTOR, bmREQ_GET_HUB_DESCRIPTOR etc
                .Request = GetDescriptor,							// We want a descriptor obviously
                .Value = (uint16_t)(type << 8 | index),				// Type and the index get compacted as the value
                .Index = langId,									// Language ID is the index
                .Length = sizeof(header),							// Duplicate the length
            },
            ControlMessageTimeout,									// The standard timeout for any control message
            NULL);													// Ignore bytes transferred
        if ((result == Ok) && (header.DescriptorType != type))
            result = ErrorGeneral;									// For some strange reason descriptor type is not right
        if (result != Ok) {											// RESULT in error
            dwc_release_channel(pipectrl.Channel);					// Release the channel
            LOG("HCD: Fail to get descriptor %#x:%#x recepient: %#x, device:%i. RESULT %#x.\n",
                type, index, recipient, device->Pipe0.Number, result);		// Log any error
            return result;											// Error reading descriptor header
        }
        if (length > header.DescriptorLength)						// Check descriptor length vs buffer space
            length = header.DescriptorLength;						// The descriptor is shorter than buffer space provided
    }
    result = HCDSumbitControlMessage(
        device,														// Pipe passed in as is
        pipectrl,												    // Pipe control structure 
        (uint8_t*)buffer,														// Buffer pointer passed in as is
        length,														// Length transferred (it may be shorter from above)
        UsbDeviceRequest {								// We will build a request structure
            .Type = recipient,										//  Recipient is a flag usually bmREQ_GET_DEVICE_DESCRIPTOR, bmREQ_GET_HUB_DESCRIPTOR etc
            .Request = GetDescriptor,								// We want a descriptor obviously
            .Value = (uint16_t)(type << 8 | index),					// Type and the index get compacted as the value
            .Index = langId,										// Language ID is the index
            .Length = (uint16_t)length,										// Duplicate the length
        },
        ControlMessageTimeout,										// The standard timeout for any control message
        &transfer);													// Set pointer to fetch transfer bytes
    if (length != transfer) result = ErrorTransmission; 			// The requested length does not match read length
    if (result != Ok) {
        dwc_release_channel(pipectrl.Channel);						// Release the channel
        LOG("HCD: Failed to get descriptor %#x:%#x for device:%i. RESULT %#x.\n",
            type, index, device->Pipe0.Number, result);						// Log any error
    }
    dwc_release_channel(pipectrl.Channel);							// Release the channel
    if (bytesTransferred) *bytesTransferred = transfer;				// Return the bytes transferred
    return result;													// Return the result
}

/*--------------------------------------------------------------------------}
{					 PUBLIC GENERIC USB INTERFACE ROUTINES					}
{--------------------------------------------------------------------------*/

/*-UsbInitialise-------------------------------------------------------------
 Initialises the USB driver by performing necessary interfactions with the
 host controller driver, and enumerating the initial device tree.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT UsbInitialise (void) {
    DWCRESULT result;
    if ((result = HCDInitialise()) != DWCRESULT::Ok) {							// Initialize host control driver
        LOG("FATAL ERROR: HCD failed to initialise.\n");			// Some hardware issue
        return ResultFromDwcResult(result);												// Return any fatal error
    }

    if ((result = HCDStart()) != DWCRESULT::Ok) {								// Start the host control driver						
        LOG("USBD: Abort, HCD failed to start.\n");
        return ResultFromDwcResult(result);												// Return any fatal error
    }
    if (auto const thisResult = UsbAttachRootHub(); thisResult != RESULT::Ok) {						// Attach the root hub .. which will launch enumeration
        LOG("USBD: Failed to enumerate devices.\n");
        return thisResult;												// Retrn any fatal error
    }
    return Ok;														// Return success
}

DeviceDescriptor GetDeviceDescriptor(uint8_t devNumber)
{
    UsbDevice* device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr) return DeviceDescriptor();
    return device->Descriptor;
}

size_t GetDeviceProductString(uint8_t devNumber, std::span<char> buffer)
{
    if (buffer.size() == 0) return 0;
    UsbDevice* device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr) return 0;
    if (device->Descriptor.iProduct == 0) return 0;
    size_t length = buffer.size();
    if (HCDReadStringDescriptor(device, device->Descriptor.iProduct, buffer.data(), length) != Ok)
    {
        return 0;
    }
    return length;
}

size_t GetDeviceManufacturerString(uint8_t devNumber, std::span<char> buffer)
{
    if (buffer.size() == 0) return 0;
    UsbDevice* device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr) return 0;
    if (device->Descriptor.iManufacturer == 0) return 0;
    size_t length = buffer.size();
    if (HCDReadStringDescriptor(device, device->Descriptor.iManufacturer, buffer.data(), length) != Ok)
    {
        return 0;
    }
    return length;
}

size_t GetDeviceSerialNumberString(uint8_t devNumber, std::span<char> buffer)
{
    if (buffer.size() == 0) return 0;
    UsbDevice* device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr) return 0;
    if (device->Descriptor.iSerialNumber == 0) return 0;
    size_t length = buffer.size();
    if (HCDReadStringDescriptor(device, device->Descriptor.iSerialNumber, buffer.data(), length) != Ok)
    {
        return 0;
    }
    return length;
}

size_t GetDeviceConfigStringString(uint8_t devNumber, std::span<char> buffer)
{
    if (buffer.size() == 0) return 0;
    UsbDevice* device = UsbDeviceAtAddress(devNumber);
    if (device == nullptr) return 0;
    if (device->Config.ConfigStringIndex == 0) return 0;
    size_t length = buffer.size();
    if (HCDReadStringDescriptor(device, device->Config.ConfigStringIndex, buffer.data(), length) != Ok)
    {
        return 0;
    }
    return length;
}

/*-IsHub---------------------------------------------------------------------
 Will return if the given usbdevice is infact a hub and thus has hub payload
 data available. Remember the gateway node of a hub is a normal usb device.
 You should always call this first up in any routine that accesses the hub
 payload to make sure the payload pointers are valid. If it returns true it
 is safe to proceed and do things with the hub payload via it's pointer.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
bool IsHub (uint8_t devNumber) {
    if ((devNumber > 0) && (devNumber <= MaximumDevices)) {			// Check the address is valid not zero and max devices or less
        struct UsbDevice* device = &DeviceTable[devNumber - 1];		// Shortcut to device pointer we are talking about	
        if (device->PayLoadId == HubPayload && device->HubPayload)	// It has a HUB payload ID and the HUB payload pointer is valid
            return true;											// Confirmed as a hub
    }
    return false;													// Not a hub
}

/*-IsHid---------------------------------------------------------------------
 Will return if the given usbdevice is infact a hid and thus has hid payload
 data available. Remember a hid device is a normal usb device which takes
 human input (like keyboard, mouse etc). You should always call this first
 in any routine that accesses the hid payload to make sure the pointers are
 valid. If it returns true it is safe to proceed and do things with the hid
 payload via it's pointer.
 24Feb17 LdB
--------------------------------------------------------------------------*/
bool IsHid (uint8_t devNumber) {
    if ((devNumber > 0) && (devNumber <= MaximumDevices)) {			// Check the address is valid not zero and max devices or less
        struct UsbDevice* device = &DeviceTable[devNumber - 1];		// Shortcut to device pointer we are talking about					
        if (device->PayLoadId == HidPayload && device->HidPayload)	// It has a HID payload ID and the HID payload pointer is valid
            return true;											// Confirmed as a hid
    }
    return false;													// Not a hid
}

/*-IsMassStorage------------------------------------------------------------
 Will return if the given usbdevice is infact a mass storage device and thus 
 has a mass storage payload data available. You should always call this first
 in any routine that accesses the storage payload to make sure the pointers 
 are valid. If it returns true it is safe to proceed and do things with the 
 storage payload via it's pointer.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
bool IsMassStorage (uint8_t devNumber) {
    if ((devNumber > 0) && (devNumber <= MaximumDevices)) {			// Check the address is valid not zero and max devices or less
        struct UsbDevice* device = &DeviceTable[devNumber - 1];		// Shortcut to device pointer we are talking about
        if (device->PayLoadId == MassStoragePayload &&				// Device pointer is valid and we have a payload id of mass storage
            device->MassPayload != NULL) return true;				// Confirmed as a mass storage device
    }
    return false;													// Not a mass storage device
}

/*-IsMouse-------------------------------------------------------------------
 Will return if the given usbdevice is infact a mouse. This initially checks
 the device IsHid and then refines that down to looking at the interface and
 checking it is defined as a mouse.
 24Feb17 LdB
--------------------------------------------------------------------------*/
bool IsMouse (uint8_t devNumber) {
    if ((devNumber > 0) && (devNumber <= MaximumDevices)) {			// Check the address is valid not zero and max devices or less
        struct UsbDevice* device = &DeviceTable[devNumber - 1];		// Shortcut to device pointer we are talking about
        if (device->PayLoadId == HidPayload && device->HidPayload   // Its a valid HID
         && device->Interfaces[0].Protocol == 2) return true;		// Protocol 2 means a mouse
    }
    return false;													// Not a mouse device
}

/*-IsKeyboard----------------------------------------------------------------
 Will return if the given usbdevice is infact a keyboard. This initially will
 check the device IsHid and then refines that down to looking at the interface
 and checking it is defined as a keyboard.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
bool IsKeyboard (uint8_t devNumber) {
    if ((devNumber > 0) && (devNumber <= MaximumDevices)) {			// Check the address is valid not zero and max devices or less
        struct UsbDevice* device = &DeviceTable[devNumber - 1];		// Shortcut to device pointer we are talking about
        if (device->PayLoadId == HidPayload && device->HidPayload   // Its a valid HID
            && device->Interfaces[0].Protocol == 1) return true;	// Protocol 1 means a keyboard
    }
    return false;													// Not a mouse device
}

/*-UsbGetRootHub ------------------------------------------------------------
 On a Universal Serial Bus, there exists a root hub. This if often a virtual
 device, and typically represents a one port hub, which is the physical
 universal serial bus for this computer. It is always address 1. It is present
 to allow uniform software manipulation of the universal serial bus itself.
 This will return that FAKE rootHub or NULL on failure. Reason for failure is
 generally not having called USBInitialize to start the USB system.          
 11Apr17 LdB
 --------------------------------------------------------------------------*/
struct UsbDevice * UsbGetRootHub (void) { 
    if (DeviceTable[0].PayLoadId != 0)								// Check the root hub is in use AKA Usbinitialize was called
        return &DeviceTable[0];										// Return the rootHub AKA DeviceList[0]
    return NULL;													// Return NULL as no valid rootHub
}

/*-UsbDeviceAtAddress -------------------------------------------------------
  Given the unique USB address this will return the pointer to the USB device
  structure. If the address is not actually in use it will return NULL.
 11Apr17 LdB
 --------------------------------------------------------------------------*/
UsbDevice* UsbDeviceAtAddress (uint8_t devNumber)
{
    if  (devNumber == 0 || devNumber > MaximumDevices)
    {
        return nullptr;
    }
    auto const device = &DeviceTable[devNumber-1];
    if (device->PayLoadId == 0)
    {
        // It's not in use.
        return nullptr;
    }
    return device;
}

uint32_t GetDeviceNumber(UsbDevice* device)
{
    if (device == nullptr || device->PayLoadId == ErrorPayload)
    {
        return 0; // Invalid device
    }
    return device->Pipe0.Number; // Return the unique USB address of the device
}

HidDevice* GetHidDevice(UsbDevice* device)
{
    if (device == nullptr || device->PayLoadId != HidPayload)
    {
        return nullptr;
    }
    return device->HidPayload;
}

UsbInterfaceDescriptor GetInterfaceDescriptor(UsbDevice* device, uint8_t interfaceIndex)
{
    if (device == nullptr || interfaceIndex >= device->MaxInterface)
    {
        return {}; // Return an empty descriptor if the device is invalid or index is out of bounds
    }
    return device->Interfaces[interfaceIndex]; // Return the interface descriptor at the specified index
}

UsbEndpointDescriptor FindEndpoint(UsbDevice* device, uint8_t interfaceIndex, usb_transfer_type type, UsbDirection direction)
{
    if (interfaceIndex >= device->MaxInterface)
    {
        return {};
    }

    // Search through endpoints for this interface to find the requested endpoint
    for (int i = 0; i < MaxEndpointsPerDevice; i++)
    {
        UsbEndpointDescriptor const& ep = device->Endpoints[interfaceIndex][i];
        if (ep.Header.DescriptorLength == 0)
        {
            // No more endpoints
            return {};
        }
        
        // Check if this is an interrupt IN endpoint
        if (ep.Attributes.Type == type && ep.EndpointAddress.Direction == direction)
        {
            return ep;
        }
    }
    return {};
}


/*--------------------------------------------------------------------------}
{					 PUBLIC USB CHANGE CHECKING ROUTINES					}
{--------------------------------------------------------------------------*/

/*-UsbCheckForChange --------------------------------------------------------
 Recursively calls HubCheckConnection on all ports on all hubs connected to
 the root hub. It will hence automatically change the device tree matching
 any physical changes.
 10Apr17 LdB
 --------------------------------------------------------------------------*/
void UsbCheckForChange(void) {
    if (DeviceTable[0].PayLoadId != 0)								// Check the root hub is in use AKA Usbinitialize was called
        HubCheckForChange(&DeviceTable[0]);							// Launch iterration of checking for changes from the roothub
}


/*--------------------------------------------------------------------------}
{					 PUBLIC DISPLAY USB INTERFACE ROUTINES					}
{--------------------------------------------------------------------------*/

/*-UsbGetDescription --------------------------------------------------------
 Returns a description for a device. This is not read from the device, this
 is just generated given by the driver.
 Unchanged from Alex Chadwick
 --------------------------------------------------------------------------*/
const char* UsbGetDescription (struct UsbDevice *device) {
    if (device->Config.Status == USB_STATUS_ATTACHED)
        return "New Device (Not Ready)\0";
    else if (device->Config.Status == USB_STATUS_POWERED)
        return "Unknown Device (Not Ready)\0";
    else if (device == &DeviceTable[0])
        return "USB Root Hub\0";

    switch (device->Descriptor.bDeviceClass) {
    case DeviceClassHub:
        if (device->Descriptor.bcdUSB == 0x210)
            return "USB 2.1 Hub\0";
        else if (device->Descriptor.bcdUSB == 0x200)
            return "USB 2.0 Hub\0";
        else if (device->Descriptor.bcdUSB == 0x110)
            return "USB 1.1 Hub\0";
        else if (device->Descriptor.bcdUSB == 0x100)
            return "USB 1.0 Hub\0";
        else
            return "USB Hub\0";
    case DeviceClassVendorSpecific:
        if (device->Descriptor.idVendor == 0x424 &&
            device->Descriptor.idProduct == 0xec00)
            return "SMSC LAN9512\0";
    case DeviceClassInInterface:
        if (device->Config.Status == USB_STATUS_CONFIGURED) {
            switch (device->Interfaces[0].Class) {
            case InterfaceClass::Audio:
                return "USB Audio Device\0";
            case InterfaceClass::Communications:
                return "USB CDC Device\0";
            case InterfaceClass::Hid:
                switch (device->Interfaces[0].Protocol) {
                case 1:
                    return "USB Keyboard\0";
                case 2:
                    return "USB Mouse\0";
                default:
                    return "USB HID\0";
                }
            case InterfaceClass::Physical:
                return "USB Physical Device\0";
            case InterfaceClass::Image:
                return "USB Imaging Device\0";
            case InterfaceClass::Printer:
                return "USB Printer\0";
            case InterfaceClass::MassStorage:
                return "USB Mass Storage Device\0";
            case InterfaceClass::Hub:
                if (device->Descriptor.bcdUSB == 0x210)
                    return "USB 2.1 Hub\0";
                else if (device->Descriptor.bcdUSB == 0x200)
                    return "USB 2.0 Hub\0";
                else if (device->Descriptor.bcdUSB == 0x110)
                    return "USB 1.1 Hub\0";
                else if (device->Descriptor.bcdUSB == 0x100)
                    return "USB 1.0 Hub\0";
                else
                    return "USB Hub\0";
            case InterfaceClass::CdcData:
                return "USB CDC-Data Device\0";
            case InterfaceClass::SmartCard:
                return "USB Smart Card\0";
            case InterfaceClass::ContentSecurity:
                return "USB Content Secuity Device\0";
            case InterfaceClass::Video:
                return "USB Video Device\0";
            case InterfaceClass::PersonalHealthcare:
                return "USB Healthcare Device\0";
            case InterfaceClass::AudioVideo:
                return "USB AV Device\0";
            case InterfaceClass::DiagnosticDevice:
                return "USB Diagnostic Device\0";
            case InterfaceClass::WirelessController:
                return "USB Wireless Controller\0";
            case InterfaceClass::Miscellaneous:
                return "USB Miscellaneous Device\0";
            case InterfaceClass::VendorSpecific:
                return "Vendor Specific\0";
            default:
                return "Generic Device\0";
            }
        }
        else if (device->Descriptor.bDeviceClass == DeviceClassVendorSpecific)
            return "Vendor Specific\0";
        else
            return "Unconfigured Device\0";
    default:
        return "Generic Device\0";
    }
}

/*-UsbShowTree --------------------------------------------------------------
 Shows the USB tree as ascii art using the Printf command. The normal command
 to show from roothub up is  UsbShowTree(UsbGetRootHub(), 1, '+');
 14Mar17 LdB
 --------------------------------------------------------------------------*/
static int TreeLevelInUse[20] = { 0 };

void UsbShowTree(struct UsbDevice *root, const int level, const char tee) {
    char indent[1024] = { 0 };								// Indent buffer for verbose lines
    for (int i = 0; i < level - 1; i++)
    {
        if (TreeLevelInUse[i] == 0)
        {
            printf("   ");
            sprintf(indent + i * 3, "   ");
        }
        else
        {
            printf(" %c ", '\xB3');							// Draw level lines if in use
            sprintf(indent + i * 3, " %c ", '\xB3');
        }
    }
    switch (tee)
    {
    case '\xC3':
        sprintf(indent + (level - 1) * 3, " %c    ", '\xB3');
        break;
    case '+':
    case '\xC0':
    {
        if (IsHub(root->Pipe0.Number))
        {
            bool drawLine = false;
            uint32_t lastChild = root->HubPayload->MaxChildren;
            for (uint32_t i = 0; i < lastChild; i++)
            {
                if (root->HubPayload->Children[i])
                {
                    // Some node is in use so we need to draw the line
                    drawLine = true;
                    break;
                }
            }
            if (drawLine)
            {
                sprintf(indent + (level - 1) * 3, "    %c ", '\xB3');
            }
            else
            {
                sprintf(indent + (level - 1) * 3, "      ");
            }
        }
        else
        {
            sprintf(indent + (level - 1) * 3, "      ");
        }
        break;
    }
    default:
        sprintf(indent + (level - 1) * 3, "      ");
        break;
    }
    
    printf(" %c-%s id: %u port: %u speed: %s packetsize: %u %s\n",
        tee, UsbGetDescription(root),
        root->Pipe0.Number,
        root->ParentHub.PortNumber,
        SpeedString[root->Pipe0.Speed],
        root->Pipe0.MaxPacketSizeInBits,
        IsHid(root->Pipe0.Number) ? "- HID interface" : ""
    );

    bool verbose = true;

    if (verbose)
    {
        printf("%s  config: %u configString: %u status: %u interfaces: %u DescriptorType %u bcdUSB %X\n",
            indent,
            root->Config.ConfigIndex,
            root->Config.ConfigStringIndex,
            root->Config.Status,
            root->MaxInterface,
            root->Descriptor.bDescriptorType,										// +0x1 Descriptor type
            root->Descriptor.bcdUSB 												// +0x2 (in BCD 0x210 = USB2.10)
        );
        printf("%s  DeviceClass %u DeviceSubClass %u DeviceProtocol %u\n",
            indent,
            root->Descriptor.bDeviceClass,											// +0x4 Class code (enum DeviceClass )
            root->Descriptor.bDeviceSubClass,										// +0x5 Subclass code (assigned by the USB-IF)
            root->Descriptor.bDeviceProtocol 										// +0x6 Protocol code (assigned by the USB-IF)
        );
        printf("%s  MaxPacketSize0 %u idVendor %u idProduct %u bcdDevice %X\n",
            indent,
            root->Descriptor.bMaxPacketSize0,										// +0x7 Maximum packet size for endpoint 0
            root->Descriptor.idVendor,												// +0x8 Vendor ID (assigned by the USB-IF)
            root->Descriptor.idProduct,												// +0xa Product ID (assigned by the manufacturer)
            root->Descriptor.bcdDevice 												// +0xc Device version number (BCD)
        );
        printf("%s  Manufacturer %u Product %u SerialNumber %u NumConfigurations %u\n",
            indent,
            root->Descriptor.iManufacturer,											// +0xe Index of String Descriptor describing the manufacturer.
            root->Descriptor.iProduct,												// +0xf Index of String Descriptor describing the product
            root->Descriptor.iSerialNumber,											// +0x10 Index of String Descriptor with the device's serial number
            root->Descriptor.bNumConfigurations 									// +0x11 Number of possible configurations
        );
        for (uint32_t i = 0; i < root->MaxInterface; i++)
        {
            printf("%s  - Interface %u Length %u Type %u Num %u Class %u SubClass %u\n",
                indent,
                i,
                root->Interfaces[i].Header.DescriptorLength,
                root->Interfaces[i].Header.DescriptorType,
                root->Interfaces[i].Number,
                root->Interfaces[i].Class,
                root->Interfaces[i].SubClass
            );
            printf("%s    Protocol %u AltSetting %u EndpointCount %u StringIndex %u\n",
                indent,
                root->Interfaces[i].Protocol,
                root->Interfaces[i].AlternateSetting,
                root->Interfaces[i].EndpointCount,
                root->Interfaces[i].StringIndex
            );
            for (uint32_t j = 0; j < root->Interfaces[i].EndpointCount; j++) { // For each endpoint on the interface
                printf("%s    - Endpoint %u Address %u %s Type %u Sync %u Usage %u\n",
                    indent,
                    j,
                    root->Endpoints[i][j].EndpointAddress.Number,
                    root->Endpoints[i][j].EndpointAddress.Direction == USB_DIRECTION_IN ? "IN" : "OUT",
                    root->Endpoints[i][j].Attributes.Type,
                    root->Endpoints[i][j].Attributes.Synchronisation,
                    root->Endpoints[i][j].Attributes.Usage
                );
                printf("%s      MaxPacketSize %u Transactions %u Interval %u\n",
                    indent,
                    root->Endpoints[i][j].Packet.MaxSize,
                    root->Endpoints[i][j].Packet.Transactions,
                    root->Endpoints[i][j].Interval
                );
            }
        }
        if (IsHid(root->Pipe0.Number))
        {
            for (uint8_t i = 0; i < GetHidCount(root->HidPayload); i++)
            {
                PrintHid(root->HidPayload, i, indent);
            }
        }
    }
    if (IsHub(root->Pipe0.Number))
    {
        uint32_t lastChild = root->HubPayload->MaxChildren;
        for (uint32_t i = 0; i < lastChild; i++) {						// For each child of hub
            char nodetee = '\xC0';									// Preset nodetee to end node ... "L"
            for (uint32_t j = i; j < lastChild - 1; j++) {				// Check if any following child node is valid
                if (root->HubPayload->Children[j + 1]) {			// We found a following node in use					
                    TreeLevelInUse[level] = 1;						// Set tree level in use flag
                    nodetee = (char)0xc3;							// Change the node character to tee looks like this "├"
                    break;											// Exit loop j
                };
            }
            if (root->HubPayload->Children[i]) {					// If child valid
                UsbShowTree(root->HubPayload->Children[i],
                    level + 1, nodetee);							// Iterate into child but level+1 down of coarse
            }
            TreeLevelInUse[level] = 0;								// Clear level in use flag
        }
    }
    else
    {
    }
}

// Sends/recieves data from/to the given buffer to/from the given endpoint.
RESULT HCDEndpointTransfer(UsbDevice* device, UsbEndpointDescriptor endpoint, uint8_t* buffer, uint32_t& bufferLength)
{
    LOG_DEBUG("HCD: %s transfer called for device %i, endpoint %i for %u bytes.\n",
        endpoint.EndpointAddress.Direction == USB_DIRECTION_IN ? "IN" : "OUT",
        device->Pipe0.Number, endpoint.EndpointAddress.Number, bufferLength);
    
    // Set up the pipe for interrupt transfer
    UsbPipe pipe = {
        .MaxPacketSizeInBits = endpoint.Packet.MaxSize,         // Endpoint max packet size
        .Speed               = device->Pipe0.Speed,             // Same speed as device
        .EndPoint            = endpoint.EndpointAddress.Number, // Endpoint address
        .Number              = device->Pipe0.Number,            // Same device address
        .splitNodePort    = device->Pipe0.splitNodePort,  // Copy low speed info
        .splitNodePoint   = device->Pipe0.splitNodePoint, // Copy low speed info
    };

    UsbPipeControl pipectrl = {
        .Type      = endpoint.Attributes.Type,
        .Channel   = dwc_get_free_channel(),
        .Direction = endpoint.EndpointAddress.Direction,
    };

    // Determine the correct data toggle (PID) for this endpoint
    // For interrupt endpoints, we need to alternate between DATA0 and DATA1
    PacketId packetId = USB_PID_DATA0;
    
    if (endpoint.Attributes.Type == USB_TRANSFER_TYPE_INTERRUPT) {
        // Find the interface this endpoint belongs to and get the data toggle state
        uint8_t interfaceIndex = 0;
        uint8_t endpointIndex = 0;
        bool foundEndpoint = false;
        
        // Search for this endpoint in the device's endpoint array
        for (uint8_t i = 0; i < device->MaxInterface && !foundEndpoint; i++) {
            for (uint8_t j = 0; j < MaxEndpointsPerDevice; j++) {
                auto const& ep = device->Endpoints[i][j];
                if (ep.Header.DescriptorLength == 0) break; // No more endpoints
                
                if (ep.EndpointAddress.Number == endpoint.EndpointAddress.Number &&
                    ep.EndpointAddress.Direction == endpoint.EndpointAddress.Direction &&
                    ep.Attributes.Type == endpoint.Attributes.Type) {
                    interfaceIndex = i;
                    endpointIndex = j;
                    foundEndpoint = true;
                    break;
                }
            }
        }
        
        if (foundEndpoint) {
            // Use the data toggle stored in the endpoint's interval field's lower bit as a simple toggle tracker
            // This is a temporary solution - in a full implementation, we'd add a proper data toggle field
            bool dataToggle = (device->Endpoints[interfaceIndex][endpointIndex].Interval & 0x80) != 0;
            packetId = dataToggle ? USB_PID_DATA1 : USB_PID_DATA0;
            
            LOG_DEBUG("HCD: Using %s for interrupt endpoint %d (toggle=%d)\n", 
                packetId == USB_PID_DATA0 ? "DATA0" : "DATA1", 
                endpoint.EndpointAddress.Number, dataToggle);
        } else {
            LOG("HCD: Warning - Could not find endpoint for data toggle tracking, using DATA0\n");
        }
    }

    // Start the interrupt transfer with the correct data toggle
    auto const result = HCDChannelTransfer(pipe, pipectrl, buffer, bufferLength, packetId);

    // Update data toggle on successful transfer for interrupt endpoints
    if (result == DWCRESULT::Ok && endpoint.Attributes.Type == USB_TRANSFER_TYPE_INTERRUPT) {
        // Find the endpoint again and toggle the data toggle bit
        for (uint8_t i = 0; i < device->MaxInterface; i++) {
            for (uint8_t j = 0; j < MaxEndpointsPerDevice; j++) {
                auto& ep = device->Endpoints[i][j];
                if (ep.Header.DescriptorLength == 0) break;
                
                if (ep.EndpointAddress.Number == endpoint.EndpointAddress.Number &&
                    ep.EndpointAddress.Direction == endpoint.EndpointAddress.Direction &&
                    ep.Attributes.Type == endpoint.Attributes.Type) {
                    // Toggle the data toggle bit (stored in bit 7 of Interval)
                    ep.Interval ^= 0x80;
                    LOG_DEBUG("HCD: Toggled data toggle for endpoint %d, new toggle=%d\n", 
                        endpoint.EndpointAddress.Number, (ep.Interval & 0x80) != 0);
                    goto toggle_updated;
                }
            }
        }
        toggle_updated:;
    }

    dwc_release_channel(pipectrl.Channel);                         // Release the channel
    return ResultFromDwcResult(result);
}
