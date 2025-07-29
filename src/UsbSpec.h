// USB 2.0 type definitions.
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
#include <stddef.h>

// Explicitly packing bitfields of different types keeps VSCode's IntelliSense happier.
#define PACKED __attribute__((__packed__))

/***************************************************************************}
{           PUBLIC USB 2.0 STRUCTURE DEFINITIONS AS PER THE MANUAL          }
****************************************************************************/

/*--------------------------------------------------------------------------}
{		Many parts of USB2.0 standard use this bit field for direction 	    }
{---------------------------------------------------------------------------}*/
enum UsbDirection {
    USB_DIRECTION_OUT = 0,											// Host to device
    USB_DIRECTION_IN = 1,											// Device to Host
};

/*--------------------------------------------------------------------------}
{	 Many parts of USB2.0 standard use this 2 bit field for speed control   }
{---------------------------------------------------------------------------}*/
enum UsbSpeed : uint8_t {
    USB_SPEED_HIGH = 0,												// USB high speed
    USB_SPEED_FULL = 1,												// USB full speed
    USB_SPEED_LOW = 2,												// USB low speed
};
constexpr char const* SpeedString[3] = { "High", "Full", "Low" };

/*--------------------------------------------------------------------------}
{			 Transfer types as layed out in USB 2.0 standard			    }
{---------------------------------------------------------------------------}*/
enum usb_transfer_type {
    USB_TRANSFER_TYPE_CONTROL = 0,
    USB_TRANSFER_TYPE_ISOCHRONOUS = 1,
    USB_TRANSFER_TYPE_BULK = 2,
    USB_TRANSFER_TYPE_INTERRUPT = 3,
};

/*--------------------------------------------------------------------------}
{			 Transfer sizes as layed out in USB 2.0 standard			    }
{---------------------------------------------------------------------------}*/
enum usb_transfer_size {
    USB_TRANSFER_SIZE_8_BIT = 0,
    USB_TRANSFER_SIZE_16_BIT = 1,
    USB_TRANSFER_SIZE_32_BIT = 2,
    USB_TRANSFER_SIZE_64_BIT = 3,
};

/*--------------------------------------------------------------------------}
{	 USB description types as per Table 9-5 in Section 9.4 of USB2.0 spec	}
{---------------------------------------------------------------------------}*/
enum usb_descriptor_type : uint8_t {
    USB_DESCRIPTOR_TYPE_DEVICE = 1,
    USB_DESCRIPTOR_TYPE_CONFIGURATION = 2,
    USB_DESCRIPTOR_TYPE_STRING = 3,
    USB_DESCRIPTOR_TYPE_INTERFACE = 4,
    USB_DESCRIPTOR_TYPE_ENDPOINT = 5,
    USB_DESCRIPTOR_TYPE_QUALIFIER = 6,
    USB_DESCRIPTOR_TYPE_OTHERSPEED_CONFIG = 7,
    USB_DESCRIPTOR_TYPE_INTERFACE_POWER = 8,
    USB_DESCRIPTOR_TYPE_HID = 33,
    USB_DESCRIPTOR_TYPE_HID_REPORT = 34,
    USB_DESCRIPTOR_TYPE_HID_PHYSICAL = 35,
    USB_DESCRIPTOR_TYPE_HUB = 41,
};

/*--------------------------------------------------------------------------}
{		 Enumeration Status defined in 9.1 of USB 2.0 standard			    }
{---------------------------------------------------------------------------}*/
enum UsbDeviceStatus : uint8_t {
    USB_STATUS_ATTACHED = 0,										// USB status is attached
    USB_STATUS_POWERED = 1,											// USB status is powered
    USB_STATUS_DEFAULT = 2,											// USB status is default
    USB_STATUS_ADDRESSED = 3,										// USB status is addressed
    USB_STATUS_CONFIGURED = 4,										// USB status is configured
};

/*--------------------------------------------------------------------------}
{		Hub Port Features that can be changed in the USB 2.0 standard	    }
{---------------------------------------------------------------------------}*/
enum HubPortFeature {
    FeatureConnection = 0,
    FeatureEnable = 1,
    FeatureSuspend = 2,
    FeatureOverCurrent = 3,
    FeatureReset = 4,
    FeaturePower = 8,
    FeatureLowSpeed = 9,
    FeatureHighSpeed = 10,
    FeatureConnectionChange = 16,
    FeatureEnableChange = 17,
    FeatureSuspendChange = 18,
    FeatureOverCurrentChange = 19,
    FeatureResetChange = 20,
};

/*--------------------------------------------------------------------------}
{		   Hub Gateway Node Features defined in the USB 2.0 standard		}
{---------------------------------------------------------------------------}*/
enum HubFeature {
    FeatureHubPower = 0,
    FeatureHubOverCurrent = 1,
};

/*--------------------------------------------------------------------------}
{	  Device Request structure (8 bytes) as per the USB 2.0 standard		}
{---------------------------------------------------------------------------}*/
enum UsbDeviceRequestRequest : uint8_t {
    // USB requests
    GetStatus = 0,
    ClearFeature = 1,
    SetFeature = 3,
    SetAddress = 5,
    GetDescriptor = 6,
    SetDescriptor = 7,
    GetConfiguration = 8,
    SetConfiguration = 9,
    GetInterface = 10,
    SetInterface = 11,
    SynchFrame = 12,
    // HID requests
    GetReport = 1,
    GetIdle = 2,
    GetProtocol = 3,
    SetReport = 9,
    SetIdle = 10,
    SetProtocol = 11,
};
struct UsbDeviceRequest {
    uint8_t Type;													// +0x0
    UsbDeviceRequestRequest Request;								// +0x1
    uint16_t Value;													// +0x2 
    uint16_t Index;													// +0x4
    uint16_t Length;												// +0x6
};

static_assert(sizeof(UsbDeviceRequest) == 0x08, "Structure should be 8 bytes");


/*--------------------------------------------------------------------------}
{	         USB description header as per 9.6 of the USB2.0				}
{---------------------------------------------------------------------------}*/
struct UsbDescriptorHeader {
    uint8_t DescriptorLength;										// +0x0
    usb_descriptor_type DescriptorType;								// +0x1
};

static_assert(sizeof(UsbDescriptorHeader) == 0x02, "Structure should be 2 bytes");


/*--------------------------------------------------------------------------}
{	         USB class id as per 9.6.1 of USB2.0 manual enumerated			}
{---------------------------------------------------------------------------}*/
enum DeviceClass {
    DeviceClassInInterface = 0x00,
    DeviceClassCommunications = 0x2,
    DeviceClassHub = 0x9,
    DeviceClassDiagnostic = 0xdc,
    DeviceClassMiscellaneous = 0xef,
    DeviceClassVendorSpecific = 0xff,
};

 /*--------------------------------------------------------------------------}
 {	   USB device descriptor .. Table 9-8 in 9.6.1 of the USB 2.0 spec	 	 }
 {---------------------------------------------------------------------------}*/
struct DeviceDescriptor {
    uint8_t  bLength;												// +0x0 Length of this descriptor
    uint8_t  bDescriptorType;										// +0x1 Descriptor type
    uint16_t bcdUSB;												// +0x2 (in BCD 0x210 = USB2.10)
    uint8_t  bDeviceClass;											// +0x4 Class code (enum DeviceClass )
    uint8_t  bDeviceSubClass;										// +0x5 Subclass code (assigned by the USB-IF)
    uint8_t  bDeviceProtocol;										// +0x6 Protocol code (assigned by the USB-IF)
    uint8_t  bMaxPacketSize0;										// +0x7 Maximum packet size for endpoint 0
    uint16_t idVendor;												// +0x8 Vendor ID (assigned by the USB-IF)
    uint16_t idProduct;												// +0xa Product ID (assigned by the manufacturer)
    uint16_t bcdDevice;												// +0xc Device version number (BCD)
    uint8_t  iManufacturer;											// +0xe Index of String Descriptor describing the manufacturer.
    uint8_t  iProduct;												// +0xf Index of String Descriptor describing the product
    uint8_t  iSerialNumber;											// +0x10 Index of String Descriptor with the device's serial number
    uint8_t  bNumConfigurations;									// +0x11 Number of possible configurations
};

static_assert(sizeof(DeviceDescriptor) == 0x12, "DeviceDescriptor must be 18 bytes long");

/*--------------------------------------------------------------------------}
{	  USB device configuration descriptor as per 9.6.3 of USB2.0 manual		}
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) ConfigurationDescriptor {
    uint8_t  bLength;												// +0x0 Length of this descriptor
    uint8_t  bDescriptorType;										// +0x1 DEVICE descriptor type(enum DescriptorType)
    uint16_t wTotalLength;											// +0x2 Total length of all descriptors for this configuration
    uint8_t  bNumInterfaces;										// +0x4 Number of interfaces in this configuration
    uint8_t  bConfigurationValue;									// +0x5 Value of this configuration (1 based)
    uint8_t  iConfiguration;										// +0x6 Index of String Descriptor describing the configuration
    union {
        uint8_t  bmAttributes;										// +0x7 Configuration characteristics
        struct {
            uint8_t _reserved0_4 : 5;								// @0
            uint8_t RemoteWakeup : 1;								// @5
            uint8_t SelfPowered : 1;								// @6
            uint8_t _reserved7 : 1;									// @7
        };
    };
    uint8_t  bMaxPower;												// +0x8 Maximum power consumed by this configuration
};

static_assert(sizeof(ConfigurationDescriptor) == 9, "ConfigurationDescriptor must be 9 bytes long");

/*--------------------------------------------------------------------------}
{  USB other speed configuration descriptor as per 9.6.4 of USB2.0 manual   }
{---------------------------------------------------------------------------}*/
//struct __attribute__((__packed__)) UsbOtherSpeedConfigurationDescriptor {
//	UsbDescriptorHeader Header;								// +0x0 Length of this descriptor, +0x1 DEVICE descriptor type (enum DescriptorType)
//	uint16_t TotalLength;											// +0x2 Total length of all descriptors for this configuration
//	uint8_t InterfaceCount;											// +0x4 Number of interfaces in this configuration
//	uint8_t ConfigurationValue;										// +0x5 Value of this configuration (1 based)
//	uint8_t StringIndex;											// +0x6 Index of String Descriptor describing the configuration
//	struct __attribute__((__packed__, aligned(1))) {
//		unsigned _reserved0_4 : 5;						// @0
//		unsigned RemoteWakeup : 1;						// @5
//		unsigned SelfPowered : 1;						// @6
//		enum {
//			Valid = 1,
//		} _reserved7 : 1;								// @7
//	} Attributes;													// +0x7 Configuration characteristics
//	uint8_t MaximumPower;											// +0x8 Maximum power consumed by this configuration
//};

/*--------------------------------------------------------------------------}
{      USB interface descriptor structure as per 9.6.5 of USB2.0 manual     }
{---------------------------------------------------------------------------}*/
enum class InterfaceClass : uint8_t {
    Reserved            = 0x00,
    Audio               = 0x01,
    Communications      = 0x02,
    Hid                 = 0x03,
    Physical            = 0x05,
    Image               = 0x06,
    Printer             = 0x07,
    MassStorage         = 0x08,
    Hub                 = 0x09,
    CdcData             = 0x0a,
    SmartCard           = 0x0b,
    ContentSecurity     = 0x0d,
    Video               = 0x0e,
    PersonalHealthcare  = 0x0f,
    AudioVideo          = 0x10,
    DiagnosticDevice    = 0xdc,
    WirelessController  = 0xe0,
    Miscellaneous       = 0xef,
    ApplicationSpecific = 0xfe,
    VendorSpecific      = 0xff,
};
struct __attribute__((__packed__)) UsbInterfaceDescriptor {
    UsbDescriptorHeader Header;    							// +0x0 Length of this descriptor, +0x1 DEVICE descriptor type (enum DescriptorType)
    uint8_t             Number;								// +0x2 Number of this interface (0 based).
    uint8_t             AlternateSetting;					// +0x3 Value of this alternate interface setting
    uint8_t             EndpointCount;						// +0x4 Number of endpoints in this interface
    InterfaceClass      Class;								// +x05 Class code (assigned by the USB-IF)
    uint8_t             SubClass;							// +x06 Subclass code (assigned by the USB-IF)
    uint8_t             Protocol;							// +x07 Protocol code (assigned by the USB-IF)
    uint8_t             StringIndex;						// +x08 Index of String Descriptor describing the interface
};

static_assert(sizeof(UsbInterfaceDescriptor) == 0x09, "Structure should be 9 bytes");


/*--------------------------------------------------------------------------}
{ USB endpoint descriptor structure (7 Bytes) as per 9.6.6 of USB2.0 manual }
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) UsbEndpointDescriptor {
    UsbDescriptorHeader Header;								// +0x0 Length of this descriptor, +0x1 DEVICE descriptor type (enum DescriptorType)
    struct __attribute__((__packed__, aligned(1))) {
        uint8_t Number : 4;					    		// @0
        uint8_t _reserved4_6 : 3;						// @4
        UsbDirection Direction : 1;							// @7
    } EndpointAddress;												// +0x2  Endpoint address. Bit 7 indicates direction (0=OUT, 1=IN).
    struct __attribute__((__packed__, aligned(1))) {
        enum usb_transfer_type Type : 2;				// @0
        enum {
            NoSynchronisation = 0,
            Asynchronous = 1,
            Adaptive = 2,
            Synchrouns = 3,
        } Synchronisation : 2;							// @2
        enum {
            Data = 0,
            Feeback = 1,
            ImplicitFeebackData = 2,
        } Usage : 2;									// @4
        unsigned _reserved6_7 : 2;						// @6
    } Attributes;													// +0x3 Endpoint transfer type
    struct __attribute__((__packed__, aligned(1))) {
        uint16_t MaxSize : 11;							// @0
        enum : uint16_t {
            None = 0,
            Extra1 = 1,
            Extra2 = 2,
        } Transactions : 2;								// @11
        uint16_t _reserved13_15 : 3;					// @13
    } Packet;														// +0x4 Maximum packet size.
    uint8_t Interval;												// +0x6 Polling interval in frames
};

static_assert(sizeof(UsbEndpointDescriptor) == 0x07, "Structure should be 7 bytes");


/*--------------------------------------------------------------------------}
{       USB string descriptor structure as per 9.6.7 of USB2.0 manual       }
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) UsbStringDescriptor {
    UsbDescriptorHeader Header;								// +0x0 Length of this descriptor, +0x1 DEVICE descriptor type (enum DescriptorType)
    uint16_t Data[];												// +0x2 Amount varies with string length
};

/*--------------------------------------------------------------------------}
{ 	   USB HUB descriptor (9 Bytes) as per 11.23.2.1 of USB2.0 manual		}
{---------------------------------------------------------------------------}*/
enum HubPortControl {
    Global = 0,
    Individual = 1,
};
struct __attribute__((__packed__)) HubDescriptor {
    struct UsbDescriptorHeader Header;								// +0x0 Length of this descriptor, +0x1 DEVICE descriptor type (enum DescriptorType)
    uint8_t PortCount;												// +0x2
    struct __attribute__((__packed__, aligned(1))) {
        HubPortControl PowerSwitchingMode : 2;			// @0
        unsigned Compound : 1;							// @2
        enum HubPortControl OverCurrentProtection : 2;	// @3
        unsigned ThinkTime : 2;							// @5
        unsigned Indicators : 1;						// @7
        unsigned _reserved8_15 : 8;						// @8
    } Attributes;													// +0x3
    uint8_t PowerGoodDelay;											// +0x5
    uint8_t MaximumHubPower;										// +0x6
    struct __attribute__((__packed__, aligned(1))) {
        unsigned Reserved0 : 1;							// @0
        unsigned Port1 : 1;								// @1
        unsigned Port2 : 1;								// @2
        unsigned Port3 : 1;								// @3
        unsigned Port4 : 1;								// @4
        unsigned Reserved1 : 3;							// @5-8
    } DeviceRemovable;												// +0x7
    uint8_t PortPowerCtrlMask;										// +0x8
};

static_assert(sizeof(HubDescriptor) == 0x09, "Structure should be 9 bytes");


/*--------------------------------------------------------------------------}
{ 	     USB HUB status (16 bits) as per 11.24.2.6 of USB2.0 manual			}
{---------------------------------------------------------------------------}*/
union HubStatus
{
    struct PACKED
    {
        uint16_t LocalPower    :  1; // @0
        uint16_t OverCurrent   :  1; // @1
        uint16_t _reserved2_15 : 14; // @2
    };
    uint16_t Raw16;
};

static_assert(sizeof(HubStatus) == 0x02, "Structure should be 16 bits (2 bytes)");


/*--------------------------------------------------------------------------}
{ 	  USB HUB status change (16 Bits) as per 11.24.2.6 of USB2.0 manual		}
{---------------------------------------------------------------------------}*/
union HubStatusChange
{
    struct PACKED
    {
        uint16_t LocalPowerChanged  :  1; // @0
        uint16_t OverCurrentChanged :  1; // @1
        uint16_t _reserved2_15      : 14; // @2
    };
    uint16_t Raw16;
};

static_assert(sizeof(HubStatusChange) == 0x02, "Structure should be 16 bits (2 bytes)");

/*--------------------------------------------------------------------------}
{ 	    USB HUB full status (32 Bits) as per 11.24.2.6 of USB2.0 manual		}
{---------------------------------------------------------------------------}*/
union HubFullStatus
{
    struct PACKED
    {
        HubStatus       Status;
        HubStatusChange Change;
    };
    uint32_t Raw32;
};

static_assert(sizeof(HubFullStatus) == 0x04, "Structure should be 32bits (4 bytes)");


/*--------------------------------------------------------------------------}
{ 	USB HUB status structure (16 bits) as per 11.24.2.7.1 of USB2.0 manual  }
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) HubPortStatus {
    unsigned Connected : 1;									// @0
    unsigned Enabled : 1;									// @1
    unsigned Suspended : 1;									// @2
    unsigned OverCurrent : 1;								// @3
    unsigned Reset : 1;										// @4
    unsigned _reserved5_7 : 3;								// @5
    unsigned Power : 1;										// @8
    unsigned LowSpeedAttatched : 1;							// @9
    unsigned HighSpeedAttatched : 1;						// @10
    unsigned TestMode : 1;									// @11
    unsigned IndicatorControl : 1;							// @12
    unsigned _reserved13_15 : 3;							// @13
} ;

/*--------------------------------------------------------------------------}
{ USB HUB status change structure (16 Bits) as 11.24.2.7.2 of USB2.0 manual }
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) HubPortStatusChange {
    unsigned ConnectedChanged : 1;							// @0
    unsigned EnabledChanged : 1;							// @1
    unsigned SuspendedChanged : 1;							// @2
    unsigned OverCurrentChanged : 1;						// @3
    unsigned ResetChanged : 1;								// @4
    unsigned _reserved5_15 : 11;							// @5
};


/*--------------------------------------------------------------------------}
{ 	USB HUB full status structure (32 Bits) per 11.24.2.7 of USB2.0 manual  }
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) HubPortFullStatus {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            union {
                struct HubPortStatus Status;						// 16 bit status as a hub port status structure
                uint16_t RawStatus;									// The same 16 bit status as raw bits
            };
            union {
                struct HubPortStatusChange Change;					// 16 bit change status as a hub port chnage structure
                uint16_t RawChange;									// The same 16  bit change status as raw bits
            };
        };
        uint32_t Raw32;												// Both status joined as one raw 32 bits
    };
};

static_assert(sizeof(HubPortFullStatus) == 0x04, "Structure should be 32bits (4 bytes)");


/*--------------------------------------------------------------------------}
{ USB struct UsbDeviceRequest .Type Bit masks to use to make full bitmask   }
{---------------------------------------------------------------------------}*/
constexpr uint8_t USB_SETUP_HOST_TO_DEVICE      = 0x00;    // Device Request bmRequestType transfer direction - host to device transfer
constexpr uint8_t USB_SETUP_DEVICE_TO_HOST      = 0x80;    // Device Request bmRequestType transfer direction - device to host transfer
constexpr uint8_t USB_SETUP_TYPE_STANDARD       = 0x00;    // Device Request bmRequestType type - standard
constexpr uint8_t USB_SETUP_TYPE_CLASS          = 0x20;    // Device Request bmRequestType type - class
constexpr uint8_t USB_SETUP_TYPE_VENDOR         = 0x40;    // Device Request bmRequestType type - vendor
constexpr uint8_t USB_SETUP_RECIPIENT_DEVICE    = 0x00;    // Device Request bmRequestType recipient - device
constexpr uint8_t USB_SETUP_RECIPIENT_INTERFACE = 0x01;    // Device Request bmRequestType recipient - interface
constexpr uint8_t USB_SETUP_RECIPIENT_ENDPOINT  = 0x02;    // Device Request bmRequestType recipient - endpoint
constexpr uint8_t USB_SETUP_RECIPIENT_OTHER		= 0x03;	   // Device Request bmRequestType recipient - other

/*--------------------------------------------------------------------------}
{ 		  USB struct UsbDeviceRequest .Type Bit masks for a HUB			    }
{---------------------------------------------------------------------------}*/
constexpr uint8_t bmREQ_HUB_FEATURE		      = USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_DEVICE;
constexpr uint8_t bmREQ_PORT_FEATURE		  = USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER;
constexpr uint8_t bmREQ_HUB_STATUS			  = USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_DEVICE;
constexpr uint8_t bmREQ_PORT_STATUS           = USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_OTHER;
constexpr uint8_t bmREQ_GET_HUB_DESCRIPTOR    = USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_DEVICE;
constexpr uint8_t bmREQ_SET_HUB_DESCRIPTOR    = USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_DEVICE;


constexpr uint8_t bmREQ_DEVICE_STATUS		  = USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE;
constexpr uint8_t bmREQ_GET_DEVICE_DESCRIPTOR = USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE;
constexpr uint8_t bmREQ_SET_DEVICE_DESCRIPTOR = USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE;

constexpr uint8_t bmREQ_INTERFACE_FEATURE	  = USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_INTERFACE;
constexpr uint8_t bmREQ_INTERFACE_STATUS	  = USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_INTERFACE;

constexpr uint8_t bmREQ_ENDPOINT_FEATURE	  = USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_ENDPOINT;
constexpr uint8_t bmREQ_ENDPOINT_STATUS		  = USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_ENDPOINT;

enum PacketId {
    USB_PID_DATA0 = 0,
    USB_PID_DATA1 = 2,
    USB_PID_DATA2 = 1,
    USB_PID_SETUP = 3,
    USB_MDATA = 3,
};
