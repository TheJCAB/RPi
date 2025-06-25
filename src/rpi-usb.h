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

#include <array>

#include <stdint.h>
#include "emb-stdio.h"				// Needed for printf

//#define LOG(...)
#define LOG(...) printf2(__VA_ARGS__)
#define LOG_DEBUG(...)
//#define LOG_DEBUG(...) printf2(__VA_ARGS__)


enum UsbPacketSize {
    Bits8 = 0,
    Bits16 = 1,
    Bits32 = 2,
    Bits64 = 3,
};


static inline UsbPacketSize SizeFromNumber(uint32_t size) {
    if (size <= 8) return Bits8;
    else if (size <= 16) return Bits16;
    else if (size <= 32) return Bits32;
    else return Bits64;
}

static inline uint32_t SizeToNumber(UsbPacketSize size) {
    if (size == Bits8) return 8;
    else if (size == Bits16) return 16;
    else if (size == Bits32) return 32;
    else return 64;
}

#define MaximumDevices 32											// Max number of devices with a USB node we will allow 


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

#define MaxHIDPerDevice 4

    
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
enum UsbSpeed {
    USB_SPEED_HIGH = 0,												// USB high speed
    USB_SPEED_FULL = 1,												// USB full speed
    USB_SPEED_LOW = 2,												// USB low speed
};
extern const char* SpeedString[3];	// Speed strings High, Low, Full provided as constants 

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
enum UsbDeviceStatus {
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

/*--------------------------------------------------------------------------}
{	         USB description header as per 9.6 of the USB2.0				}
{---------------------------------------------------------------------------}*/
struct UsbDescriptorHeader {
    uint8_t DescriptorLength;										// +0x0
    usb_descriptor_type DescriptorType;								// +0x1
};

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
enum class InterfaceClass : uint8_t{
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

/*--------------------------------------------------------------------------}
{ USB endpoint descriptor structure (7 Bytes) as per 9.6.6 of USB2.0 manual }
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) UsbEndpointDescriptor {
    UsbDescriptorHeader Header;								// +0x0 Length of this descriptor, +0x1 DEVICE descriptor type (enum DescriptorType)
    struct __attribute__((__packed__, aligned(1))) {
        unsigned Number : 4;							// @0
        unsigned _reserved4_6 : 3;						// @4
        unsigned Direction : 1;							// @7
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
        unsigned MaxSize : 11;							// @0
        enum {
            None = 0,
            Extra1 = 1,
            Extra2 = 2,
        } Transactions : 2;								// @11
        unsigned _reserved13_15 : 3;					// @13
    } Packet;														// +0x4 Maximum packet size.
    uint8_t Interval;												// +0x6 Polling interval in frames
};

/*--------------------------------------------------------------------------}
{       USB string descriptor structure as per 9.6.7 of USB2.0 manual       }
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) UsbStringDescriptor {
    struct UsbDescriptorHeader Header;								// +0x0 Length of this descriptor, +0x1 DEVICE descriptor type (enum DescriptorType)
    uint16_t Data[];												// +0x2 Amount varies with string length
};

template < size_t N >
struct __attribute__((__packed__)) UsbStringDescriptorT {
    struct UsbDescriptorHeader Header;								// +0x0 Length of this descriptor, +0x1 DEVICE descriptor type (enum DescriptorType)
    // This is a template for a string descriptor with a fixed size.
    // It allows us to create a string descriptor with a specific length.
    static constexpr size_t Length = N;
    char16_t Data[N];											// +0x2 Amount varies with string length

    operator UsbStringDescriptor const&() const {
        // Convert this literal descriptor to a UsbStringDescriptor.
        // This is useful for passing the descriptor to functions that expect a UsbStringDescriptor.
        return *reinterpret_cast<UsbStringDescriptor const*>(this);
    }
};

template < size_t N >
constexpr auto LiteralUsbStringDescriptor(char16_t const (&str)[N]) {
    // Create a string descriptor from a string literal.
    // The string literal must be null-terminated.
    UsbStringDescriptorT<N> desc;
    desc.Header.DescriptorLength = (N + 1) * 2;
    desc.Header.DescriptorType = USB_DESCRIPTOR_TYPE_STRING;
    for (size_t i = 0; i < N; ++i) {
        desc.Data[i] = str[i];
    }
    return desc;
}

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

/*--------------------------------------------------------------------------}
{ 	     USB HUB status (16 bits) as per 11.24.2.6 of USB2.0 manual			}
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) HubStatus {
    unsigned LocalPower : 1;							// @0
    unsigned OverCurrent : 1;							// @1
    unsigned _reserved2_15 : 14;						// @2
};

/*--------------------------------------------------------------------------}
{ 	  USB HUB status change (16 Bits) as per 11.24.2.6 of USB2.0 manual		}
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) HubStatusChange {
    unsigned LocalPowerChanged : 1;						// @0
    unsigned OverCurrentChanged : 1;					// @1
    unsigned _reserved2_15 : 14;						// @2
};

/*--------------------------------------------------------------------------}
{ 	    USB HUB full status (32 Bits) as per 11.24.2.6 of USB2.0 manual		}
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) HubFullStatus {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            union {
                struct HubStatus Status;							// 16 bit hub status as hub status structure
                uint16_t RawStatus;									// The same 16 bit status as raw bits
            };
            union {
                struct HubStatusChange Change;						// 16 bit change status as a hub port chnage structure
                uint16_t RawChange;									// The same 16  bit change status as raw bits
            };
        };
        uint32_t Raw32;												// Both status joined as one raw 32 bits
    };
};

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
{ 		 USB HID 1.11 descriptor structure as per manual in 6.2.1		    }
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) HidDescriptor {
    struct UsbDescriptorHeader Header;								// +0x0 Length of this descriptor, +0x1 DEVICE descriptor type (enum DescriptorType)
    union {															// Place a union over BCD version .. alignment issues on ARM7/8
        struct __attribute__((__packed__, aligned(1))) {
            uint8_t HidVersionLo;									// Lo of BCD version
            uint8_t HidVersionHi;									// Hi of BCD version
        };
        uint16_t HidVersion;										// (bcd version) +0x2 
    };
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
    union {															// Place a union over length .. alignment issues on ARM7/8
        struct __attribute__((__packed__, aligned(1))) {
            uint8_t LengthLo;										// Lo of Length
            uint8_t LengthHi;										// Hi of Length
        };
        uint16_t Length;											// +0x7 
    };
};


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
constexpr uint8_t bmREQ_GET_HUB_DESCRIPTOR    = USB_SETUP_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_DEVICE;
constexpr uint8_t bmREQ_SET_HUB_DESCRIPTOR    = USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_DEVICE;


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

/***************************************************************************}
{             PUBLIC USB STRUCTURES DEFINITIONS DEFINED BY US				}
****************************************************************************/

/*--------------------------------------------------------------------------}
{ 	USB pipe our own special structure encompassing a pipe in the USB spec	}
{---------------------------------------------------------------------------}*/
struct __attribute__((__packed__)) UsbPipe {
    UsbPacketSize MaxSize : 2;										// @0		Maximum packet size
    UsbSpeed Speed : 2;												// @2		Speed of device
    unsigned EndPoint : 4;											// @4		Endpoint address
    unsigned Number : 8;											// @8		Unique device number sometimes called address or id
    unsigned _reserved : 2;											// @16-17
    unsigned lowSpeedNodePort : 7;									// @18-24		In low speed transfers it is port device is on closest parent high speed hub
    unsigned lowSpeedNodePoint : 7;									// @25-31	In low speed transfers it is closest parent high speed hub
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
    unsigned ConfigIndex : 8;										// @0 Current set config index
    unsigned ConfigStringIndex : 8;									// @8 Current config string index
    enum UsbDeviceStatus Status : 8;								// @16 Device enumeration status .. USB_ATTACHED, USB_POWERED, USB_ADDRESSED, etc
    unsigned reserved : 8;											// @24-31
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
    UsbPipeControl PipeCtrl0;				// Usb device pipe control AKA pipectrl0
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
{	 USB hid structure which is just extra data attached to a USB node	    }
{---------------------------------------------------------------------------}*/
struct HidDevice {
    HidDescriptor Descriptor[MaxHIDPerDevice];	// HID descriptor of this device
    uint8_t HIDInterface[MaxHIDPerDevice];				// The interface the HID descriptor is on
    uint8_t MaxHID ALIGN4;								// Maxiumum HID in array (usually less than the max array size) .. align it for ARM7/8
};

/*--------------------------------------------------------------------------}
{	USB mass storage structure which is extra data attached to a USB node   }
{---------------------------------------------------------------------------}*/
struct MassStorageDevice {
    uint8_t SCSI;
};

#endif						// end RPI_USB guard
