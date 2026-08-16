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

#include "DesignWareUsb.h"
#include "DesignWareUsbHost.h"
#include "DesignWareUsbChannel.h"

#include "Cpu.h"
#include "Timer.h"

#include "emb-stdio.h"				// Needed for printf

#include <string.h>
#include <wchar.h>
#include <vector>
#include <expected>
#include <algorithm>
#include <optional>

#include <span>

#define LOG(...)
//#define LOG(...) printf(__VA_ARGS__)
#define LOG_DEBUG(...)
//#define LOG_DEBUG(...) printf(__VA_ARGS__)

/*--------------------------------------------------------------------------}
{	  Forward declare our USB device types which form our device tree		}
{---------------------------------------------------------------------------}*/
struct UsbDevice;			// Single device endpoint
struct HubDevice;			// Hub connects to multiple other devices so we get a tree as well as being an endpoint itself
struct HidDevice;			// Single device endpoint which is a human interface 
struct MassStorageDevice;	// Single device endpoint which is a mass storage device 

HidDevice* AllocateHidPayload();
void FreeHidPayload(HidDevice* device);
uint8_t GetHidCount(HidDevice* device);
bool SetHidDescriptor(HidDevice* hidDevice, uint8_t hidIndex, uint8_t interface, std::byte const* buffer, uint8_t size);
void PrintHid(HidDevice* device, uint8_t hidIndex, char const* indent);
void BindHidOwner(HidDevice* hidDevice, std::shared_ptr<UsbDevice> const& device);

/*-INTERNAL: EnumerateHID------------------------------------------------------
If normal device enumeration detects a hid device, after normal single node
enumeration it will call this procedure to enumerate connected HID devices.
11Feb17 LdB
--------------------------------------------------------------------------*/
Async::task<RESULT> EnumerateHID(UsbDevice& device);

#define ALIGN4 __attribute__((aligned(4)))			// Alignment attribute shortcut macro .. I hate the attribute text length nothing tricky

/*--------------------------------------------------------------------------}
{	USB mass storage structure which is extra data attached to a USB node   }
{---------------------------------------------------------------------------}*/
struct MassStorageDevice {
    uint8_t SCSI;
};

class DesignWareUsbDriver : public UsbDriver
{

    std::shared_ptr<HCDHost> Host;

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

    IoHandle GetIoHandle(uint8_t deviceAddress) override
    {
        if (deviceAddress == 0 || deviceAddress > DeviceTable.size())
        {
            return {};
        }
        auto& devicePtr = DeviceTable[deviceAddress - 1];
        if (!devicePtr)
        {
            return {};
        }
        return IoHandle(Host->GetChannel().release(), IoHandleDeleter{ this });
    }

    void DeleteIoHandle(void* ptr) override
    {
        HCDHost::ReleaseChannel{}(static_cast<HCDChannel*>(ptr));
    }

    std::vector<std::shared_ptr<UsbDevice>> DeviceTable = {};
    std::vector<std::shared_ptr<HubDevice>> HubTable = {};

    std::shared_ptr<UsbDriver> DriverRef()
    {
        return std::shared_ptr<UsbDriver>(this, [](UsbDriver*) {});
    }

    std::shared_ptr<UsbDevice> SharedHandleFor(UsbDevice& device)
    {
        for (auto const& entry : DeviceTable)
        {
            if (entry && entry.get() == &device)
            {
                return entry;
            }
        }
        return {};
    }

    std::generator<UsbDevice&> EnumerateDevices() override
    {
        for (auto&& device : DeviceTable)
        {
            if (device)
            {
                co_yield *device;
            }
        }
    }


    /*-HCDSubmitControlMessage --------------------------------------------------
    Sends a control message to a device. Handles all necessary channel creation
    and other processing. The sequence of a control transfer is defined in the
    USB 2.0 manual section 5.5.  Success is indicated by return of RESULT::Ok (0) all
    other codes indicate an error.
    24Feb17 LdB
    --------------------------------------------------------------------------*/
    Async::task<RESULT> HCDSubmitControlMessage (UsbDevice* device,
                                    IoHandle const& ioHandle,
                                    std::span<std::byte> buffer,					// Data buffer both send and recieve				 
                                    UsbDeviceRequest request,	// USB request message
                                    uint32_t timeout,					// Timeout in microseconds on message
                                    uint32_t* bytesTransferred)			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
    {
        DWCRESULT result;

        LOG_DEBUG("HCD: SETUP packet to device: %#x req: %#x req Type: %#x Speed: %i PacketSize: %i LowNode: %i LowPort: %i\n",
            device->Pipe0.Number, request.Request, request.Type, device->Pipe0.Speed, device->Pipe0.MaxPacketSizeInBytes, device->Pipe0.splitNodePoint, device->Pipe0.splitNodePort);

        uint32_t lastTransfer = 0;

        auto& channel = *static_cast<HCDChannel*>(ioHandle.get());

        UsbDirection Direction = (request.Type & 0x80) ? USB_DIRECTION_IN : USB_DIRECTION_OUT;

        LOG_DEBUG("Setup phase\n");
        // Setup phase
        uint32_t transferLength = co_await channel.TransferOut(device->Pipe0, USB_TRANSFER_TYPE_CONTROL, { (std::byte const*)&request, sizeof(request) }, USB_PID_SETUP);
        if (transferLength != sizeof(request))
        {
            LOG("HCD: SETUP packet to device: %#x req: %#x req Type: %#x Speed: %i PacketSize: %i LowNode: %i LowPort: %i Error: %zu != %zu\n",
                device->Pipe0.Number, request.Request, request.Type, device->Pipe0.Speed, device->Pipe0.MaxPacketSizeInBytes, device->Pipe0.splitNodePoint, device->Pipe0.splitNodePort, transferLength, sizeof(request));
            co_return ResultFromDwcResult(result);
        }

        if (buffer.empty())
        {
            if (Direction == USB_DIRECTION_IN) {
                LOG("HCD: No buffer provided for IN transfer to device %i.\n", device->Pipe0.Number);
                co_return RESULT::ErrorArgument;
            }

            lastTransfer = 0;

            // Nothing more to send, just receive the status.
            LOG_DEBUG("No transfer phase, just status phase\n");
            co_await channel.TransferIn(device->Pipe0, USB_TRANSFER_TYPE_CONTROL, {}, USB_PID_DATA1);
        }
        else if (Direction == USB_DIRECTION_OUT)
        {
            LOG_DEBUG("Transfer phase\n");
            lastTransfer = co_await channel.TransferOut(device->Pipe0, USB_TRANSFER_TYPE_CONTROL, buffer, USB_PID_DATA1);
            if (lastTransfer != buffer.size())
            {
                LOG("HCD: OUT transfer to device %i failed, expected %zu bytes, got %u bytes.\n",
                    device->Pipe0.Number, buffer.size(), lastTransfer);
                co_return RESULT::ErrorGeneral; // Some parameter or communication issue.
            }
            LOG_DEBUG("Status phase\n");
            co_await channel.TransferIn(device->Pipe0, USB_TRANSFER_TYPE_CONTROL, {}, USB_PID_DATA1);
        }
        else
        {
            LOG_DEBUG("Transfer phase\n");
            lastTransfer = co_await channel.TransferIn(device->Pipe0, USB_TRANSFER_TYPE_CONTROL, buffer, USB_PID_DATA1);
            if (lastTransfer != buffer.size())
            {
                LOG("HCD: IN transfer to device %i failed, expected %zu bytes, got %u bytes.\n",
                    device->Pipe0.Number, buffer.size(), lastTransfer);
                co_return RESULT::ErrorGeneral; // Some parameter or communication issue.
            }
            LOG_DEBUG("Status phase\n");
            co_await channel.TransferOut(device->Pipe0, USB_TRANSFER_TYPE_CONTROL, {}, USB_PID_DATA1);
        }

        if (bytesTransferred) *bytesTransferred = lastTransfer;
        co_return RESULT::Ok;
    }

    using UsbDriver::HCDSubmitControlMessageOUT;
    using UsbDriver::HCDSubmitControlMessageIN;

    Async::task<RESULT> HCDSubmitControlMessageOUT(
        UsbDevice* device,
        IoHandle const& ioHandle,
        std::span<std::byte const> buffer, // Data buffer to send
        UsbDeviceRequest request,	// USB request message
        uint32_t timeout,					// Timeout in microseconds on message
        uint32_t* bytesTransferred			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
    ) override
    {
        if (request.Type & 0x80) {
            LOG("HCDSubmitControlMessageOUT called with IN request type: %#x\n", request.Type);
            co_return RESULT::ErrorArgument;
        }

        // Just returning the task from HCDSubmitControlMessage is tempting, but...
        // In order to respect the lifetime of the channel, we need this to be a proper coroutine.
        co_return co_await HCDSubmitControlMessage(
            device,
            ioHandle,
            { const_cast<std::byte*>(buffer.data()), buffer.size() }, // Cast away constness for the transfer, as the underlying API expects a non-const span
            request,
            timeout,
            bytesTransferred
        );
    }

    Async::task<RESULT> HCDSubmitControlMessageIN(
        UsbDevice* device,
        IoHandle const& ioHandle,
        std::span<std::byte> buffer,					// Data buffer both send and recieve				 
        UsbDeviceRequest request,	// USB request message
        uint32_t timeout,					// Timeout in microseconds on message
        uint32_t* bytesTransferred			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
    ) override
    {
        if (!(request.Type & 0x80)) {
            LOG("HCDSubmitControlMessageIN called with OUT request type: %#x\n", request.Type);
            co_return RESULT::ErrorArgument;
        }

        // Just returning the task from HCDSubmitControlMessage is tempting, but...
        // In order to respect the lifetime of the channel, we need this to be a proper coroutine.
        co_return co_await HCDSubmitControlMessage(
            device,
            ioHandle,
            buffer,
            request,
            timeout,
            bytesTransferred
        );
    }


    /*-HCDSetAddress ------------------------------------------------------------
    Sets the address of the device with control endpoint given by the pipe. Zero
    is a restricted address for the rootHub and will return if attempted.
    24Feb17 LdB
    --------------------------------------------------------------------------*/
    Async::task<RESULT> HCDSetAddress (UsbDevice* device, IoHandle const& ioHandle, uint8_t address)
    {
        if (address == 0) co_return RESULT::ErrorArgument;							// You can't set address zero that is strictly reserved for roothub
        co_return co_await HCDSubmitControlMessage(
            device,
            ioHandle,
            {},													// No data
            UsbDeviceRequest {
                .Type = 0,
                .Request = SetAddress,									// Set address request
                .Value = address,										// Address to set
            },
            ControlMessageTimeout,
            nullptr);
    }

    /*-INTERNAL: HCDSetConfiguration---------------------------------------------
    Sets a given USB device configuration to the config index number requested.
    28Feb17 LdB
    --------------------------------------------------------------------------*/
    Async::task<RESULT> HCDSetConfiguration (UsbDevice* device, IoHandle const& ioHandle, uint8_t configuration)
    {
        return HCDSubmitControlMessage(
            device,
            ioHandle,
            {},                                                    // No data
            UsbDeviceRequest {
                .Type = 0,
                .Request = SetConfiguration,							// Set configuration
                .Value = configuration,									// Config index
            },
            ControlMessageTimeout,
            nullptr);													// Read the requested configuration
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
    Async::task<RESULT> HCDReadHubPortStatus (UsbDevice* device,
                                uint8_t port,							// Port to get status  OR  0 = Gateway node
                                uint32_t& Status)						// HubPortFullStatus or HubFullStatus .. use Raw union  
    {
        uint32_t transfer = 0;
        auto const result = co_await HCDSubmitControlMessageIN(
            device,
            { (std::byte*)&Status, sizeof(Status) },
            UsbDeviceRequest {								// Construct a USB request
                .Type = port ? bmREQ_PORT_STATUS : bmREQ_HUB_STATUS,	// Request bit mask is for hub if port = 0, hub port otherwise 
                .Request = GetStatus,									// Get status id
                .Index = port,											// Port number is index 1 so we add one
                .Length = sizeof(uint32_t),								// We want full structure size
            },
            ControlMessageTimeout,										// Standard control message timeouts
            &transfer
        );
        if (result != RESULT::Ok)
        {
            LOG("HCD Hub read status failed on device: %i, port: %i, Result: %#x, Pipe Speed: %#x, Pipe MaxPacket: %u\n",
                device->Pipe0.Number, port, result, device->Pipe0.Speed, device->Pipe0.MaxPacketSizeInBytes);	// Log any error
            co_return result;												// Return error result
        }
        if (transfer < sizeof(uint32_t)) {								// Hub did not read amount requested
            LOG("HUB: Failed to read hub device:%i port:%i status\n",
                device->Pipe0.Number, port);										// Log error
            co_return RESULT::ErrorDevice;											// Some quirk in enumeration usually
        }
        co_return RESULT::Ok;														// Return success
    }

    /*-INTERNAL: HCDChangeHubPortFeature-----------------------------------------
    Changes a feature setting on the given port on a hub device. Port input is
    index 1 and so requesting port 0 is interpretted as you are changing the
    feature on the port gateway node.
    21Mar17 LdB
    --------------------------------------------------------------------------*/
    Async::task<RESULT> HCDChangeHubPortFeature (UsbDevice* device,
                                    HubPortFeature feature,		// Which feature to change
                                    uint8_t port,						// Port to change feature  OR  0 = Gateway node
                                    bool set)							// Set or clear the feature
    {
        auto const result = co_await HCDSubmitControlMessageOUT(
            device,
            {},
            UsbDeviceRequest {
                .Type = port > 0 ? bmREQ_PORT_FEATURE : bmREQ_HUB_FEATURE,
                .Request = set ? SetFeature : ClearFeature,				// Set or clear feature as requested
                .Value = (uint16_t)feature,								// Feature we are changing
                .Index = port,
            },
            ControlMessageTimeout,										// Standard control message timeouts
            nullptr
        );
        if (result != RESULT::Ok)
        {
            LOG("HUB: Failed to change port feature for device: %i, Port:%d feature:%d set:%d.\n",
                device->Pipe0.Number, port, feature, set);						// Log any error
            co_return result;
        }
        co_return RESULT::Ok;
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
    Async::task<RESULT> HCDReadStringDescriptor (UsbDevice* device,
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

        if (buffer == nullptr || stringIndex == 0) co_return RESULT::ErrorArgument;
        result = co_await HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_STRING, 0, 0, &langIds, 2,
            bmREQ_GET_DEVICE_DESCRIPTOR, &transfer, true);				// Get language support header
        if ((result != RESULT::Ok) && (transfer < 2)) {							// Could not read language support data
            LOG("HCD: Could not read language support for device: %i\n",
                device->Pipe0.Number);											// Log the error
            co_return RESULT::ErrorArgument;										// I am lost what is going on bail
        }

        // langIds 0 actually has 0x03 (string descriptor) and size of language support words .. if it doesn't bail
        if ((langIds[0] >> 8) != 0x03) {								// The top byte has to be 0x03
            LOG("HCD: Not a valid language support descriptor on device: %i\n",
                device->Pipe0.Number);											// Log the error
            co_return RESULT::ErrorArgument;										// I am lost what is going on bail
        }
        // So we have size to read for all the language support pairs
        result = co_await HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_STRING, 0, 0, &langIds, langIds[0] & 0xFF,
            bmREQ_GET_DEVICE_DESCRIPTOR, &transfer, true);				// Get all language support pair data
        if ((result != RESULT::Ok) && (transfer < (langIds[0] & 0xFF))) {		// We failed to read all the support data
            LOG("HCD: Could not read all the language support data on device: %i\n",
                device->Pipe0.Number);											// Log the error		
            co_return RESULT::ErrorArgument;										// I am lost what is going on bail
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
        result = co_await HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_STRING, stringIndex,
            NoEnglishSupport ? langIds[1] : 0x409, &Header,
            sizeof(struct UsbDescriptorHeader), bmREQ_GET_DEVICE_DESCRIPTOR, 
            &transfer, true);											// Read string descriptor header only
        if ((result != RESULT::Ok) || (transfer != sizeof(struct UsbDescriptorHeader))) {
            LOG("HCD: Could not fetch string descriptor header (%i) for device: %i\n",
                stringIndex, device->Pipe0.Number);								// Log the error
            co_return RESULT::ErrorDevice;											// No idea what problem is so bail										
        }

        // Okay we got the size of the string so now read the entire size
        result = co_await HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_STRING, stringIndex,
            NoEnglishSupport ? langIds[1] : 0x409, &descBuffer,
            Header.DescriptorLength, bmREQ_GET_DEVICE_DESCRIPTOR, 
            &transfer, true);											// Read the full string 	
        if ((result != RESULT::Ok) || (transfer != Header.DescriptorLength)) {
            LOG("HCD: Could not fetch string descriptor (%i) for device: %i\n",
                stringIndex, device->Pipe0.Number);								// Log the error
            co_return RESULT::ErrorArgument;										// No idea what problem is so bail
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

        co_return RESULT::Ok;														// Return success
    }

    RESULT AddHidPayload(UsbDevice& device)
    {
        if (device.PayLoadId != PayLoadType::None)
        {
            return RESULT::ErrorArgument;
        }

        device.HidPayload = AllocateHidPayload();
        if (device.HidPayload == nullptr)
        {
            return RESULT::ErrorMemory;
        }

        if (auto shared = SharedHandleFor(device))
        {
            BindHidOwner(device.HidPayload, shared);
        }

        device.PayLoadId = PayLoadType::Hid;
        return RESULT::Ok;
    }

    void RemoveHidPayload(UsbDevice& device)
    {
        if (device.PayLoadId == PayLoadType::Hid && device.HidPayload != nullptr)
        {
            FreeHidPayload(device.HidPayload);
            device.HidPayload = nullptr;
            device.PayLoadId = PayLoadType::None;
        }
    }

    RESULT AddHubPayload(UsbDevice& device) {
        if (device.PayLoadId == PayLoadType::None) {
            auto hub = std::make_shared<HubDevice>();
            if (!hub) {
                return RESULT::ErrorMemory;
            }
            device.HubPayload = hub.get();
            device.PayLoadId = PayLoadType::Hub;
            HubTable.push_back(std::move(hub));
            return RESULT::Ok;
        }
        return RESULT::ErrorArgument;
    }

    void RemoveHubPayload(UsbDevice& device) {
        if (device.PayLoadId == PayLoadType::Hub && device.HubPayload) {
            for (auto* pChild : device.HubPayload->Children) {
                if (pChild) {
                    UsbDeallocateDevice(pChild);
                }
            }

            auto const it = std::remove_if(HubTable.begin(), HubTable.end(), [&device](std::shared_ptr<HubDevice> const& hub) {
                return hub.get() == device.HubPayload;
            });
            HubTable.erase(it, HubTable.end());

            device.HubPayload = nullptr;
            device.PayLoadId = PayLoadType::None;
        }
    }

    std::expected<std::shared_ptr<UsbDevice>, RESULT> UsbAllocateDevice()
    {
        std::shared_ptr<UsbDevice> device = std::make_shared<UsbDevice>(DriverRef());
        if (!device)
        {
            return std::unexpected(RESULT::ErrorMemory);
        }

        device->Config.Status = USB_STATUS_ATTACHED;
        device->ParentHub.PortNumber = 0;
        device->ParentHub.Number = 0xFF;
        device->PayLoadId = PayLoadType::None;
        device->HubPayload = nullptr;

        for (uint8_t number = 0; number < DeviceTable.size(); ++number) {	// Search device table entries
            if (!DeviceTable[number]) {
                device->Pipe0.Number = number + 1;
                DeviceTable[number] = device;
                return device;
            }
        }
        if (DeviceTable.size() >= UINT8_MAX)
        {
			return std::unexpected(RESULT::ErrorMemory);
        }
        device->Pipe0.Number = static_cast<uint8_t>(DeviceTable.size() + 1);
        DeviceTable.push_back(device);
        return device;
    }

    void UsbDeallocateDevice (struct UsbDevice *device) {
        if (device == nullptr) {
            return;
        }

        if (device->IsHub()) {								// If this device is a hub we will need to deal with the children
            for (auto* child : device->HubPayload->Children) {
                if (child != nullptr)
                    UsbDeallocateDevice(child);
            }
            RemoveHubPayload(*device);
        }

        if (auto* parent = UsbDeviceAtAddress(static_cast<uint8_t>(device->ParentHub.Number)); parent && parent->PayLoadId == PayLoadType::Hub && parent->HubPayload)
        {
            auto const port = static_cast<size_t>(device->ParentHub.PortNumber);
            if (port < parent->HubPayload->Children.size() && parent->HubPayload->Children[port] == device)
            {
                parent->HubPayload->Children[port] = nullptr;
            }
        }

        auto const deviceAddress = static_cast<size_t>(device->Pipe0.Number);
        if (deviceAddress > 0 && deviceAddress <= DeviceTable.size())
        {
            DeviceTable[deviceAddress - 1].reset();
        }
    }

    /*==========================================================================}
    {			    NON HCD INTERNAL HUB FUNCTIONS ON PORTS						}
    {==========================================================================*/
    Async::task<RESULT> HubPortReset(UsbDevice& device, uint8_t port) {
        RESULT result;
        struct HubPortFullStatus portStatus;
        uint32_t retry, timeout;
        if (!device.IsHub()) co_return RESULT::ErrorDevice;
        LOG_DEBUG("HUB: Reseting device: %u Port: %u. source: %i\n", device.Pipe0.Number, port, 0/*source*/);
        for (retry = 0; retry < 3; retry++) {
            if ((result = co_await HCDChangeHubPortFeature(&device,
                FeatureReset, port + 1, true)) != RESULT::Ok) 					// Issue a setfeature of reset
            {
                LOG("HUB: Device %i Failed to reset Port%d.\n",
					device.Pipe0.Number, port + 1);
                co_return result;											// Return result that is causing failure
            }
            timeout = 0;
            do {
                co_await Async::DelayInMicroseconds(20000);
                if ((result = co_await HCDReadHubPortStatus(&device, port + 1, portStatus.Raw32)) != RESULT::Ok) {
                    LOG("HUB: Hub failed to get status (4) for %s.Port%d.\n", UsbGetDescription(&device), port + 1);
                    co_return result;
                }
                timeout++;
            } while (!portStatus.Change.ResetChanged && !portStatus.Status.Enabled && timeout < 10);

            if (timeout == 10) continue;

            LOG_DEBUG("HUB: %s.Port%d Status %x:%x.\n", UsbGetDescription(&device), port + 1, portStatus.RawStatus, portStatus.RawChange);

            if (portStatus.Change.ConnectedChanged || !portStatus.Status.Connected)
                co_return RESULT::ErrorDevice;

            if (portStatus.Status.Enabled)
                break;
        }

        if (retry == 3) {
            LOG("HUB: Cannot enable %s.Port%d. Please verify the hardware is working.\n", UsbGetDescription(&device), port + 1);
            co_return RESULT::ErrorDevice;
        }

        if ((result = co_await HCDChangeHubPortFeature(&device, FeatureResetChange, port + 1, false)) != RESULT::Ok) {
            LOG("HUB: Failed to clear reset on %s.Port%d.\n", UsbGetDescription(&device), port + 1);
        }
        co_return RESULT::Ok;
    }

    /*-INTERNAL: HubPortConnectionChanged ---------------------------------------
    If a connection on a port on a hub as changed this routine is called to deal
    with the change. This will involve it enumerating an added new device or the
    deallocation of a removed or detached device.
    21Mar17 LdB
    --------------------------------------------------------------------------*/
    __attribute__((noinline)) Async::task<RESULT> HubPortConnectionChanged(UsbDevice& device, uint8_t port) {
        RESULT result;
        struct HubDevice *data;
        struct HubPortFullStatus portStatus;
        if (!device.IsHub()) co_return RESULT::ErrorDevice;

        data = device.HubPayload;

        if ((result = co_await HCDReadHubPortStatus(&device, port + 1, portStatus.Raw32)) != RESULT::Ok) {
            LOG("HUB: Hub failed to get status (2) for %s.Port%d.\n", UsbGetDescription(&device), port + 1);
            co_return result;
        }
        LOG_DEBUG("HUB: %s.Port%d Status %x:%x.\n", UsbGetDescription(&device), port + 1, portStatus.RawStatus, portStatus.RawChange);

        if ((result = co_await HCDChangeHubPortFeature(&device, FeatureConnectionChange, port + 1, false)) != RESULT::Ok) {
            LOG("HUB: Failed to clear change on %s.Port%d.\n", UsbGetDescription(&device), port + 1);
        }

        if ((!portStatus.Status.Connected && !portStatus.Status.Enabled) || data->Children[port] != nullptr) {
            LOG("HUB: Disconnected %s.Port%d - %s.\n", UsbGetDescription(&device), port + 1, UsbGetDescription(data->Children[port]));
            UsbDeallocateDevice(data->Children[port]);
            data->Children[port] = nullptr;
            if (!portStatus.Status.Connected) co_return RESULT::Ok;
        }

        if ((result = co_await HubPortReset(device, port)) != RESULT::Ok) {
            LOG("HUB: Could not reset %s.Port%d for new device.\n", UsbGetDescription(&device), port + 1);
            co_return result;
        }

        auto childEx = UsbAllocateDevice();
        if (!childEx.has_value()) {
            LOG("HUB: Could not allocate a new device entry for %s.Port%d.\n", UsbGetDescription(&device), port + 1);
            co_return childEx.error();
        }

        data->Children[port] = childEx.value().get();

        if ((result = co_await HCDReadHubPortStatus(&device, port + 1, portStatus.Raw32)) != RESULT::Ok) {
            LOG("HUB: Hub failed to get status (3) for %s.Port%d.\n", UsbGetDescription(&device), port + 1);
            co_return result;
        }

        LOG("HUB: %s. Device:%i Port:%d Status %04x:%04x.\n", UsbGetDescription(&device), device.Pipe0.Number, port, portStatus.RawStatus, portStatus.RawChange);

        if (portStatus.Status.HighSpeedAttatched)
        {
            data->Children[port]->Pipe0.Speed = USB_SPEED_HIGH;
        }
        else if (portStatus.Status.LowSpeedAttatched)
        {
            data->Children[port]->Pipe0.Speed = USB_SPEED_LOW;
            data->Children[port]->Pipe0.splitNodePoint = device.Pipe0.Number;
            data->Children[port]->Pipe0.splitNodePort = port;
        }
        else
        {
            data->Children[port]->Pipe0.Speed = USB_SPEED_FULL;
            data->Children[port]->Pipe0.splitNodePoint = device.Pipe0.Number;
            data->Children[port]->Pipe0.splitNodePort = port;
        }
        data->Children[port]->ParentHub.Number = device.Pipe0.Number;
        data->Children[port]->ParentHub.PortNumber = port;
        if ((result = co_await EnumerateDevice(data->Children[port], &device, port)) != RESULT::Ok) {
            LOG("HUB: Could not connect to new device in %s.Port%d. Disabling.\n", UsbGetDescription(&device), port + 1);
            UsbDeallocateDevice(data->Children[port]);
            data->Children[port] = nullptr;
            if (co_await HCDChangeHubPortFeature(&device, FeatureEnable, port + 1, false) != RESULT::Ok) {
                LOG("HUB: Failed to disable %s.Port%d.\n", UsbGetDescription(&device), port + 1);
            }
            co_return result;
        }
        co_return RESULT::Ok;
    }


    /*-HubCheckConnection -------------------------------------------------------
    Checks device is a hub and if a valid hub checks connection status of given
    port on the hub. If it has changed performs necessary actions such as the
    enumerating of a new device or deallocating an old one.
    10Apr17 LdB
    --------------------------------------------------------------------------*/
    Async::task<RESULT> HubCheckConnection(UsbDevice& device, uint8_t port)
    {
        RESULT result;
        HubPortFullStatus portStatus;
        HubDevice *data;

        if (!device.IsHub()) co_return RESULT::ErrorDevice;
        data = device.HubPayload;

        LOG("HUB: Checking connection for device %i, Port: %i.\n", device.Pipe0.Number, port);

        if ((result = co_await HCDReadHubPortStatus(&device, port + 1, portStatus.Raw32)) != RESULT::Ok) {
            if (result != RESULT::ErrorDisconnected)
                LOG("HUB: Failed to get hub port status (1) for %s.Port%d.\n", UsbGetDescription(&device), port + 1);
            co_return result;
        }

        LOG("HUB: device %i, Port: %i, status: %04X:%04X.\n", device.Pipe0.Number, port, portStatus.RawStatus, portStatus.RawChange);

        if (portStatus.Change.ConnectedChanged) {
            LOG_DEBUG("Device %i, Port: %i changed\n", device.Pipe0.Number, port);
            co_await HubPortConnectionChanged(device, port);
        }

        LOG_DEBUG("Device %i, Port: %i checking the rest\n", device.Pipe0.Number, port);

        if (portStatus.Change.EnabledChanged) {
            if (co_await HCDChangeHubPortFeature(&device, FeatureEnableChange, port + 1, false) != RESULT::Ok) {
                LOG("HUB: Failed to clear enable change %s.Port%d.\n", UsbGetDescription(&device), port + 1);
            }

            // This may indicate EM interference.
            if (!portStatus.Status.Enabled && portStatus.Status.Connected && data->Children[port] != nullptr) {
                LOG("HUB: %s.Port%d has been disabled, but is connected. This can be cause by interference. Reenabling!\n", UsbGetDescription(&device), port + 1);
                co_await HubPortConnectionChanged(device, port);
            }
        }

        if (portStatus.Status.Suspended) {
            if (co_await HCDChangeHubPortFeature(&device, FeatureSuspend, port + 1, false) != RESULT::Ok) {
                LOG("HUB: Failed to clear suspended port - %s.Port%d.\n", UsbGetDescription(&device), port + 1);
            }
        }

        if (portStatus.Change.OverCurrentChanged) {
            if (co_await HCDChangeHubPortFeature(&device, FeatureOverCurrentChange, port + 1, false) != RESULT::Ok) {
                LOG("HUB: Failed to clear over current port - %s.Port%d.\n", UsbGetDescription(&device), port + 1);
            }
        }

        if (portStatus.Change.ResetChanged) {
            if (co_await HCDChangeHubPortFeature(&device, FeatureResetChange, port + 1, false) != RESULT::Ok) {
                LOG("HUB: Failed to clear reset port - %s.Port%d.\n", UsbGetDescription(&device), port + 1);
            }
        }

        co_return RESULT::Ok;
    }

    /*-INTERNAL: HubCheckForChange ----------------------------------------------
    This performs an iteration loop to check each port on each hub to see if any
    device has been added or removed.
    21Mar17 LdB
    --------------------------------------------------------------------------*/
    Async::task<void> HubCheckForChange(UsbDevice& device) {
        if (device.IsHub()) {
            for (size_t i = 0; i < device.HubPayload->Children.size(); i++) {
                if (co_await HubCheckConnection(device, static_cast<uint8_t>(i)) != RESULT::Ok) continue;
                if (device.HubPayload->Children[i] != nullptr)
                    co_await HubCheckForChange(*device.HubPayload->Children[i]);
            }
        }
    }

    /*==========================================================================}
    {						 INTERNAL ENUMERATION ROUTINES						}
    {==========================================================================*/

    /*-INTERNAL: EnumerateHub ---------------------------------------------------
    Continues enumeration of each port if an enumerated detected device is a hub
    11Feb17 LdB
    --------------------------------------------------------------------------*/
    Async::task<RESULT> EnumerateHub(UsbDevice& device) {
        RESULT result;
        uint32_t transfer;
        HubDevice *data;
        HubFullStatus status;

        if (auto const thisResult = AddHubPayload(device); thisResult != RESULT::Ok) {
            LOG("Could not allocate hub payload, Error ID %i\n", thisResult);
			co_return thisResult;
        }

        data = device.HubPayload;

        result = co_await HCDGetDescriptor(&device, USB_DESCRIPTOR_TYPE_HUB,
            0, 0, &data->Descriptor, sizeof(HubDescriptor),
            bmREQ_GET_HUB_DESCRIPTOR, &transfer, true);
        if ((result != RESULT::Ok) || (transfer != sizeof(HubDescriptor)))
        {
            LOG("HCD: Could not fetch hub descriptor for device: %i\n",
                device.Pipe0.Number);
			co_return RESULT::ErrorDevice;
        }
        LOG_DEBUG("Hub device %i has %i ports\n", device.Pipe0.Number, data->Descriptor.PortCount);
        LOG_DEBUG("HUB: Hub power to good: %dms.\n", data->Descriptor.PowerGoodDelay * 2);
        LOG_DEBUG("HUB: Hub current required: %dmA.\n", data->Descriptor.MaximumHubPower * 2);

        data->Children.assign(data->Descriptor.PortCount, nullptr);

        if (auto const thisResult = co_await HCDReadHubPortStatus(&device, 0, status.Raw32); thisResult != RESULT::Ok)
        {
            LOG("HUB device:%i failed to get hub status.\n", device.Pipe0.Number);
            co_return thisResult;
        }

        LOG("HUB: Hub powering ports on.\n");
        for (size_t i = 0; i < data->Children.size(); i++) {
            if (co_await HCDChangeHubPortFeature(&device, FeaturePower, static_cast<uint8_t>(i + 1), true) != RESULT::Ok)
                LOG("HUB: device: %i could not power Port%d.\n", device.Pipe0.Number, i + 1);
        }
        co_await Async::DelayInMicroseconds(data->Descriptor.PowerGoodDelay * 2000);
        /*co_await Async*/ Cpu::DelayInMicroseconds(1'000);

        LOG("HUB: device: %i checking %u port connections.\n", device.Pipe0.Number, data->Children.size());

        for (size_t port = 0; port < data->Children.size(); port++) {
            co_await HubCheckConnection(device, static_cast<uint8_t>(port));
        }

        co_return RESULT::Ok;														// Return success
    }


    /*-INTERNAL: EnumerateDevice ------------------------------------------------
    All detected devices start enumeration here. We recover critical information
    of every USB device and hold those details in the device data block. Finally 
    if the device is recognized as any of the sepcial specific class then it will
    call extended enumeration for those specific classes.
    11Feb17 LdB
    --------------------------------------------------------------------------*/
    Async::task<RESULT> EnumerateDevice(struct UsbDevice *device, struct UsbDevice* ParentHub, uint8_t PortNum) {
        RESULT result;
        DWCRESULT dwcResult;
        uint8_t address;
        uint32_t transferred;
        DeviceDescriptor desc = { 0 };
        char buffer[256] __attribute__((aligned(4)));					// Text buffer

        auto const ioHandle = GetIoHandle(device->GetAddress());

        /* Store the unique address until it is actually assigned. */
        address = device->Pipe0.Number;									// Hold unique address we will set device to
        device->Pipe0.Number = 0;										// Initially it starts as zero
        LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 1 = Read first 8 Bytes of Device Descriptor\n");
        device->Pipe0.MaxPacketSizeInBytes = 8;							// Set max packet size to 8 ( So exchange will be exactly 1 packet)

        result = co_await HCDSubmitControlMessageIN(
            device,
            ioHandle,
            { reinterpret_cast<std::byte*>(&desc), 8 }, // Ask for first 8 bytes as per USB specification
            UsbDeviceRequest {								// We will build a request structure
                .Type = bmREQ_GET_DEVICE_DESCRIPTOR,					// Recipient is a flag usually 0x0 for normal device, 0x20 for a hub
                .Request = GetDescriptor,								// We want a descriptor obviously
                .Value = (uint16_t)USB_DESCRIPTOR_TYPE_DEVICE << 8,		// Type and the index (0) get compacted as the value
                .Index = 0,												// We want descriptor 0
                .Length = 8,											// 8 bytes as per USB enumeration by the book
            },
            ControlMessageTimeout,										// The standard timeout for any control message
            &transferred);												// Pass in pointer to get bytes transferred back
        if ((result != RESULT::Ok) || (transferred != 8)) {						// This should pass on any valid device
            LOG("Enumeration: Step 1 on device %i failed, Result: %#x, transferred: %u.\n",
                address, result, transferred);										// Log any error
            co_return result != RESULT::Ok ? result : RESULT::ErrorTransmission;	// Fatal enumeration error of this device
        }
        LOG_DEBUG("Max packet size: %u\n", desc.bMaxPacketSize0);
        device->Pipe0.MaxPacketSizeInBytes = desc.bMaxPacketSize0;	    // Set the maximum endpoint packet size to pipe from response
        device->Config.Status = USB_STATUS_DEFAULT;						// Move device enumeration to default

        LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 2 = Reset Port (old device support)\n");
        if (ParentHub != nullptr) {
            // Reset the port for what will be the second time.
            if ((result = co_await HubPortReset(*ParentHub, PortNum)) != RESULT::Ok) {
                LOG("HCD: Failed to reset port again for new device %s.\n", UsbGetDescription(device));
                device->Pipe0.Number = address;
                co_return result;
            }
        }
        
        LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 3 = Set Device Address %u\n", address);
        if ((result = co_await HCDSetAddress(device, ioHandle, address)) != RESULT::Ok) {
            LOG("Enumeration: Failed to assign address to %#x.\n", address);// Log the error
            device->Pipe0.Number = address;								// Set device number just so it stays valid
            co_return result;												// Fatal enumeration error of this device
        }
        device->Pipe0.Number = address;									// Device successfully addressed so put it back to control pipe								
        co_await Async::DelayInMicroseconds(10000);												// Allows time for address to propagate.
        device->Config.Status = USB_STATUS_ADDRESSED;					// Our enumeration status in now addressed

        LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 4 = Read Device Descriptor At Address\n");
        result = co_await HCDGetDescriptor(
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
            LOG("Enumeration: Step 4 on device %i failed, Got %u bytes != %u.\n",
                device->Pipe0.Number, transferred, (uint32_t)sizeof(device->Descriptor));
            co_return RESULT::ErrorTransmission;
        }
        if (result != RESULT::Ok)
        {
            LOG("Enumeration: Step 4 on device %i failed, Result: %#x.\n",
                device->Pipe0.Number, result);						// Log any error
            co_return result;
        }
        LOG_DEBUG("Device: %u, Class: %u, Subclass: %u\n", device->Pipe0.Number, device->Descriptor.bDeviceClass, device->Descriptor.bDeviceSubClass);


        LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 5 = Read Device Configurations\n");
        // Read the master Config at index 0 ... this is not really a config but an index to avail configs
        uint32_t transfer;
        ConfigurationDescriptor configDesc;
        result = co_await HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_CONFIGURATION, 0, 0,
            &configDesc, sizeof(configDesc), bmREQ_GET_DEVICE_DESCRIPTOR,
            &transfer, true);											// Read the config descriptor 	
        if (result == RESULT::Ok && transfer != sizeof(configDesc))
        {
            LOG("HCD: Got %u bytes != %u reading configuration descriptor for device: %i\n",
                transfer, (uint32_t)sizeof(device->Descriptor), device->Pipe0.Number);
            co_return RESULT::ErrorTransmission;
        }
        if (result != RESULT::Ok) {
            LOG("HCD: Error: %i, reading configuration descriptor for device: %i\n",
                result, device->Pipe0.Number);
            co_return RESULT::ErrorDevice;											// No idea what problem is so bail
        }
        device->Config.ConfigStringIndex = configDesc.iConfiguration;	// Grab string index while here

        // Most devices I played with only have 1 config .. regardless we will take first
        // The index to call is given as at offset 5 bConfigurationValue
        // Read it by that index it's probably the same but just do it
        uint8_t configNum = configDesc.bConfigurationValue;
        // Okay we have the total length of config so we will read it in entirity
        std::byte configBuffer[1024];										// Largest config I have ever seen is few hundred bytes this is 1K buffer
        result = co_await HCDSubmitControlMessageIN(
            device,												// Device 
            ioHandle,
            { configBuffer, configDesc.wTotalLength },
            UsbDeviceRequest {								// We will build a request structure
                .Type = bmREQ_GET_DEVICE_DESCRIPTOR,					// We want normal device descriptor
                .Request = GetDescriptor,								// We want a descriptor obviously
                .Value = (uint16_t)USB_DESCRIPTOR_TYPE_CONFIGURATION << 8,// Type and the index get compacted as the value
                .Index = 0,												// Language ID is the index
                .Length = configDesc.wTotalLength,						// Duplicate the length
            },
            ControlMessageTimeout,										// The standard timeout for any control message
            &transfer);													// Set pointer to fetch transfer bytes
        if ((result != RESULT::Ok) || (transfer != configDesc.wTotalLength)) {	// Check if anything went wrong
            LOG("HCD: Failed to read configuration descriptor for device %i, %u bytes read, Error: %i.\n",
                device->Pipe0.Number, (unsigned int)transfer, result);				// Log error
            if (result != RESULT::Ok) co_return result;							// Return error result
            co_return RESULT::ErrorDevice;											// Something went badly wrong .. bail
        }

        device->Interfaces.clear();
        device->Endpoints.clear();

        uint8_t hidCount = 0;
        std::optional<uint8_t> currentInterfaceIndex;
        uint32_t i = 0;
        while (i < configDesc.wTotalLength - 1) {
            switch (static_cast<usb_descriptor_type>(configBuffer[i + 1])) {
            case USB_DESCRIPTOR_TYPE_INTERFACE: {
                UsbInterfaceDescriptor interfaceDescriptor{};
                memcpy(&interfaceDescriptor, &configBuffer[i], sizeof(interfaceDescriptor));
                device->Interfaces.push_back(interfaceDescriptor);
                device->Endpoints.emplace_back();
                currentInterfaceIndex = static_cast<uint8_t>(device->Interfaces.size() - 1);
                break;
            }
            case USB_DESCRIPTOR_TYPE_ENDPOINT: {
                if (!currentInterfaceIndex.has_value()) {
                    break;
                }
                UsbEndpointDescriptor endpointDescriptor{};
                memcpy(&endpointDescriptor, &configBuffer[i], sizeof(endpointDescriptor));
                device->Endpoints[*currentInterfaceIndex].push_back(endpointDescriptor);
                break;
            }
            case USB_DESCRIPTOR_TYPE_HID: {
                if (!currentInterfaceIndex.has_value()) {
                    break;
                }
                if (hidCount == 0) {
                    if ((result = AddHidPayload(*device)) != RESULT::Ok) {
                        LOG("Could not allocate hid payload, Error ID %i\n", result);
                        co_return result;
                    }
                }
                if (SetHidDescriptor(device->HidPayload, hidCount, *currentInterfaceIndex, &configBuffer[i], static_cast<uint8_t>(configBuffer[i]))) {
                    hidCount++;
                }
                break;
            }
            default:
                break;
            }
            i = i + static_cast<uint8_t>(configBuffer[i]);
        }

        LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 6 = Set Configuration to Device\n");
        if (auto const thisResult = co_await HCDSetConfiguration(device, ioHandle, configNum); thisResult != RESULT::Ok) {
            LOG("HCD: Failed to set configuration %#x for device %i.\n",
                configNum, device->Pipe0.Number);
            co_return thisResult;
        }
        device->Config.ConfigIndex = configNum;							// Hold the configuration index
        device->Config.Status = USB_STATUS_CONFIGURED;					// Set device status to configured

        LOG("HCD: Attach Device %s. Address:%d Class:%d USB:%x.%x, %d configuration(s), %d interface(s).\n",
            UsbGetDescription(device), address, device->Descriptor.bDeviceClass, (device->Descriptor.bcdUSB >> 8) & 0xFF,
            device->Descriptor.bcdUSB & 0xFF, device->Descriptor.bNumConfigurations, device->Interfaces.size());
        
        if (device->Descriptor.iProduct != 0) {
            size_t length = sizeof(buffer);
            if (co_await HCDReadStringDescriptor(device, device->Descriptor.iProduct, &buffer[0], length) == RESULT::Ok)
            {
                LOG("HCD:  -Product:       %s.\n", buffer);
            }
        }
        
        if (device->Descriptor.iManufacturer != 0) {
            size_t length = sizeof(buffer);
            if (co_await HCDReadStringDescriptor(device, device->Descriptor.iManufacturer, &buffer[0], length) == RESULT::Ok)
            {
                LOG("HCD:  -Manufacturer:  %s.\n", buffer);
            }
        }
        if (device->Descriptor.iSerialNumber != 0) {
            size_t length = sizeof(buffer);
            if (co_await HCDReadStringDescriptor(device, device->Descriptor.iSerialNumber, &buffer[0], length) == RESULT::Ok)
            {
                LOG("HCD:  -SerialNumber:  %s.\n", buffer);
            }
        }


        if (device->Config.ConfigStringIndex != 0) {
            size_t length = sizeof(buffer);
            if (co_await HCDReadStringDescriptor(device, device->Config.ConfigStringIndex, &buffer[0], length) == RESULT::Ok)
            {
                LOG("HCD:  -Configuration: %s.\n", buffer);
            }
        }


        LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 7 = ENUMERATE SPECIAL DEVICES\n");
        if (device->Descriptor.bDeviceClass == DeviceClassHub) {		// If device is a hub then enumerate it
            LOG_DEBUG("Device is a hub, enumerating ports.\n");
            if ((result = co_await EnumerateHub(*device)) != RESULT::Ok) {
                LOG("Could not enumerate HUB device %i, Error ID %i\n",
                    device->Pipe0.Number, result);						// Log error
                co_return result;											// Return the error
            }
        } else if (hidCount > 0) {										// HID interface on the device
            LOG_DEBUG("Device hidCount: %u, enumerating ports.\n", hidCount);
            if ((result = co_await EnumerateHID(*device)) != RESULT::Ok) {	// RESULT::Ok so enumerate the HID device
                LOG("Could not enumerate HID device %i, Error ID %i\n",
                    device->Pipe0.Number, result);
                co_return result;											// return the error
            }
        }
        else {														// If not a hub or HID then just log the device
            LOG_DEBUG("Device is not a hub or HID, skipping enumeration.\n");
        }

        co_return RESULT::Ok;
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
    Async::task<RESULT> HCDGetDescriptor (UsbDevice* device,
                            enum usb_descriptor_type type,				// The type of descriptor
                            uint8_t index,								// The index of the type descriptor
                            uint16_t langId,							// The language id
                            void* buffer,								// Buffer to recieve descriptor
                            uint32_t length,							// Maximumlength of descriptor
                            uint8_t recipient,							// Recipient flags									 
                            uint32_t *bytesTransferred,     			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)								
                            bool runHeaderCheck) override						// Whether to run header check
    {
        auto const ioHandle = GetIoHandle(device->GetAddress());

        RESULT result;
        uint32_t transfer;
        alignas(4) struct UsbDescriptorHeader header  = { 0 };
        if (runHeaderCheck) {
            result = co_await HCDSubmitControlMessageIN(
                device,													// Pipe passed in as is
                ioHandle,
                { reinterpret_cast<std::byte*>(&header), sizeof(header) },
                UsbDeviceRequest {							// We will build a request structure
                    .Type = recipient,									// Recipient is a flag usually bmREQ_GET_DEVICE_DESCRIPTOR, bmREQ_GET_HUB_DESCRIPTOR etc
                    .Request = GetDescriptor,							// We want a descriptor obviously
                    .Value = (uint16_t)(type << 8 | index),				// Type and the index get compacted as the value
                    .Index = langId,									// Language ID is the index
                    .Length = sizeof(header),							// Duplicate the length
                },
                ControlMessageTimeout,									// The standard timeout for any control message
                nullptr);													// Ignore bytes transferred
            if ((result == RESULT::Ok) && (header.DescriptorType != type))
            {
                LOG("HCD: Descriptor type mismatch, expected %#x got %#x for device:%i.\n",
                    type, header.DescriptorType, device->Pipe0.Number);	// Log error
                result = RESULT::ErrorGeneral;									// For some strange reason descriptor type is not right
            }
            if (result != RESULT::Ok) {											// RESULT in error
                LOG("HCD: Fail to get descriptor header %#x:%#x recepient: %#x, device:%i. RESULT %#x.\n",
                    type, index, recipient, device->Pipe0.Number, result);		// Log any error
                co_return result;
            }
            if (length > header.DescriptorLength)						// Check descriptor length vs buffer space
                length = header.DescriptorLength;						// The descriptor is shorter than buffer space provided
        }
        result = co_await HCDSubmitControlMessageIN(
            device,														// Pipe passed in as is
            ioHandle,
            { reinterpret_cast<std::byte*>(buffer), length },
            UsbDeviceRequest {								// We will build a request structure
                .Type = recipient,										//  Recipient is a flag usually bmREQ_GET_DEVICE_DESCRIPTOR, bmREQ_GET_HUB_DESCRIPTOR etc
                .Request = GetDescriptor,								// We want a descriptor obviously
                .Value = (uint16_t)(type << 8 | index),					// Type and the index get compacted as the value
                .Index = langId,										// Language ID is the index
                .Length = (uint16_t)length,										// Duplicate the length
            },
            ControlMessageTimeout,										// The standard timeout for any control message
            &transfer);													// Set pointer to fetch transfer bytes
        if (length != transfer) result = RESULT::ErrorTransmission; 			// The requested length does not match read length
        if (result != RESULT::Ok) {
            LOG("HCD: Failed to get descriptor %#x:%#x recepient: %#x, device:%i. RESULT %#x.\n",
                type, index, recipient, device->Pipe0.Number, result);
        }
        if (bytesTransferred) *bytesTransferred = transfer;
        co_return result;
    }

    /*--------------------------------------------------------------------------}
    {					 PUBLIC GENERIC USB INTERFACE ROUTINES					}
    {--------------------------------------------------------------------------*/

    RESULT error_ = RESULT::ErrorGeneral;

    RESULT GetError() override { return error_; }

    /// Initializes the USB driver by performing necessary interfactions with the
    /// host controller driver, and enumerating the initial device tree.
    Async::task<> Initialize()
    {
        auto hostEx = co_await HCDInitialize();
        if (!hostEx.has_value())
        {
            LOG("FATAL ERROR: HCD failed to Initialize.\n");
            error_ = ResultFromDwcResult(hostEx.error());
            co_return;
        }

        Host = std::move(hostEx).value();

        // Attach the root hub .. which will launch enumeration
        LOG_DEBUG("Allocating RootHub\n");

        auto rootHubDeviceEx = UsbAllocateDevice();
        if (!rootHubDeviceEx.has_value())
        {
            error_ = rootHubDeviceEx.error();
            co_return;
        }

        UsbDevice* rootHubDevice = rootHubDeviceEx.value().get();
        rootHubDevice->Pipe0.Number = 1;
        rootHubDevice->Pipe0.Speed = USB_SPEED_HIGH;
        rootHubDevice->Config.Status = USB_STATUS_ATTACHED;
        rootHubDevice->PayLoadId = PayLoadType::None;
        rootHubDevice->HubPayload = nullptr;
        rootHubDevice->ParentHub.Number = 0xFF;
        rootHubDevice->ParentHub.PortNumber = 0;

        auto const result = co_await EnumerateDevice(rootHubDevice, nullptr, 0);
        if (result != RESULT::Ok)
        {
            LOG("FATAL ERROR: Could not enumerate root HUB\n");
            error_ = result;
            co_return;
        }

        error_ = RESULT::Ok;
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
    struct UsbDevice * UsbGetRootHub() override
    { 
		if (!DeviceTable.empty() && DeviceTable[0])
			return DeviceTable[0].get();
                return nullptr;
    }

    /*-UsbDeviceAtAddress -------------------------------------------------------
    Given the unique USB address this will return the pointer to the USB device
    structure. If the address is not actually in use it will return NULL.
    11Apr17 LdB
    --------------------------------------------------------------------------*/
    UsbDevice* UsbDeviceAtAddress (uint8_t devNumber) override
    {
		if  (devNumber == 0 || devNumber > DeviceTable.size())
        {
            return nullptr;
        }
		auto const& device = DeviceTable[devNumber - 1];
		if (!device || device->PayLoadId == PayLoadType::Error)
        {
            return nullptr;
        }
        return device.get();
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
    Async::task<void> UsbCheckForChange() override
    {
        if (!DeviceTable.empty() && DeviceTable[0])
        {
            return HubCheckForChange(*DeviceTable[0]);
        }
        else
        {
            return {};
        }
    }


    /*--------------------------------------------------------------------------}
    {					 PUBLIC DISPLAY USB INTERFACE ROUTINES					}
    {--------------------------------------------------------------------------*/

    /*-UsbGetDescription --------------------------------------------------------
    Returns a description for a device. This is not read from the device, this
    is just generated given by the driver.
    Unchanged from Alex Chadwick
    --------------------------------------------------------------------------*/
    const char* UsbGetDescription (struct UsbDevice *device) override
    {
        if (device->Config.Status == USB_STATUS_ATTACHED)
            return "New Device (Not Ready)\0";
        else if (device->Config.Status == USB_STATUS_POWERED)
            return "Unknown Device (Not Ready)\0";
        else if (!DeviceTable.empty() && DeviceTable[0] && device == DeviceTable[0].get())
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

    // Sends/recieves data from/to the given buffer to/from the given endpoint.
    Async::task<RESULT> HCDEndpointTransfer(UsbDevice* device, UsbEndpointDescriptor endpoint, std::byte* buffer, uint32_t& bufferLength) override
    {
        LOG_DEBUG("HCD: %s transfer called for device %i, endpoint %i for %u bytes.\n",
            endpoint.EndpointAddress.Direction == USB_DIRECTION_IN ? "IN" : "OUT",
            device->Pipe0.Number, endpoint.EndpointAddress.Number, bufferLength);

        auto const channel = Host->GetChannel();

        // Set up the pipe for interrupt transfer
        UsbPipe pipe = {
            .MaxPacketSizeInBytes = endpoint.Packet.MaxSize,         // Endpoint max packet size
            .Speed               = device->Pipe0.Speed,             // Same speed as device
            .EndPoint            = endpoint.EndpointAddress.Number, // Endpoint address
            .Number              = device->Pipe0.Number,            // Same device address
            .splitNodePort    = device->Pipe0.splitNodePort,  // Copy low speed info
            .splitNodePoint   = device->Pipe0.splitNodePoint, // Copy low speed info
        };

        // Determine the correct data toggle (PID) for this endpoint
        // For interrupt endpoints, we need to alternate between DATA0 and DATA1
        PacketId packetId = USB_PID_DATA0;
        
        if (endpoint.Attributes.Type == USB_TRANSFER_TYPE_INTERRUPT) {
            // Find the interface this endpoint belongs to and get the data toggle state
            size_t interfaceIndex = 0;
            size_t endpointIndex = 0;
            bool foundEndpoint = false;
            
            for (size_t i = 0; i < device->Endpoints.size() && !foundEndpoint; i++) {
                for (size_t j = 0; j < device->Endpoints[i].size(); j++) {
                    auto const& ep = device->Endpoints[i][j];
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
        auto const transferred = endpoint.EndpointAddress.Direction == USB_DIRECTION_IN
            ? co_await channel->TransferIn (pipe, endpoint.Attributes.Type, { buffer, bufferLength }, packetId)
            : co_await channel->TransferOut(pipe, endpoint.Attributes.Type, { buffer, bufferLength }, packetId);

        // Update data toggle on successful transfer for interrupt endpoints
        if (transferred > 0 && endpoint.Attributes.Type == USB_TRANSFER_TYPE_INTERRUPT)
        {
            for (size_t i = 0; i < device->Endpoints.size(); i++) {
                for (size_t j = 0; j < device->Endpoints[i].size(); j++) {
                    auto& ep = device->Endpoints[i][j];

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

        co_return transferred > 0 ? RESULT::Ok : RESULT::ErrorTransmission;
    }

    friend Async::task<std::shared_ptr<UsbDriver>> UsbInitializeDesignWare();
};

Async::task<std::shared_ptr<UsbDriver>> UsbInitializeDesignWare()
{
    printf("Initializing DesignWare USB Driver\n");
    auto result = std::make_shared<DesignWareUsbDriver>();
    co_await result->Initialize();
    co_return result;
};
