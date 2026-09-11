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
#include <print>

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

    std::shared_ptr<UsbDriver> DriverRef()
    {
        return std::shared_ptr<UsbDriver>(this, [](UsbDriver*) {});
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
            device->GetAddress(), request.Request, request.Type, device->Pipe0.Speed, device->Pipe0.MaxPacketSizeInBytes, device->Pipe0.splitNodePoint, device->Pipe0.splitNodePort);

        uint32_t lastTransfer = 0;

        auto& channel = *static_cast<HCDChannel*>(ioHandle.get());

        UsbDirection Direction = (request.Type & 0x80) ? USB_DIRECTION_IN : USB_DIRECTION_OUT;

        LOG_DEBUG("Setup phase\n");
        // Setup phase
        uint32_t transferLength = co_await channel.TransferOut(device->Pipe0, USB_TRANSFER_TYPE_CONTROL, { (std::byte const*)&request, sizeof(request) }, USB_PID_SETUP);
        if (transferLength != sizeof(request))
        {
            LOG("HCD: SETUP packet to device: %#x req: %#x req Type: %#x Speed: %i PacketSize: %i LowNode: %i LowPort: %i Error: %zu != %zu\n",
                device->GetAddress(), request.Request, request.Type, device->Pipe0.Speed, device->Pipe0.MaxPacketSizeInBytes, device->Pipe0.splitNodePoint, device->Pipe0.splitNodePort, transferLength, sizeof(request));
            co_return ResultFromDwcResult(result);
        }

        if (buffer.empty())
        {
            if (Direction == USB_DIRECTION_IN) {
                LOG("HCD: No buffer provided for IN transfer to device %i.\n", device->GetAddress());
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
                    device->GetAddress(), buffer.size(), lastTransfer);
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
                    device->GetAddress(), buffer.size(), lastTransfer);
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


    /*==========================================================================}
    {      INTERNAL FUNCTIONS THAT OPERATE TO GET DESCRIPTORS FROM DEVICES	    }
    {==========================================================================*/

    void RemoveHidPayload(UsbDevice& device)
    {
        if (device.PayLoadId == PayLoadType::Hid && device.HidPayload != nullptr)
        {
            FreeHidPayload(device.HidPayload);
            device.HidPayload = nullptr;
            device.PayLoadId = PayLoadType::None;
        }
    }

    void RemoveHubPayload(UsbDevice& device) {
        if (device.PayLoadId == PayLoadType::Hub && device.HubPayload) {
            for (auto* pChild : device.HubPayload->Children) {
                if (pChild) {
                    UsbDeallocateDevice(pChild);
                }
            }

            device.HubPayload = nullptr;
            device.PayLoadId = PayLoadType::None;
        }
    }

    uint8_t AllocateDeviceAddress()
    {
        auto const deviceCount = DeviceTable.size();
        for (uint8_t number = 0; number < deviceCount; ++number)
        {	// Search device table entries
            if (!DeviceTable[number])
            {
                return number + 1;
            }
        }
        if (DeviceTable.size() >= UINT8_MAX)
        {
			return 0;
        }
        DeviceTable.resize(deviceCount + 1);
        return static_cast<uint8_t>(deviceCount + 1);
    }

    std::expected<std::shared_ptr<UsbDevice>, RESULT> UsbAllocateDevice(UsbDevice* parentHubDevice, uint8_t parentHubPort) override
    {
        auto const address = AllocateDeviceAddress();
        if (address == 0)
        {
            return std::unexpected(RESULT::ErrorMemory);
        }

        std::shared_ptr<UsbDevice> device = std::make_shared<UsbDevice>(address, DriverRef());
        if (!device)
        {
            return std::unexpected(RESULT::ErrorMemory);
        }

        device->Config.Status = USB_STATUS_ATTACHED;
        device->ParentHub.PortNumber = parentHubPort;
        device->ParentHub.Device = parentHubDevice ? DeviceTable[parentHubDevice->GetAddress() - 1] : nullptr;
        device->PayLoadId = PayLoadType::None;

        DeviceTable[address - 1] = device;
        return std::move(device);
    }

    void UsbDeallocateDevice (struct UsbDevice *device) override
    {
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

        if (auto& parent = device->ParentHub.Device; parent && parent->PayLoadId == PayLoadType::Hub && parent->HubPayload)
        {
            auto const port = static_cast<size_t>(device->ParentHub.PortNumber);
            if (port < parent->HubPayload->Children.size() && parent->HubPayload->Children[port] == device)
            {
                parent->HubPayload->Children[port] = nullptr;
            }
        }

        auto const deviceAddress = device->GetAddress();
        if (deviceAddress > 0 && deviceAddress <= DeviceTable.size())
        {
            DeviceTable[deviceAddress - 1].reset();
        }
    }

    Async::task<IoHandle> InitializeDevice(UsbDevice& device) override
    {
        RESULT result;
        uint8_t address;
        uint32_t transferred;
        DeviceDescriptor desc = { 0 };

        auto ioHandle = GetIoHandle(device.GetAddress());

        LOG_DEBUG("Read first 8 Bytes of Device Descriptor using address 0 to obtain MaxPacketSizeInBytes\n");

        /* Store the unique address until it is actually assigned. */
        address = device.GetAddress();									// Hold unique address we will set device to
        device.Pipe0.Number = 0;										// Initially it starts as zero
        device.Pipe0.MaxPacketSizeInBytes = 8;							// Set max packet size to 8 ( So exchange will be exactly 1 packet)

        result = co_await HCDSubmitControlMessageIN(
            &device,
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
            //co_return result != RESULT::Ok ? result : RESULT::ErrorTransmission;	// Fatal enumeration error of this device
            co_return IoHandle{nullptr};
        }
        else
        {
            LOG_DEBUG("Max packet size: %u\n", desc.bMaxPacketSize0);
            device.Pipe0.MaxPacketSizeInBytes = desc.bMaxPacketSize0;	    // Set the maximum endpoint packet size to pipe from response
            device.Config.Status = USB_STATUS_DEFAULT;						// Move device enumeration to default
        }

        device.Pipe0.Number = address;

        if (device.ParentHub.Device && device.ParentHub.PortNumber > 0)
        {
            LOG_DEBUG("Reset port %u of hub %u (old device support)\n",  device.ParentHub.PortNumber, device.ParentHub.Device->GetAddress());

            // Reset the port for what will be the second time.
            if (auto resetResult = co_await HubPortReset(*device.ParentHub.Device, device.ParentHub.PortNumber - 1); !resetResult.has_value()) {
                LOG("HCD: Failed to reset port again for new device %s.\n", UsbGetDescription(device));
                co_return IoHandle{nullptr}; //result.error();
            }
        }

        // Nothing to do except get a channel to communicate.
        co_return std::move(ioHandle);
    }


    /*==========================================================================}
    {			    NON HCD INTERNAL HUB FUNCTIONS ON PORTS						}
    {==========================================================================*/

    /*==========================================================================}
    {						 INTERNAL ENUMERATION ROUTINES						}
    {==========================================================================*/

    /***************************************************************************}
    {					      PUBLIC INTERFACE ROUTINES			                }
    ****************************************************************************/

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

        auto rootHubDeviceEx = UsbAllocateDevice(nullptr, 0);
        if (!rootHubDeviceEx.has_value())
        {
            error_ = rootHubDeviceEx.error();
            co_return;
        }

        UsbDevice& rootHubDevice = *rootHubDeviceEx.value();
        rootHubDevice.Pipe0.Speed = USB_SPEED_HIGH;
        rootHubDevice.Config.Status = USB_STATUS_ATTACHED;
        rootHubDevice.PayLoadId = PayLoadType::None;
        rootHubDevice.HubPayload = nullptr;

        auto const result = co_await EnumerateDevice(rootHubDevice);
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
    {					 PUBLIC DISPLAY USB INTERFACE ROUTINES					}
    {--------------------------------------------------------------------------*/

    // Sends/recieves data from/to the given buffer to/from the given endpoint.
    Async::task<RESULT> HCDEndpointTransfer(UsbDevice* device, UsbEndpointDescriptor endpoint, std::byte* buffer, uint32_t& bufferLength) override
    {
        LOG_DEBUG("HCD: %s transfer called for device %i, endpoint %i for %u bytes.\n",
            endpoint.EndpointAddress.Direction == USB_DIRECTION_IN ? "IN" : "OUT",
            device->GetAddress(), endpoint.EndpointAddress.Number, bufferLength);

        auto const channel = Host->GetChannel();

        // Set up the pipe for interrupt transfer
        UsbPipe pipe = {
            .MaxPacketSizeInBytes = endpoint.Packet.MaxSize,         // Endpoint max packet size
            .Speed               = device->Pipe0.Speed,             // Same speed as device
            .EndPoint            = endpoint.EndpointAddress.Number, // Endpoint address
            .Number              = device->GetAddress(),            // Same device address
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
    std::println("Initializing DesignWare USB Driver");
    auto result = std::make_shared<DesignWareUsbDriver>();
    co_await result->Initialize();
    co_return result;
};
