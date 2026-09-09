#include "UsbDriver.h"

#include "HidUsbDevices.h"

#include "emb-stdio.h"

#include <optional>

#define LOG(...) printf(__VA_ARGS__)
#define LOG_DEBUG(...) LOG(__VA_ARGS__)

uint8_t GetHidCount(HidDevice* device);
void PrintHid(HidDevice* device, uint8_t hidIndex, char const* indent);

HidDevice* AllocateHidPayload();
bool SetHidDescriptor(HidDevice* hidDevice, uint8_t hidIndex, uint8_t interface, std::byte const* buffer, uint8_t size);

Async::task<RESULT> EnumerateHub(UsbDevice& device);
Async::task<RESULT> EnumerateHID(UsbDevice& device);

// ---------------------------------------------------------------------------------------------------------------------
// Device functions

size_t UsbDevice::GetDeviceProductString(std::span<char> buffer)
{
    if (buffer.size() == 0) return 0;
    if (Descriptor.iProduct == 0) return 0;
    size_t length = buffer.size();
    if (Async::WaitOnTask(driver_->HCDReadStringDescriptor(*this, Descriptor.iProduct, buffer.data(), length)) != RESULT::Ok)
    {
        return 0;
    }
    return length;
}

size_t UsbDevice::GetDeviceManufacturerString(std::span<char> buffer)
{
    if (buffer.size() == 0) return 0;
    if (Descriptor.iManufacturer == 0) return 0;
    size_t length = buffer.size();
    if (Async::WaitOnTask(driver_->HCDReadStringDescriptor(*this, Descriptor.iManufacturer, buffer.data(), length)) != RESULT::Ok)
    {
        return 0;
    }
    return length;
}

size_t UsbDevice::GetDeviceSerialNumberString(std::span<char> buffer)
{
    if (buffer.size() == 0) return 0;
    if (Descriptor.iSerialNumber == 0) return 0;
    size_t length = buffer.size();
    if (Async::WaitOnTask(driver_->HCDReadStringDescriptor(*this, Descriptor.iSerialNumber, buffer.data(), length)) != RESULT::Ok)
    {
        return 0;
    }
    return length;
}

size_t UsbDevice::GetDeviceConfigStringString(std::span<char> buffer)
{
    if (buffer.size() == 0) return 0;
    if (Config.ConfigStringIndex == 0) return 0;
    size_t length = buffer.size();
    if (Async::WaitOnTask(driver_->HCDReadStringDescriptor(*this, Config.ConfigStringIndex, buffer.data(), length)) != RESULT::Ok)
    {
        return 0;
    }
    return length;
}

UsbEndpointDescriptor UsbDevice::FindEndpoint(uint8_t interfaceIndex, usb_transfer_type type, UsbDirection direction) const
{
    if (interfaceIndex >= Endpoints.size())
    {
        return {};
    }
    for (auto const& endpoint : Endpoints[interfaceIndex])
    {
        if (endpoint.Attributes.Type == type && endpoint.EndpointAddress.Direction == direction)
        {
            return endpoint;
        }
    }
    return NullEndpointDescriptor;
}


// ---------------------------------------------------------------------------------------------------------------------
// Driver functions

Async::task<RESULT> UsbDriver::HCDSubmitControlMessageOUT(
    UsbDevice* device,
    std::span<std::byte const> buffer, // Data buffer to send
    UsbDeviceRequest request,	// USB request message
    uint32_t timeout,					// Timeout in microseconds on message
    uint32_t* bytesTransferred			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
)
{
    if (request.Type & 0x80)
    {
        LOG("HCDSubmitControlMessageOUT called with IN request type: %#x\n", request.Type);
        co_return RESULT::ErrorArgument;
    }

    auto const handle = GetIoHandle(device->GetAddress());

    // Just returning the task from HCDSubmitControlMessage is tempting, but...
    // In order to respect the lifetime of the channel, we need this to be a proper coroutine.
    co_return co_await HCDSubmitControlMessageOUT(
        device,
        handle,
        buffer,
        request,
        timeout,
        bytesTransferred
    );
}

Async::task<RESULT> UsbDriver::HCDSubmitControlMessageIN(
    UsbDevice* device,
    std::span<std::byte> buffer,					// Data buffer both send and recieve				 
    UsbDeviceRequest request,	// USB request message
    uint32_t timeout,					// Timeout in microseconds on message
    uint32_t* bytesTransferred			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)				
)
{
    if (!(request.Type & 0x80)) {
        LOG("HCDSubmitControlMessageIN called with OUT request type: %#x\n", request.Type);
        co_return RESULT::ErrorArgument;
    }

    auto const handle = GetIoHandle(device->GetAddress());

    // Just returning the task from HCDSubmitControlMessage is tempting, but...
    // In order to respect the lifetime of the channel, we need this to be a proper coroutine.
    co_return co_await HCDSubmitControlMessageIN(
        device,
        handle,
        buffer,
        request,
        timeout,
        bytesTransferred
    );
}

// Has the ability to fetches all the different descriptors from the device if
// you provide the right parameters. It is a marshal call that many internal
// descriptor reads will use and it has no checking on parameters. So if you
// provide invalid parameters it will most likely fail and return with error.
// The descriptor is read in two calls first the header is read to check the
// type matches and it provides the descriptor size. If the buffer length is
// longer than the descriptor the second call shortens the length to just the
// descriptor length. So the call provides the length of data requested or
// shorter if the descriptor is shorter than the buffer space provided.
Async::task<RESULT> HCDGetDescriptor (UsbDevice& device,
                        enum usb_descriptor_type type,				// The type of descriptor
                        uint8_t index,								// The index of the type descriptor
                        uint16_t langId,							// The language id
                        void* buffer,								// Buffer to recieve descriptor
                        uint32_t length,							// Maximumlength of descriptor
                        uint8_t recipient,							// Recipient flags									 
                        uint32_t *bytesTransferred,     			// Value at pointer will be updated with bytes transfered to/from buffer (NULL to ignore)								
                        bool runHeaderCheck)						// Whether to run header check
{
    auto& driver = device.GetDriver();

    auto const ioHandle = driver.GetIoHandle(device.GetAddress());

    RESULT result;
    uint32_t transfer;
    alignas(4) struct UsbDescriptorHeader header  = { 0 };
    if (runHeaderCheck) {
        result = co_await driver.HCDSubmitControlMessageIN(
            &device,													// Pipe passed in as is
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
                type, header.DescriptorType, device.GetAddress());	// Log error
            result = RESULT::ErrorGeneral;									// For some strange reason descriptor type is not right
        }
        if (result != RESULT::Ok) {											// RESULT in error
            LOG("HCD: Fail to get descriptor header %#x:%#x recepient: %#x, device:%i. RESULT %#x.\n",
                type, index, recipient, device.GetAddress(), result);		// Log any error
            co_return result;
        }
        if (length > header.DescriptorLength)						// Check descriptor length vs buffer space
            length = header.DescriptorLength;						// The descriptor is shorter than buffer space provided
    }
    result = co_await driver.HCDSubmitControlMessageIN(
        &device,														// Pipe passed in as is
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
            type, index, recipient, device.GetAddress(), result);
    }
    if (bytesTransferred) *bytesTransferred = transfer;
    co_return result;
}

Async::task<RESULT> UsbDriver::HCDReadStringDescriptor (UsbDevice& device,
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
            device.GetAddress());											// Log the error
        co_return RESULT::ErrorArgument;										// I am lost what is going on bail
    }

    // langIds 0 actually has 0x03 (string descriptor) and size of language support words .. if it doesn't bail
    if ((langIds[0] >> 8) != 0x03) {								// The top byte has to be 0x03
        LOG("HCD: Not a valid language support descriptor on device: %i\n",
            device.GetAddress());											// Log the error
        co_return RESULT::ErrorArgument;										// I am lost what is going on bail
    }
    // So we have size to read for all the language support pairs
    result = co_await HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_STRING, 0, 0, &langIds, langIds[0] & 0xFF,
        bmREQ_GET_DEVICE_DESCRIPTOR, &transfer, true);				// Get all language support pair data
    if ((result != RESULT::Ok) && (transfer < (langIds[0] & 0xFF))) {		// We failed to read all the support data
        LOG("HCD: Could not read all the language support data on device: %i\n",
            device.GetAddress());											// Log the error		
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
            device.GetAddress());											// Log the error
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
            stringIndex, device.GetAddress());								// Log the error
        co_return RESULT::ErrorDevice;											// No idea what problem is so bail										
    }

    // Okay we got the size of the string so now read the entire size
    result = co_await HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_STRING, stringIndex,
        NoEnglishSupport ? langIds[1] : 0x409, &descBuffer,
        Header.DescriptorLength, bmREQ_GET_DEVICE_DESCRIPTOR, 
        &transfer, true);											// Read the full string 	
    if ((result != RESULT::Ok) || (transfer != Header.DescriptorLength)) {
        LOG("HCD: Could not fetch string descriptor (%i) for device: %i\n",
            stringIndex, device.GetAddress());								// Log the error
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

// Reads the given port status on a hub device. Port input is index 1 and so
// requesting port 0 is interpretted as you want the port gateway node status.
// When reading a port the return is really a HubPortFullStatus, while for
// port = 0 the return will be a struct HubFullStatus. There are uint32_t unions
// on those two structures to pass the raw 32 bits in/out.
Async::task<RESULT> UsbDriver::HCDReadHubPortStatus (UsbDevice* device,
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
            device->GetAddress(), port, result, device->Pipe0.Speed, device->Pipe0.MaxPacketSizeInBytes);	// Log any error
        co_return result;												// Return error result
    }
    if (transfer < sizeof(uint32_t)) {								// Hub did not read amount requested
        LOG("HUB: Failed to read hub device:%i port:%i status\n",
            device->GetAddress(), port);										// Log error
        co_return RESULT::ErrorDevice;											// Some quirk in enumeration usually
    }
    co_return RESULT::Ok;														// Return success
}

// Changes a feature setting on the given port on a hub device. Port input is
// index 1 and so requesting port 0 is interpretted as you are changing the
// feature on the port gateway node.
Async::task<RESULT> HCDChangeHubPortFeature (UsbDevice* device,
                                HubPortFeature feature,		// Which feature to change
                                uint8_t port,						// Port to change feature  OR  0 = Gateway node
                                bool set)							// Set or clear the feature
{
    auto& driver = device->GetDriver();

    auto const result = co_await driver.HCDSubmitControlMessageOUT(
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
            device->GetAddress(), port, feature, set);						// Log any error
        co_return result;
    }
    co_return RESULT::Ok;
}


Async::task<std::expected<HubPortFullStatus, RESULT>> UsbDriver::HubPortReset(UsbDevice& device, uint8_t port)
{
    RESULT result;
    struct HubPortFullStatus portStatus;
    uint32_t retry, timeout;
    if (!device.IsHub()) co_return std::unexpected(RESULT::ErrorDevice);
    LOG_DEBUG("HUB: Reseting device: %u Port: %u. source: %i\n", device.GetAddress(), port, 0/*source*/);
    for (retry = 0; retry < 3; retry++) {
        if ((result = co_await HCDChangeHubPortFeature(&device,
            FeatureReset, port + 1, true)) != RESULT::Ok) 					// Issue a setfeature of reset
        {
            LOG("HUB: Device %i Failed to reset Port%d.\n",
                device.GetAddress(), port + 1);
            co_return std::unexpected(result);											// Return result that is causing failure
        }
        timeout = 0;
        do {
            co_await Async::Delay(20ms);
            if ((result = co_await HCDReadHubPortStatus(&device, port + 1, portStatus.Raw32)) != RESULT::Ok) {
                LOG("HUB: Hub failed to get status (4) for %s.Port%d.\n", UsbGetDescription(device), port + 1);
                co_return std::unexpected(result);
            }
            timeout++;
        } while (!portStatus.Change.ResetChanged && !portStatus.Status.Enabled && timeout < 10);

        if (timeout == 10) continue;

        LOG_DEBUG("HUB: %s.Port%d Status %x:%x.\n", UsbGetDescription(device), port + 1, portStatus.RawStatus, portStatus.RawChange);

        if (portStatus.Change.ConnectedChanged || !portStatus.Status.Connected)
            co_return std::unexpected(RESULT::ErrorDevice);

        if (portStatus.Status.Enabled)
            break;
    }

    if (retry == 3) {
        LOG("HUB: Cannot enable %s.Port%d. Please verify the hardware is working.\n", UsbGetDescription(device), port + 1);
        co_return std::unexpected(RESULT::ErrorDevice);
    }

    if ((result = co_await HCDChangeHubPortFeature(&device, FeatureResetChange, port + 1, false)) != RESULT::Ok) {
        LOG("HUB: Failed to clear reset on %s.Port%d.\n", UsbGetDescription(device), port + 1);
    }
    co_return portStatus;
}

// Sets the address of the device with control endpoint given by the pipe. Zero
// is a restricted address for the rootHub and will return if attempted.
Async::task<RESULT> UsbDriver::HCDSetAddress(UsbDevice& device, IoHandle const& ioHandle)
{
    auto const address = device.GetAddress();
    device.Pipe0.Number = 0;
    if (address == 0) co_return RESULT::ErrorArgument;							// You can't set address zero that is strictly reserved for roothub
    auto result = co_await HCDSubmitControlMessageOUT(
        &device,
        ioHandle,
        {},													// No data
        UsbDeviceRequest {
            .Type = 0,
            .Request = SetAddress,									// Set address request
            .Value = address,										// Address to set
        },
        ControlMessageTimeout,
        nullptr);
    device.Pipe0.Number = address;
    co_return result;
}

// Sets a given USB device configuration to the config index number requested.
Async::task<RESULT> UsbDriver::HCDSetConfiguration (UsbDevice* device, IoHandle const& ioHandle, uint8_t configuration)
{
    return HCDSubmitControlMessageOUT(
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

    device.PayLoadId = PayLoadType::Hid;
    return RESULT::Ok;
}

// Returns a description for a device. This is not read from the device, this
// is just generated given by the driver.
const char* UsbDriver::UsbGetDescription(UsbDevice& device)
{
    if (device.Config.Status == USB_STATUS_ATTACHED)
        return "New Device (Not Ready)";
    else if (device.Config.Status == USB_STATUS_POWERED)
        return "Unknown Device (Not Ready)";
    else if (!device.ParentHub.Device)
        return "USB Root device (Hub?)";

    switch (device.Descriptor.bDeviceClass) {
    case DeviceClassHub:
        if (device.Descriptor.bcdUSB == 0x210)
            return "USB 2.1 Hub";
        else if (device.Descriptor.bcdUSB == 0x200)
            return "USB 2.0 Hub";
        else if (device.Descriptor.bcdUSB == 0x110)
            return "USB 1.1 Hub";
        else if (device.Descriptor.bcdUSB == 0x100)
            return "USB 1.0 Hub";
        else
            return "USB Hub";
    case DeviceClassVendorSpecific:
        if (device.Descriptor.idVendor == 0x424 &&
            device.Descriptor.idProduct == 0xec00)
            return "SMSC LAN9512";
    case DeviceClassInInterface:
        if (device.Config.Status == USB_STATUS_CONFIGURED) {
            switch (device.Interfaces[0].Class) {
            case InterfaceClass::Audio:
                return "USB Audio Device";
            case InterfaceClass::Communications:
                return "USB CDC Device";
            case InterfaceClass::Hid:
                switch (device.Interfaces[0].Protocol) {
                case 1:
                    return "USB Keyboard";
                case 2:
                    return "USB Mouse";
                default:
                    return "USB HID";
                }
            case InterfaceClass::Physical:
                return "USB Physical Device";
            case InterfaceClass::Image:
                return "USB Imaging Device";
            case InterfaceClass::Printer:
                return "USB Printer";
            case InterfaceClass::MassStorage:
                return "USB Mass Storage Device";
            case InterfaceClass::Hub:
                if (device.Descriptor.bcdUSB == 0x210)
                    return "USB 2.1 Hub";
                else if (device.Descriptor.bcdUSB == 0x200)
                    return "USB 2.0 Hub";
                else if (device.Descriptor.bcdUSB == 0x110)
                    return "USB 1.1 Hub";
                else if (device.Descriptor.bcdUSB == 0x100)
                    return "USB 1.0 Hub";
                else
                    return "USB Hub";
            case InterfaceClass::CdcData:
                return "USB CDC-Data Device";
            case InterfaceClass::SmartCard:
                return "USB Smart Card";
            case InterfaceClass::ContentSecurity:
                return "USB Content Secuity Device";
            case InterfaceClass::Video:
                return "USB Video Device";
            case InterfaceClass::PersonalHealthcare:
                return "USB Healthcare Device";
            case InterfaceClass::AudioVideo:
                return "USB AV Device";
            case InterfaceClass::DiagnosticDevice:
                return "USB Diagnostic Device";
            case InterfaceClass::WirelessController:
                return "USB Wireless Controller";
            case InterfaceClass::Miscellaneous:
                return "USB Miscellaneous Device";
            case InterfaceClass::VendorSpecific:
                return "Vendor Specific";
            default:
                return "Generic Device";
            }
        }
        else if (device.Descriptor.bDeviceClass == DeviceClassVendorSpecific)
            return "Vendor Specific";
        else
            return "Unconfigured Device";
    default:
        return "Generic Device";
    }
}

// All detected devices start enumeration here. We recover critical information
// of every USB device and hold those details in the device data block. Finally 
// if the device is recognized as any of the sepcial specific class then it will
// call extended enumeration for those specific classes.
Async::task<RESULT> UsbDriver::EnumerateDevice(UsbDevice& device)
{
    RESULT result;
    uint32_t transferred;
    char buffer[256] __attribute__((aligned(4)));					// Text buffer

    if (device.ParentHub.Device)
    {
        LOG_DEBUG("\n---\nUSB ENUMERATION of device on port %u of hub %u (off of root port %u)\n", device.ParentHub.PortNumber, device.ParentHub.Device->GetAddress(), device.ParentHub.Device->RootHubPort);
    }
    else
    {
        LOG_DEBUG("\n---\nUSB ENUMERATION of device on root port %u\n", device.RootHubPort);
    }

    LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 1 & 2 = initialize the device\n");

    auto const ioHandle = co_await InitializeDevice(device);
    if (!ioHandle)
    {
        LOG("Enumeration: Failed to initialize device %i.\n", device.GetAddress());
        co_return RESULT::ErrorGeneral;
    }

    LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 3 = Set Device Address %u\n", device.GetAddress());
    if ((result = co_await HCDSetAddress(device, ioHandle)) != RESULT::Ok)
    {
        LOG("Enumeration: Failed to assign address to %#x.\n", device.GetAddress());// Log the error
        co_return result;												// Fatal enumeration error of this device
    }
    co_await Async::Delay(10ms);												// Allows time for address to propagate.
    device.Config.Status = USB_STATUS_ADDRESSED;					// Our enumeration status in now addressed

    LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 4 = Read Device Descriptor At Address\n");
    result = co_await HCDGetDescriptor(
        device,												// Device control 0 pipe
        USB_DESCRIPTOR_TYPE_DEVICE,							        // Fetch device descriptor 
        0,															// Index 0
        0,															// Language 0
        &device.Descriptor,										// Pointer to buffer in device structure 
        sizeof(device.Descriptor),									// Ask for entire descriptor
        bmREQ_GET_DEVICE_DESCRIPTOR,								// Recipient device
        &transferred, true);										// Pass in pointer to get bytes transferred back
    if (result == RESULT::Ok && transferred != sizeof(device.Descriptor))
    {
        // This should pass on any valid device
        LOG("Enumeration: Step 4 on device %i failed, Got %u bytes != %u.\n",
            device.GetAddress(), transferred, (uint32_t)sizeof(device.Descriptor));
        co_return RESULT::ErrorTransmission;
    }
    if (result != RESULT::Ok)
    {
        LOG("Enumeration: Step 4 on device %i failed, Result: %#x.\n",
            device.GetAddress(), result);						// Log any error
        co_return result;
    }
    LOG_DEBUG("Device: %u, Class: %u, Subclass: %u\n", device.GetAddress(), device.Descriptor.bDeviceClass, device.Descriptor.bDeviceSubClass);


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
            transfer, (uint32_t)sizeof(device.Descriptor), device.GetAddress());
        co_return RESULT::ErrorTransmission;
    }
    if (result != RESULT::Ok) {
        LOG("HCD: Error: %i, reading configuration descriptor for device: %i\n",
            result, device.GetAddress());
        co_return RESULT::ErrorDevice;											// No idea what problem is so bail
    }
    device.Config.ConfigStringIndex = configDesc.iConfiguration;	// Grab string index while here

    // Most devices I played with only have 1 config .. regardless we will take first
    // The index to call is given as at offset 5 bConfigurationValue
    // Read it by that index it's probably the same but just do it
    uint8_t configNum = configDesc.bConfigurationValue;
    // Okay we have the total length of config so we will read it in entirity
    std::byte configBuffer[1024];										// Largest config I have ever seen is few hundred bytes this is 1K buffer
    result = co_await HCDSubmitControlMessageIN(
        &device,												// Device 
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
            device.GetAddress(), (unsigned int)transfer, result);				// Log error
        if (result != RESULT::Ok) co_return result;							// Return error result
        co_return RESULT::ErrorDevice;											// Something went badly wrong .. bail
    }

    device.Interfaces.clear();
    device.Endpoints.clear();

    uint8_t hidCount = 0;
    std::optional<uint8_t> currentInterfaceIndex;
    uint32_t i = 0;
    while (i < configDesc.wTotalLength - 1) {
        switch (static_cast<usb_descriptor_type>(configBuffer[i + 1])) {
        case USB_DESCRIPTOR_TYPE_INTERFACE: {
            UsbInterfaceDescriptor interfaceDescriptor{};
            memcpy(&interfaceDescriptor, &configBuffer[i], sizeof(interfaceDescriptor));
            device.Interfaces.push_back(interfaceDescriptor);
            device.Endpoints.emplace_back();
            currentInterfaceIndex = static_cast<uint8_t>(device.Interfaces.size() - 1);
            break;
        }
        case USB_DESCRIPTOR_TYPE_ENDPOINT: {
            if (!currentInterfaceIndex.has_value()) {
                break;
            }
            UsbEndpointDescriptor endpointDescriptor{};
            memcpy(&endpointDescriptor, &configBuffer[i], sizeof(endpointDescriptor));
            device.Endpoints[*currentInterfaceIndex].push_back(endpointDescriptor);
            break;
        }
        case USB_DESCRIPTOR_TYPE_HID: {
            if (!currentInterfaceIndex.has_value()) {
                break;
            }
            if (hidCount == 0) {
                if ((result = AddHidPayload(device)) != RESULT::Ok) {
                    LOG("Could not allocate hid payload, Error ID %i\n", result);
                    co_return result;
                }
            }
            if (SetHidDescriptor(device.HidPayload, hidCount, *currentInterfaceIndex, &configBuffer[i], static_cast<uint8_t>(configBuffer[i]))) {
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
    if (auto const thisResult = co_await HCDSetConfiguration(&device, ioHandle, configNum); thisResult != RESULT::Ok) {
        LOG("HCD: Failed to set configuration %#x for device %i.\n",
            configNum, device.GetAddress());
        co_return thisResult;
    }
    device.Config.ConfigIndex = configNum;							// Hold the configuration index
    device.Config.Status = USB_STATUS_CONFIGURED;					// Set device status to configured

    LOG("HCD: Attach Device %s. Address:%d Class:%d USB:%x.%x, %d configuration(s), %d interface(s).\n",
        UsbGetDescription(device), device.GetAddress(), device.Descriptor.bDeviceClass, (device.Descriptor.bcdUSB >> 8) & 0xFF,
        device.Descriptor.bcdUSB & 0xFF, device.Descriptor.bNumConfigurations, device.Interfaces.size());
    
    if (device.Descriptor.iProduct != 0) {
        size_t length = sizeof(buffer);
        if (co_await HCDReadStringDescriptor(device, device.Descriptor.iProduct, &buffer[0], length) == RESULT::Ok)
        {
            LOG("HCD:  -Product:       %s.\n", buffer);
        }
    }
    
    if (device.Descriptor.iManufacturer != 0) {
        size_t length = sizeof(buffer);
        if (co_await HCDReadStringDescriptor(device, device.Descriptor.iManufacturer, &buffer[0], length) == RESULT::Ok)
        {
            LOG("HCD:  -Manufacturer:  %s.\n", buffer);
        }
    }
    if (device.Descriptor.iSerialNumber != 0) {
        size_t length = sizeof(buffer);
        if (co_await HCDReadStringDescriptor(device, device.Descriptor.iSerialNumber, &buffer[0], length) == RESULT::Ok)
        {
            LOG("HCD:  -SerialNumber:  %s.\n", buffer);
        }
    }


    if (device.Config.ConfigStringIndex != 0) {
        size_t length = sizeof(buffer);
        if (co_await HCDReadStringDescriptor(device, device.Config.ConfigStringIndex, &buffer[0], length) == RESULT::Ok)
        {
            LOG("HCD:  -Configuration: %s.\n", buffer);
        }
    }


    LOG_DEBUG("\n---\nUSB ENUMERATION BY THE BOOK STEP 7 = ENUMERATE SPECIAL DEVICES\n");
    if (device.Descriptor.bDeviceClass == DeviceClassHub) {		// If device is a hub then enumerate it
        LOG_DEBUG("Device is a hub, enumerating ports.\n");
        if ((result = co_await EnumerateHub(device)) != RESULT::Ok) {
            LOG("Could not enumerate HUB device %i, Error ID %i\n",
                device.GetAddress(), result);						// Log error
            co_return result;											// Return the error
        }
    } else if (hidCount > 0) {										// HID interface on the device
        LOG_DEBUG("Device hidCount: %u, enumerating ports.\n", hidCount);
        if ((result = co_await EnumerateHID(device)) != RESULT::Ok) {	// RESULT::Ok so enumerate the HID device
            LOG("Could not enumerate HID device %i, Error ID %i\n",
                device.GetAddress(), result);
            co_return result;											// return the error
        }
    }
    else {														// If not a hub or HID then just log the device
        LOG_DEBUG("Device is not a hub or HID, skipping enumeration.\n");
    }

    co_return RESULT::Ok;
}

RESULT AddHubPayload(UsbDevice& device)
{
    if (device.PayLoadId == PayLoadType::None || (device.PayLoadId == PayLoadType::Hub && device.HubPayload == nullptr)) {
        auto hub = std::make_shared<HubDevice>();
        if (!hub) {
            return RESULT::ErrorMemory;
        }
        device.HubPayload = hub.get();
        device.PayLoadId = PayLoadType::Hub;
        return RESULT::Ok;
    }
    return RESULT::ErrorArgument;
}

// If a connection on a port on a hub as changed this routine is called to deal
// with the change. This will involve it enumerating an added new device or the
// deallocation of a removed or detached device.
__attribute__((noinline)) Async::task<RESULT> HubPortConnectionChanged(UsbDevice& device, uint8_t port)
{
    RESULT result;
    struct HubDevice *data;
    struct HubPortFullStatus portStatus;
    if (!device.IsHub()) co_return RESULT::ErrorDevice;

    data = device.HubPayload;

    auto& driver = device.GetDriver();

    if ((result = co_await driver.HCDReadHubPortStatus(&device, port + 1, portStatus.Raw32)) != RESULT::Ok) {
        LOG("HUB: Hub failed to get status (2) for %s.Port%d.\n", driver.UsbGetDescription(device), port + 1);
        co_return result;
    }
    LOG_DEBUG("HUB: %s.Port%d Status %x:%x.\n", driver.UsbGetDescription(device), port + 1, portStatus.RawStatus, portStatus.RawChange);

    if ((result = co_await HCDChangeHubPortFeature(&device, FeatureConnectionChange, port + 1, false)) != RESULT::Ok) {
        LOG("HUB: Failed to clear change on %s.Port%d.\n", driver.UsbGetDescription(device), port + 1);
    }

    if ((!portStatus.Status.Connected && !portStatus.Status.Enabled) || data->Children[port] != nullptr) {
        LOG("HUB: Disconnected %s.Port%d - %s.\n", driver.UsbGetDescription(device), port + 1, driver.UsbGetDescription(*data->Children[port]));
        driver.UsbDeallocateDevice(data->Children[port]);
        data->Children[port] = nullptr;
        if (!portStatus.Status.Connected) co_return RESULT::Ok;
    }

    if (auto resetResult = co_await driver.HubPortReset(device, port); !resetResult.has_value()) {
        LOG("HUB: Could not reset %s.Port%d for new device.\n", driver.UsbGetDescription(device), port + 1);
        co_return resetResult.error();
    }

    auto childEx = driver.UsbAllocateDevice(&device, port + 1);
    if (!childEx.has_value()) {
        LOG("HUB: Could not allocate a new device entry for %s.Port%d.\n", driver.UsbGetDescription(device), port + 1);
        co_return childEx.error();
    }

    auto& child = *childEx.value();
    
    data->Children[port] = &child;

    if ((result = co_await driver.HCDReadHubPortStatus(&device, port + 1, portStatus.Raw32)) != RESULT::Ok) {
        LOG("HUB: Hub failed to get status (3) for %s.Port%d.\n", driver.UsbGetDescription(device), port + 1);
        co_return result;
    }

    LOG("HUB: %s. Device:%i Port:%d Status %04x:%04x.\n", driver.UsbGetDescription(device), device.GetAddress(), port, portStatus.RawStatus, portStatus.RawChange);

    if (portStatus.Status.HighSpeedAttatched)
    {
        child.Pipe0.Speed = USB_SPEED_HIGH;
    }
    else if (portStatus.Status.LowSpeedAttatched)
    {
        child.Pipe0.Speed = USB_SPEED_LOW;
        child.Pipe0.splitNodePoint = device.GetAddress();
        child.Pipe0.splitNodePort = port;
    }
    else
    {
        child.Pipe0.Speed = USB_SPEED_FULL;
        child.Pipe0.splitNodePoint = device.GetAddress();
        child.Pipe0.splitNodePort = port;
    }
    child.RootHubPort = device.RootHubPort;
    if ((result = co_await driver.EnumerateDevice(child)) != RESULT::Ok)
    {
        LOG("HUB: Could not connect to new device in %s.Port%d. Disabling.\n", driver.UsbGetDescription(device), port + 1);
        driver.UsbDeallocateDevice(&child);
        data->Children[port] = nullptr;
        if (co_await HCDChangeHubPortFeature(&device, FeatureEnable, port + 1, false) != RESULT::Ok) {
            LOG("HUB: Failed to disable %s.Port%d.\n", driver.UsbGetDescription(device), port + 1);
        }
        co_return result;
    }
    co_return RESULT::Ok;
}


// Checks device is a hub and if a valid hub checks connection status of given
// port on the hub. If it has changed performs necessary actions such as the
// enumerating of a new device or deallocating an old one.
Async::task<RESULT> HubCheckConnection(UsbDevice& device, uint8_t port)
{
    RESULT result;
    HubPortFullStatus portStatus;
    HubDevice *data;

    if (!device.IsHub()) co_return RESULT::ErrorDevice;
    data = device.HubPayload;

    LOG("HUB: Checking connection for device %i, Port: %i.\n", device.GetAddress(), port);

    auto& driver = device.GetDriver();

    if ((result = co_await driver.HCDReadHubPortStatus(&device, port + 1, portStatus.Raw32)) != RESULT::Ok) {
        if (result != RESULT::ErrorDisconnected)
            LOG("HUB: Failed to get hub port status (1) for %s.Port%d.\n", driver.UsbGetDescription(device), port + 1);
        co_return result;
    }

    LOG("HUB: device %i, Port: %i, status: %04X:%04X.\n", device.GetAddress(), port, portStatus.RawStatus, portStatus.RawChange);

    if (portStatus.Change.ConnectedChanged) {
        LOG_DEBUG("Device %i, Port: %i changed\n", device.GetAddress(), port);
        co_await HubPortConnectionChanged(device, port);
    }

    LOG_DEBUG("Device %i, Port: %i checking the rest\n", device.GetAddress(), port);

    if (portStatus.Change.EnabledChanged) {
        if (co_await HCDChangeHubPortFeature(&device, FeatureEnableChange, port + 1, false) != RESULT::Ok) {
            LOG("HUB: Failed to clear enable change %s.Port%d.\n", driver.UsbGetDescription(device), port + 1);
        }

        // This may indicate EM interference.
        if (!portStatus.Status.Enabled && portStatus.Status.Connected && data->Children[port] != nullptr) {
            LOG("HUB: %s.Port%d has been disabled, but is connected. This can be cause by interference. Reenabling!\n", driver.UsbGetDescription(device), port + 1);
            co_await HubPortConnectionChanged(device, port);
        }
    }

    if (portStatus.Status.Suspended) {
        if (co_await HCDChangeHubPortFeature(&device, FeatureSuspend, port + 1, false) != RESULT::Ok) {
            LOG("HUB: Failed to clear suspended port - %s.Port%d.\n", driver.UsbGetDescription(device), port + 1);
        }
    }

    if (portStatus.Change.OverCurrentChanged) {
        if (co_await HCDChangeHubPortFeature(&device, FeatureOverCurrentChange, port + 1, false) != RESULT::Ok) {
            LOG("HUB: Failed to clear over current port - %s.Port%d.\n", driver.UsbGetDescription(device), port + 1);
        }
    }

    if (portStatus.Change.ResetChanged) {
        if (co_await HCDChangeHubPortFeature(&device, FeatureResetChange, port + 1, false) != RESULT::Ok) {
            LOG("HUB: Failed to clear reset port - %s.Port%d.\n", driver.UsbGetDescription(device), port + 1);
        }
    }

    co_return RESULT::Ok;
}

// This performs an iteration loop to check each port on each hub to see if any
// device has been added or removed.
Async::task<void> HubCheckForChange(UsbDevice& device)
{
    if (device.IsHub()) {
        for (size_t i = 0; i < device.HubPayload->Children.size(); i++) {
            if (co_await HubCheckConnection(device, static_cast<uint8_t>(i)) != RESULT::Ok) continue;
            if (device.HubPayload->Children[i] != nullptr)
                co_await HubCheckForChange(*device.HubPayload->Children[i]);
        }
    }
}

// Recursively calls HubCheckConnection on all ports on all hubs connected to
// the root hub. It will hence automatically change the device tree matching
// any physical changes.
Async::task<void> UsbDriver::UsbCheckForChange()
{
    if (auto* const hub = UsbGetRootHub())
    {
        return HubCheckForChange(*hub);
    }
    else
    {
        return {};
    }
}

// Continues enumeration of each port if an enumerated detected device is a hub
Async::task<RESULT> UsbDriver::EnumerateHub(UsbDevice& device)
{
    RESULT result;
    uint32_t transfer;
    HubDevice *data;
    HubFullStatus status;

    if (auto const thisResult = AddHubPayload(device); thisResult != RESULT::Ok) {
        LOG("Could not allocate hub payload, Error ID %i\n", thisResult);
        co_return thisResult;
    }

    data = device.HubPayload;

    result = co_await HCDGetDescriptor(device, USB_DESCRIPTOR_TYPE_HUB,
        0, 0, &data->Descriptor, sizeof(HubDescriptor),
        bmREQ_GET_HUB_DESCRIPTOR, &transfer, true);
    if ((result != RESULT::Ok) || (transfer != sizeof(HubDescriptor)))
    {
        LOG("HCD: Could not fetch hub descriptor for device: %i\n",
            device.GetAddress());
        co_return RESULT::ErrorDevice;
    }
    LOG_DEBUG("Hub device %i has %i ports\n", device.GetAddress(), data->Descriptor.PortCount);
    LOG_DEBUG("HUB: Hub power to good: %dms.\n", data->Descriptor.PowerGoodDelay * 2);
    LOG_DEBUG("HUB: Hub current required: %dmA.\n", data->Descriptor.MaximumHubPower * 2);

    data->Children.assign(data->Descriptor.PortCount, nullptr);

    if (auto const thisResult = co_await HCDReadHubPortStatus(&device, 0, status.Raw32); thisResult != RESULT::Ok)
    {
        LOG("HUB device:%i failed to get hub status.\n", device.GetAddress());
        co_return thisResult;
    }

    LOG("HUB: Hub powering ports on.\n");
    for (size_t i = 0; i < data->Children.size(); i++) {
        if (co_await HCDChangeHubPortFeature(&device, FeaturePower, static_cast<uint8_t>(i + 1), true) != RESULT::Ok)
            LOG("HUB: device: %i could not power Port%d.\n", device.GetAddress(), i + 1);
    }
    co_await Async::Delay(data->Descriptor.PowerGoodDelay * 2ms);
    /*co_await Async*/ Cpu::Delay(1ms);

    LOG("HUB: device: %i checking %u port connections.\n", device.GetAddress(), data->Children.size());

    for (size_t port = 0; port < data->Children.size(); port++) {
        co_await HubCheckConnection(device, static_cast<uint8_t>(port));
    }

    co_return RESULT::Ok;														// Return success
}


static void UsbShowTree(UsbDevice *root, const int level, const char tee)
{
    static int TreeLevelInUse[20] = { 0 }; // TODO: Use local state, not global

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
        if (root->IsHub())
        {
            bool drawLine = false;
            uint32_t lastChild = root->HubPayload->Children.size();
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
        tee, root->GetDriver().UsbGetDescription(*root),
        root->GetAddress(),
        root->ParentHub.PortNumber,
        SpeedString[root->Pipe0.Speed],
        root->Pipe0.MaxPacketSizeInBytes,
        root->IsHid() ? "- HID interface" : ""
    );

    bool verbose = true;

    if (verbose)
    {
        printf("%s  config: %u configString: %u status: %u interfaces: %u DescriptorType %u bcdUSB %X\n",
            indent,
            root->Config.ConfigIndex,
            root->Config.ConfigStringIndex,
            root->Config.Status,
            root->Interfaces.size(),
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
        for (size_t i = 0; i < root->Interfaces.size(); i++)
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
            for (size_t j = 0; j < root->Endpoints[i].size(); j++) {
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
        if (root->IsHid())
        {
            for (uint8_t i = 0; i < GetHidCount(root->HidPayload); i++)
            {
                PrintHid(root->HidPayload, i, indent);
            }
        }
    }
    if (root->IsHub())
    {
        uint32_t lastChild = root->HubPayload->Children.size();
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

void UsbDriver::UsbShowTree()
{
    ::UsbShowTree(UsbGetRootHub(), 1, '+');
}
