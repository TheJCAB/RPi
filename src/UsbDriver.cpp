#include "UsbDriver.h"

#include "HidUsbDevices.h"


#include "emb-stdio.h"

uint8_t GetHidCount(HidDevice* device);
void PrintHid(HidDevice* device, uint8_t hidIndex, char const* indent);

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
    result = co_await HCDGetDescriptor(&device, USB_DESCRIPTOR_TYPE_STRING, 0, 0, &langIds, 2,
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
    result = co_await HCDGetDescriptor(&device, USB_DESCRIPTOR_TYPE_STRING, 0, 0, &langIds, langIds[0] & 0xFF,
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
    result = co_await HCDGetDescriptor(&device, USB_DESCRIPTOR_TYPE_STRING, stringIndex,
        NoEnglishSupport ? langIds[1] : 0x409, &Header,
        sizeof(struct UsbDescriptorHeader), bmREQ_GET_DEVICE_DESCRIPTOR, 
        &transfer, true);											// Read string descriptor header only
    if ((result != RESULT::Ok) || (transfer != sizeof(struct UsbDescriptorHeader))) {
        LOG("HCD: Could not fetch string descriptor header (%i) for device: %i\n",
            stringIndex, device.GetAddress());								// Log the error
        co_return RESULT::ErrorDevice;											// No idea what problem is so bail										
    }

    // Okay we got the size of the string so now read the entire size
    result = co_await HCDGetDescriptor(&device, USB_DESCRIPTOR_TYPE_STRING, stringIndex,
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
        tee, root->GetDriver().UsbGetDescription(root),
        root->Pipe0.Number,
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
