#pragma once

#include <stdint.h>
#include <stddef.h>

#include <memory>
#include <span>
#include <vector>

#include "UsbSpec.h"

namespace PCIe {
    struct Bcm2711Driver;
    struct DeviceAddress;
}

namespace Usb
{

struct DeviceInfo
{
    uint32_t SlotId = 0;
    uint32_t Port = 0;
    uint32_t RootHubPort = 0;
    uint32_t Speed = 0;
    DeviceDescriptor Descriptor{};
    ConfigurationDescriptor Configuration{};
    std::vector<UsbInterfaceDescriptor> Interfaces{};
    std::vector<std::vector<UsbEndpointDescriptor>> Endpoints{};
    bool HasConfiguration = false;
};

enum class Status {
    Success,
    Error,
    Timeout,
    NotFound
};

class Controller
{
public:
    virtual ~Controller() = default;
    
    virtual Status initialize() = 0;
    virtual Status shutdown() = 0;
    
    virtual uint32_t allocate_device_slot() = 0;

    virtual Status read(uint8_t endpoint, std::span<uint8_t> buffer) = 0;
    virtual Status write(uint8_t endpoint, std::span<const uint8_t> data) = 0;
    
    virtual Status controlTransfer(uint8_t requestType, uint8_t request,
                                 uint16_t value, uint16_t index,
                                 std::span<uint8_t> data = {})
    {
        return controlTransfer(0, requestType, request, value, index, data);
    }

    virtual Status controlTransfer(uint8_t slotId, uint8_t requestType, uint8_t request,
                                 uint16_t value, uint16_t index,
                                 std::span<uint8_t> data = {}) = 0;

    virtual std::span<DeviceInfo const> discovered_devices() const = 0;

    virtual void process_pending_events() = 0;

    virtual bool run_hello_world_test() { return true; }
};

namespace Xhci
{
    
    std::unique_ptr<Controller> CreateController(PCIe::Bcm2711Driver&, PCIe::DeviceAddress const&);

} // namespace Xhci

/*
class UsbDevice {
public:
    UsbDevice(UsbController& controller, uint8_t deviceId) 
        : controller_(controller), deviceId_(deviceId) {}
    
    Status connect() {
        return controller_.initialize();
    }
    
    Status disconnect() {
        return controller_.shutdown();
    }
    
    Status sendData(uint8_t endpoint, std::span<const uint8_t> data) {
        return controller_.write(endpoint, data);
    }
    
    Status receiveData(uint8_t endpoint, std::span<uint8_t> buffer) {
        return controller_.read(endpoint, buffer);
    }
    
    Status getDescriptor(uint8_t descriptorType, uint8_t descriptorIndex, 
                        std::span<uint8_t> buffer) {
        return controller_.controlTransfer(0x80, 0x06, 
                                         (descriptorType << 8) | descriptorIndex, 
                                         0, buffer);
    }
    
    uint8_t getId() const { return deviceId_; }

private:
    UsbController& controller_;
    uint8_t deviceId_;
};

class UsbManager {
public:
    virtual ~UsbManager() = default;
    virtual std::vector<UsbDevice> enumerateDevices() = 0;
    virtual Status createController(std::unique_ptr<UsbController>& controller) = 0;
};

class ExampleUsbManager : public UsbManager {
private:
    std::unique_ptr<UsbController> controller_;
    
public:
    ExampleUsbManager() {
        createController(controller_);
        controller_->initialize();
    }
    
    std::vector<UsbDevice> enumerateDevices() override {
        std::vector<UsbDevice> devices;
        
        // Try to communicate with device IDs 1-127 to find connected devices
        for (uint8_t deviceId = 1; deviceId <= 127; ++deviceId) {
            UsbDevice candidate(*controller_, deviceId);
            
            // Try to get device descriptor to see if device exists
            std::array<uint8_t, 18> descriptor;
            if (candidate.getDescriptor(0x01, 0, descriptor) == Status::Success) {
                devices.push_back(candidate);
            }
        }
        
        return devices;
    }
    
    Status createController(std::unique_ptr<UsbController>& controller) override {
        // This would create a platform-specific controller implementation
        // controller = std::make_unique<PlatformUsbController>();
        return Status::Success;
    }
};

// Example usage
void demonstrateUsbUsage() {
    ExampleUsbManager manager;
    
    auto devices = manager.enumerateDevices();
    
    for (auto& device : devices) {
        std::array<uint8_t, 18> descriptor;
        if (device.getDescriptor(0x01, 0, descriptor) == Status::Success) {
            uint16_t vendorId = descriptor[8] | (descriptor[9] << 8);
            uint16_t productId = descriptor[10] | (descriptor[11] << 8);
            // Process vendorId and productId...
        }
    }
}
*/

}
// namespace Usb