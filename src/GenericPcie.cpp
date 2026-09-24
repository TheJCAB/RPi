
#include "PCIe.h"

#include "Cpu.h"
#include "Mmio.h"
#include "Uart.h"
#include "Timer.h"
#include "Mailbox.h"
#include "emb-stdio.h"

#include <fmt/format.h>

#include <stdint.h>
#include <type_traits>
#include <utility>

namespace PCIe
{

struct GenericDriver : public Driver
{
    GenericDriver(PhysicalAddress mmioBase, PhysicalAddress memBase, uintptr_t pciBaseAddress, size_t memSize);

    bool Initialize() override;

    Configuration ConfigureDevice(DeviceAddress addr) noexcept override;

    union Registers;

    std::atomic<PCIeError> initError_ = PCIeError::DRIVER_NOT_INITIALIZED;

    CommonConfigHeader&          rootHeader;                  // The root device's configuration header

    std::shared_mutex       driver_mutex_;
};

GenericDriver::GenericDriver(PhysicalAddress mmioBase, PhysicalAddress memBase, uintptr_t pciBaseAddress, size_t memSize)
    : Driver(pciBaseAddress, memBase, memSize)
    , rootHeader{ *reinterpret_cast<CommonConfigHeader*>(mmioBase) }
{
    fmt::println("PCIe configuration registers base is 0x{:X}...", reinterpret_cast<std::uint64_t>(&rootHeader));

    initError_.store(PCIeError::SUCCESS);
}

bool GenericDriver::Initialize()
{
    // Late initialization logic here

    if (initError_.load() != PCIeError::SUCCESS)
    {
        return false;
    }

    return Driver::Initialize();
}

Configuration GenericDriver::ConfigureDevice(DeviceAddress addr) noexcept
{
    uintptr_t offset = addr.Bus << 20 | addr.Device << 15 | addr.Function << 12; // ECAM
    return Configuration(addr, *reinterpret_cast<CommonConfigHeader*>(reinterpret_cast<uintptr_t>(&rootHeader) + offset));
}

std::shared_ptr<Driver> CreateGenericDriver(PhysicalAddress mmioBase, PhysicalAddress memBase, uintptr_t pciBaseAddress, size_t memSize)
{
    auto driver = std::make_shared<GenericDriver>(mmioBase, memBase, pciBaseAddress, memSize);
    if (!driver->Initialize())
    {
        return {};
    }
    return driver;
}

}
// namespace PCIe
