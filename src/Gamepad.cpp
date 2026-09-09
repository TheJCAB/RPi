#include "Gamepad.h"

#include "Cpu.h"
#include "Timer.h"
#include "HidUsbDevices.h"

#include "emb-stdio.h"

#include <array>

namespace Gamepad
{

enum class Device : uint8_t
{
    None = 0,

    DragonRise        = 1, // DragonRise Inc. Gamepad
    NintendoSwitchPro = 2, // NintendoSwitchPro Gamepad
};

static uint8_t firstGamepad = 0;
static Device firstGamepadType = Device::None;

static bool    ButtonStates[static_cast<size_t>(Button::Count)]{};
static int16_t AxisStates  [static_cast<size_t>(Axis  ::Count)]{};


// DragonRise Inc.
// idVendor 121 idProduct 294 bcdDevice 1003

namespace DragonRise
{

struct Report {
    // 13 buttons (1 bit each) + 3 bits padding = 2 bytes
    uint16_t buttons : 13;
    uint16_t padding1 : 3;

    // Hat switch (4 bits) + 1 bit padding = 1 byte total
    uint8_t hat : 4;
    uint8_t padding2 : 4;

    // 4 analog axes (X, Y, Z, Rx), 8 bits each
    uint8_t x;
    uint8_t y;
    uint8_t z;
    uint8_t rx;

    // 12 vendor-defined bytes
    uint8_t vendor_data[12];

    // 4 high-resolution axes (16 bits each)
    int16_t axis1;
    int16_t axis2;
    int16_t axis3;
    int16_t axis4;
};

static Cpu::PerformanceTimeDiff RefreshState(UsbDriver& driver)
{
    uint16_t const USB_HID_REPORT_TYPE_INPUT = 1;
    Report report;
    auto const status = Async::WaitOnTask(HIDReadInterruptReport(driver, firstGamepad, 0, reinterpret_cast<std::byte*>(&report), sizeof(report), nullptr));
    //auto const status = HIDReadReport(firstKbd, 0, USB_HID_REPORT_TYPE_INPUT << 8 | 1, &buf[0], 8);
    if (status == RESULT::Ok)
    {
        //printf("HID Gamepad Buttons: %016b Hat: %2x Axes: %02X %02X %02X %02X %6d %6d %6d %6d Vendor: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
        //    report.buttons, report.hat, report.x, report.y, report.z, report.rx,
        //    report.axis1, report.axis2, report.axis3, report.axis4,
        //    report.vendor_data[0], report.vendor_data[1], report.vendor_data[2], report.vendor_data[3],
        //    report.vendor_data[4], report.vendor_data[5], report.vendor_data[6], report.vendor_data[7],
        //    report.vendor_data[8], report.vendor_data[9], report.vendor_data[10], report.vendor_data[11]);

        ButtonStates[static_cast<size_t>(Button::A)] = (report.buttons & (1 << 1)) != 0;
        ButtonStates[static_cast<size_t>(Button::B)] = (report.buttons & (1 << 2)) != 0;
        ButtonStates[static_cast<size_t>(Button::X)] = (report.buttons & (1 << 0)) != 0;
        ButtonStates[static_cast<size_t>(Button::Y)] = (report.buttons & (1 << 3)) != 0;
        ButtonStates[static_cast<size_t>(Button::LeftBumper)]  = (report.buttons & (1 << 4)) != 0;
        ButtonStates[static_cast<size_t>(Button::RightBumper)] = (report.buttons & (1 << 5)) != 0;
        ButtonStates[static_cast<size_t>(Button::Back)] = (report.buttons & (1 << 8)) != 0;
        ButtonStates[static_cast<size_t>(Button::Start)] = (report.buttons & (1 << 9)) != 0;
        ButtonStates[static_cast<size_t>(Button::DPadUp)]    = report.y < 64;
        ButtonStates[static_cast<size_t>(Button::DPadDown)]  = report.y > 192;
        ButtonStates[static_cast<size_t>(Button::DPadRight)] = report.x > 192;
        ButtonStates[static_cast<size_t>(Button::DPadLeft)]  = report.x < 64;
        ButtonStates[static_cast<size_t>(Button::AnyUp)]    = ButtonStates[static_cast<size_t>(Button::DPadUp)];
        ButtonStates[static_cast<size_t>(Button::AnyDown)]  = ButtonStates[static_cast<size_t>(Button::DPadDown)];
        ButtonStates[static_cast<size_t>(Button::AnyRight)] = ButtonStates[static_cast<size_t>(Button::DPadRight)];
        ButtonStates[static_cast<size_t>(Button::AnyLeft)]  = ButtonStates[static_cast<size_t>(Button::DPadLeft)];

        AxisStates[static_cast<size_t>(Axis::LeftX)]  = static_cast<int16_t>(static_cast<int8_t>(report.x - 128) * 256);
        AxisStates[static_cast<size_t>(Axis::LeftY)]  = static_cast<int16_t>(static_cast<int8_t>(report.y - 128) * 256);

        return Cpu::ToTicks(20ms); // Refresh every 20 ms
    }
    else
    {
        printf("HID Gamepad Read Error: %d\n", status);
        return Cpu::ToTicks(1ms); // Try again in 1ms
    }
}

}
// namespace DragonRise

namespace NintendoSwitchPro
{

// idVendor 1406 idProduct 8201 bcdDevice 200

struct __attribute__((packed)) Report
{
    uint8_t report_id; // 0x30

    uint8_t timer;
    uint8_t reserved60; // always 0x60

    uint8_t  buttons;
    uint8_t  buttons2;
    uint8_t  buttons3;
    uint16_t leftX;
    uint8_t  leftY;
    uint16_t rightX;
    uint8_t  rightY;


    // 52 bytes of padding or reserved space
    uint8_t reserved[52];
};

static_assert(sizeof(Report) == 64, "Nintendo Switch Pro Gamepad report size must be 64 bytes");

// Input:

typedef struct __attribute__((packed)) {
    uint8_t report_id;         // 0x21, 0x81
    uint8_t data[63];          // Vendor-defined input
} HIDReport_0x21_t;

// Output:

typedef struct __attribute__((packed)) {
    uint8_t report_id;         // 0x01, 0x10, 0x80, 0x82
    uint8_t data[63];          // Vendor-defined output
} HIDReport_0x01_t;

static Cpu::PerformanceTimeDiff RefreshState(UsbDriver& driver)
{
    uint16_t const USB_HID_REPORT_TYPE_INPUT = 1;
    Report report;
    auto time = Cpu::GetPerformanceCounter();
    auto const status = Async::WaitOnTask(HIDReadInterruptReport(driver, firstGamepad, 0, reinterpret_cast<std::byte*>(&report), sizeof(report), nullptr));
    //printf("Gamepad time: %lld us\n", GetUsForPerformanceTicks(Cpu::GetPerformanceCounter() - time));
    //auto const status = HIDReadReport(firstKbd, 0, USB_HID_REPORT_TYPE_INPUT << 8 | 1, &buf[0], 8);
    if (status == RESULT::Ok)
    {
        if (report.report_id != 0x30)
        {
            printf("HID Gamepad unknown report ID: %02X\n", report.report_id);
            return Cpu::ToTicks(1ms); // Try again in 1ms
        }
        //printf("HID Gamepad Buttons: %08b %08b %08b Axes: %6d %6d %6d %6d Vendor: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
        //    report.buttons, report.buttons2, report.buttons3, report.leftX - 0x7F0, report.leftY - 0x80, report.rightX - 0x7F0, report.rightY - 0x80,
        //    report.reserved[0], report.reserved[1], report.reserved[2], report.reserved[3],
        //    report.reserved[4], report.reserved[5], report.reserved[6], report.reserved[7],
        //    report.reserved[8], report.reserved[9], report.reserved[10], report.reserved[11]);

        ButtonStates[static_cast<size_t>(Button::A)] = (report.buttons & (1 << 2)) != 0;
        ButtonStates[static_cast<size_t>(Button::B)] = (report.buttons & (1 << 3)) != 0;
        ButtonStates[static_cast<size_t>(Button::X)] = (report.buttons & (1 << 0)) != 0;
        ButtonStates[static_cast<size_t>(Button::Y)] = (report.buttons & (1 << 1)) != 0;
        ButtonStates[static_cast<size_t>(Button::LeftBumper)]  = (report.buttons3 & (1 << 6)) != 0;
        ButtonStates[static_cast<size_t>(Button::RightBumper)] = (report.buttons & (1 << 6)) != 0;
        ButtonStates[static_cast<size_t>(Button::LeftTrigger)]  = (report.buttons3 & (1 << 7)) != 0;
        ButtonStates[static_cast<size_t>(Button::RightTrigger)] = (report.buttons & (1 << 7)) != 0;
        ButtonStates[static_cast<size_t>(Button::Back)] = (report.buttons2 & (1 << 0)) != 0;
        ButtonStates[static_cast<size_t>(Button::Start)] = (report.buttons2 & (1 << 1)) != 0;
        ButtonStates[static_cast<size_t>(Button::Center)] = (report.buttons2 & (1 << 4)) != 0;
        ButtonStates[static_cast<size_t>(Button::LeftStick)]  = (report.buttons2 & (1 << 2)) != 0;
        ButtonStates[static_cast<size_t>(Button::RightStick)] = (report.buttons2 & (1 << 3)) != 0;
        ButtonStates[static_cast<size_t>(Button::DPadUp)]    = (report.buttons3 & (1 << 1)) != 0;
        ButtonStates[static_cast<size_t>(Button::DPadDown)]  = (report.buttons3 & (1 << 0)) != 0;
        ButtonStates[static_cast<size_t>(Button::DPadRight)] = (report.buttons3 & (1 << 2)) != 0;
        ButtonStates[static_cast<size_t>(Button::DPadLeft)]  = (report.buttons3 & (1 << 3)) != 0;
        ButtonStates[static_cast<size_t>(Button::AnyUp)]    = ButtonStates[static_cast<size_t>(Button::DPadUp)]    || (report.leftY > 192);
        ButtonStates[static_cast<size_t>(Button::AnyDown)]  = ButtonStates[static_cast<size_t>(Button::DPadDown)]  || (report.leftY < 64);
        ButtonStates[static_cast<size_t>(Button::AnyRight)] = ButtonStates[static_cast<size_t>(Button::DPadRight)] || (report.leftX > 0xC00);
        ButtonStates[static_cast<size_t>(Button::AnyLeft)]  = ButtonStates[static_cast<size_t>(Button::DPadLeft)]  || (report.leftX < 0x400);

        AxisStates[static_cast<size_t>(Axis::LeftX)]  = static_cast<int16_t>(static_cast<int16_t>(report.leftX  - 0x7F0) * 16);
        AxisStates[static_cast<size_t>(Axis::LeftY)]  = static_cast<int16_t>(static_cast<int8_t >(report.leftY  - 128) * 256);
        AxisStates[static_cast<size_t>(Axis::RightX)] = static_cast<int16_t>(static_cast<int16_t>(report.rightX - 0x7F0) * 16);
        AxisStates[static_cast<size_t>(Axis::RightY)] = static_cast<int16_t>(static_cast<int8_t >(report.rightY - 128) * 256);

        return Cpu::ToTicks(20ms); // Refresh every 20 ms
    }
    else
    {
        //printf("HID Gamepad Read Error: %d\n", status);
        return Cpu::ToTicks(1ms); // Try again in 1ms
    }
}

}
// namespace NintendoSwitchPro

void Init(UsbDriver& driver)
{
    DeviceDescriptor descriptor;

    // Detect the first supported gamepad on USB bus.
    for (auto& device : driver.EnumerateDevices())
    {
        if (!device.IsHid())
        {
            continue;
        }
        auto i = device.GetAddress();
        descriptor = device.GetDescriptor();
        if (descriptor.bDeviceProtocol != 0) // Generic HID protocol (not a mouse or keyboard)
        {
            continue;
        }
        if (descriptor.idVendor == 121 && descriptor.idProduct == 294) // DragonRise Inc.
        {
            firstGamepad = i;
            firstGamepadType = Device::DragonRise;
            break;
        }
        else if (descriptor.idVendor == 1406 && descriptor.idProduct == 8201) // Nintendo Switch Pro
        {
            firstGamepad = i;
            firstGamepadType = Device::NintendoSwitchPro;

            std::byte buf[2] = { std::byte{0x80}, std::byte{0x04} }; // Request to stay on USB instead of reverting to Bluetooth
            auto const status = Async::WaitOnTask(HIDReadReport(driver, firstGamepad, 0, USB_HID_REPORT_TYPE_FEATURE << 8 | 0x80, &buf[0], 8));
            if (status != RESULT::Ok)
            {
                printf("HID Gamepad Feature Report Error: %d\n", status);
            }
            else
            {
                printf("HID Gamepad Feature Report sent successfully.\n");
            }

            break;
        }
    }
    if (firstGamepad > 0)
    {
        auto& device = *driver.UsbDeviceAtAddress(firstGamepad);
        printf("Gamepad detected\r\n");
        printf("Vendor ID: %04X, Product ID: %04X\r\n", descriptor.idVendor, descriptor.idProduct);
        char buffer[256];
        if (size_t length = device.GetDeviceProductString(buffer))
        {
            printf("Product: %s\r\n", buffer);
        }
        if (size_t length = device.GetDeviceManufacturerString(buffer))
        {
            printf("Manufacturer: %s\r\n", buffer);
        }
        if (size_t length = device.GetDeviceSerialNumberString(buffer))
        {
            printf("Serial Number: %s\r\n", buffer);
        }
        if (size_t length = device.GetDeviceConfigStringString(buffer))
        {
            printf("Configuration: %s\r\n", buffer);
        }
        Async::WaitOnTask(HIDEnableInterruptINSimple(driver, firstGamepad, 0));
        printf("Gamepad configured\r\n");
    }
}

static void RefreshStateIfNeeded(UsbDriver& driver)
{
    if (firstGamepad == 0)
    {
        return;
    }

    auto const time = Cpu::GetPerformanceCounter();
    static Cpu::PerformanceTime nextRefresh{0};
    if (time >= nextRefresh)
    {
        switch (firstGamepadType)
        {
            case Device::NintendoSwitchPro: { auto nextRefreshDelay = NintendoSwitchPro::RefreshState(driver); nextRefresh = Cpu::GetPerformanceCounter() + nextRefreshDelay; break; }
            case Device::DragonRise       : { auto nextRefreshDelay = DragonRise       ::RefreshState(driver); nextRefresh = Cpu::GetPerformanceCounter() + nextRefreshDelay; break; }
            default:
                printf("Gamepad: Unknown device type %d\n", static_cast<int>(firstGamepadType));
                nextRefresh = Cpu::PerformanceTime{0xFFFF'FFFF'FFFF'FFFF}; // Don't try again
                return;
        }
    }
}

bool IsButtonPressed(UsbDriver& driver, Button button)
{
    RefreshStateIfNeeded(driver);
    return ButtonStates[static_cast<size_t>(button)];
}

int16_t GetAxisState(UsbDriver& driver, Axis axis)
{
    RefreshStateIfNeeded(driver);
    return AxisStates[static_cast<size_t>(axis)];
}

}
// namespace Gamepad
