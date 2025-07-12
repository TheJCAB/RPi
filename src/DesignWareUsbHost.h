#pragma once

#include "DesignWareUsb.h"

#include <stdint.h>
#include <stddef.h>

#include <memory>

class HCDChannel;

class HCDHost
{
public:
    enum ClockRate : unsigned {
        Clock30_60MHz, // 30-60Mhz clock to USB
        Clock48MHz,    // 48Mhz clock to USB
        Clock6MHz,     // 6Mhz clock to USB
    };

    HCDHost(uintptr_t baseAddress, ClockRate, uint8_t numChannels);

    void              DwcClearEnable      ();
    void              DwcResume           ();
    void              DwcPowerOff         ();
    void              DwcConnectionChange ();
    void              DwcEnableChange     ();
    void              DwcOverCurrentChange();
    void              DwcReset            ();
    void              DwcPowerOn          ();
    HubPortFullStatus DwcGetPortStatus    ();

    void HandlePortInterrupt();
    void HandleChannelInterrupt();

    uint32_t GetCurrentFrame();

    struct ReleaseChannel
    {
        void operator()(HCDChannel*) const noexcept;
    };

    using LockedChannel = std::unique_ptr<HCDChannel, ReleaseChannel>;

    /// Finds and reserves an unused DWC USB host channel. This is blocking and
    /// will wait until a channel is available if all in use.
    LockedChannel GetChannel();

private:
    union Registers;

    Registers& registers;

    /// Number of DWC host channels, each of which can be used for an independent
    /// USB transfer.  On the BCM2835 (Raspberry Pi), 8 are available.  This is
    /// documented on page 201 of the BCM2835 ARM Peripherals document.
    static constexpr uint8_t MaxChannels = 8;

    uint8_t m_NumChannels = 0;

    std::unique_ptr<HCDChannel> m_Channels[MaxChannels]{};
};
