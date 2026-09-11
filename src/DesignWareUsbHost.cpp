// DesignWare2 hardware driver for Raspberry Pi 3.
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

#include "DesignWareUsbHost.h"

#include "DesignWareUsbChannel.h"

#include <BootLib/RegisterProxy.h>

#include "Cpu.h"
#include "Mmio.h"
#include "Mmu.h"
#include "Mailbox.h"
#include "Timer.h"
#include "Processor.h"
#include "Interrupts.h"

#include "emb-stdio.h"				// Needed for printf

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include <concepts>
#include <atomic>
#include <print>

#define LOG(...)
//#define LOG(...) printf(__VA_ARGS__)
#define LOG_DEBUG(...)
//#define LOG_DEBUG(...) printf(__VA_ARGS__)



#define ReceiveFifoSize 20480 /* 16 to 32768 */
#define NonPeriodicFifoSize 20480 /* 16 to 32768 */
#define PeriodicFifoSize 20480 /* 16 to 32768 */

/***************************************************************************}
{						 PRIVATE INTERNAL ENUMERATIONS					    }
****************************************************************************/

/*--------------------------------------------------------------------------}
{	       FLUSH TYPE ENUMERATION FOR FIFO ON THE DESIGNWARE 2.0		    }
{--------------------------------------------------------------------------*/
enum CoreFifoFlush
{
    FlushNonPeriodic = 0,
    FlushPeriodic1 = 1,
    FlushPeriodic2 = 2,
    FlushPeriodic3 = 3,
    FlushPeriodic4 = 4,
    FlushPeriodic5 = 5,
    FlushPeriodic6 = 6,
    FlushPeriodic7 = 7,
    FlushPeriodic8 = 8,
    FlushPeriodic9 = 9,
    FlushPeriodic10 = 10,
    FlushPeriodic11 = 11,
    FlushPeriodic12 = 12,
    FlushPeriodic13 = 13,
    FlushPeriodic14 = 14,
    FlushPeriodic15 = 15,
    FlushAll = 16,
};

/***************************************************************************}
{         PRIVATE INTERNAL DESIGNWARE 2.0 CORE REGISTER STRUCTURES          }
****************************************************************************/

/***************************************************************************}
{         PRIVATE INTERNAL DESIGNWARE 2.0 HOST REGISTER STRUCTURES          }
****************************************************************************/


/*--------------------------------------------------------------------------}
{                          USB HOST CONFIG STRUCTURE					    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) HostConfig {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile HCDHost::ClockRate ClockRate : 2;			    // @0
            volatile bool FslsOnly : 1;								// @2
            volatile unsigned _reserved3_6 : 4;						// @3
            volatile unsigned en_32khz_susp : 1;					// @7
            volatile unsigned res_val_period : 8;					// @8
            volatile unsigned _reserved16_22 : 7;					// @16
            volatile bool EnableDmaDescriptor : 1;					// @23
            volatile unsigned FrameListEntries : 2;					// @24
            volatile bool PeriodicScheduleEnable : 1;				// @26
            volatile bool PeriodicScheduleStatus : 1;				// @27
            volatile unsigned reserved28_30 : 3;					// @28
            volatile bool mode_chg_time : 1;						// @31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{                       USB HOST FRAME INTERVAL STRUCTURE				    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) HostFrameInterval {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile unsigned Interval : 16;						// @0
            volatile bool DynamicFrameReload : 1;					// @16
            volatile unsigned _reserved17_31 : 15;					// @17-31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{                       USB HOST FRAME CONTROL STRUCTURE				    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) HostFrameControl {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile unsigned FrameNumber : 16;						// @0
            volatile unsigned FrameRemaining : 16;					// @16
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{                         USB FIFO STATUS STRUCTURE						    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) HostFifoStatus {
    enum TokenTypeT {
        ZeroLength = 0,
        Ping = 1,
        Disable = 2,
    };
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile unsigned SpaceAvailable : 16;					// @0
            volatile unsigned QueueSpaceAvailable : 8;				// @16
            volatile unsigned Terminate : 1;						// @24
            volatile TokenTypeT TokenType : 2;						// @25
            volatile unsigned Channel : 4;							// @27
            volatile unsigned Odd : 1;								// @31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{                         USB HOST PORT STRUCTURE						    }
{--------------------------------------------------------------------------*/
/* Due to the inconsistent design of the bits in this register, sometime it requires  zeroing 
   bits in the register before the write, so you do not unintentionally write 1's to them. */
#define HOSTPORTMASK  ~0x2E								// These are the funky bits on this register and we "NOT" them to make "AND" mask
struct __attribute__((__packed__, aligned(4))) HostPort {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile bool Connect : 1;								// @0
            volatile bool ConnectChanged : 1;						// @1 funky bit
            volatile bool Enable : 1;								// @2 funky bit
            volatile bool EnableChanged : 1;						// @3 funky bit
            volatile bool OverCurrent : 1;							// @4
            volatile bool OverCurrentChanged : 1;					// @5 funky bit
            volatile bool Resume : 1;								// @6
            volatile bool Suspend : 1;								// @7
            volatile bool Reset : 1;								// @8
            volatile unsigned _reserved9 : 1;						// @9
            volatile unsigned PortLineStatus : 2;					// @10
            volatile bool Power : 1;								// @12
            volatile unsigned TestControl : 4;						// @13
            volatile UsbSpeed Speed : 2;							// @17
            volatile unsigned _reserved19_31 : 13;					// @19-31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{					DWC POWER AND CLOCK REGISTER STRUCTURE				    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) PowerReg {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile bool StopPClock : 1;							// @0
            volatile bool GateHClock : 1;							// @1
            volatile bool PowerClamp : 1;							// @2
            volatile bool PowerDownModules : 1;						// @3
            volatile bool PhySuspended : 1;							// @4
            volatile bool EnableSleepClockGating : 1;				// @5
            volatile bool PhySleeping : 1;							// @6
            volatile bool DeepSleep : 1;							// @7
            volatile unsigned _reserved8_31 : 24;					// @8-31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};


/***************************************************************************}
{    PRIVATE POINTERS TO ALL OUR DESIGNWARE 2.0 HOST REGISTER STRUCTURES    }
****************************************************************************/

#define USB_CORE_OFFSET  0x98'0000	// USB CORE OFFSET FROM PERIPHERAL IO BASE ADDRESS

/*--------------------------------------------------------------------------}
{					DWC USB HOST REGISTER POINTERS						    }
{--------------------------------------------------------------------------*/

constexpr uintptr_t DWC_HOST_OFFSET = USB_CORE_OFFSET + 0x400;

union HCDHost::Registers
{
    BootLib::Register<HostConfig       , 0x00> CONFIG       ; // @0x400
    BootLib::Register<HostFrameInterval, 0x04> FRAMEINTERVAL; // @0x404
    BootLib::Register<HostFrameControl , 0x08> FRAMECONTROL ; // @0x408
    BootLib::Register<HostFifoStatus   , 0x10> FIFOSTATUS   ; // @0x410
    BootLib::Register<uint32_t         , 0x14> INTERRUPT    ; // @0x414
    BootLib::Register<uint32_t         , 0x18> INTERRUPTMASK; // @0x418
    BootLib::Register<uint32_t         , 0x1C> FRAMELIST    ; // @0x41C
    BootLib::Register<HostPort         , 0x40> PORT         ; // @0x440
};


/*==========================================================================}
{			    INTERNAL FAKE ROOT HUB MESSAGE HANDLER FUNCTIONS		    }
{==========================================================================*/

/*==========================================================================}
{					   INTERNAL HOST CONTROL FUNCTIONS					    }
{==========================================================================*/

void HCDHost::HandlePortInterrupt()
{
    HostPort port = registers.PORT;
    std::println("registers.PORT: {:#b}", port.Raw32);

    if (port.ConnectChanged    ) { std::println("ConnectChanged    : {}", static_cast<int>(port.Connect    )); }
    if (port.EnableChanged     ) { std::println("EnableChanged     : {}", static_cast<int>(port.Enable     )); }
    if (port.OverCurrentChanged) { std::println("OverCurrentChanged: {}", static_cast<int>(port.OverCurrent)); }

    port.Enable = false;

    registers.PORT = port;
}

void HCDHost::HandleChannelInterrupt()
{
    uint32_t interrupt = registers.INTERRUPT;

//    Uart::Puts("Host: ");
//    Uart::PutHex(this);
//    Uart::Puts("\n");
//
//    Uart::Puts("Interrupt: ");
//    Uart::PutBin(interrupt);
//    Uart::Puts("\n");
//
//    Uart::Puts("Interrupt mask: ");
//    Uart::PutBin(registers.INTERRUPTMASK.get());
//    Uart::Puts("\n");

    uint32_t channelsMask = interrupt;
    while (channelsMask)
    {
        uint32_t channel = std::countr_zero(channelsMask);
        channelsMask &= ~(1u << channel);

        if (channel >= m_NumChannels)
        {
            Cpu::Panic("HCD: Channel %u interrupt received, but only %u channels are available.\n", channel, m_NumChannels);
        }

        //printf("HCD: Channel %u (%p) interrupt received.\n", channel, m_Channels[channel].get());
        //Uart::Puts("!");

        m_Channels[channel]->HandleInterrupt();
    }

    registers.INTERRUPT = interrupt;
}

uint32_t HCDHost::GetCurrentFrame()
{
    return (*registers.FRAMECONTROL).FrameNumber;
}

/// Initializes the host controller hardware.
Async::task<std::shared_ptr<HCDHost>> HCDHost::Make(uintptr_t baseAddress, ClockRate clock, uint8_t numChannels)
{
    std::shared_ptr<HCDHost> host{ new HCDHost(*reinterpret_cast<Registers*>(baseAddress)) };

    host->registers.CONFIG = [clock](auto& r) {
        // ULPI FsLs Host mode, I assume other mode is ULPI only  .. documentation would be nice
        r.FslsOnly = true;
        r.ClockRate = clock;
    };

    HostPort tempPort = *host->registers.PORT;
    LOG_DEBUG("HCD: Initial host port: 0x%08X\n", tempPort.Raw32);
    if (!tempPort.Power) {
        LOG_DEBUG("HCD: Initial power physical host up.\n");
        tempPort.Raw32 &= HOSTPORTMASK;
        tempPort.Power = true;
        host->registers.PORT = tempPort;
    }

    /*co_await Async*/ Cpu::Delay(1'000us);

    LOG_DEBUG("HCD: Initial resetting physical host.\n");
    tempPort = *host->registers.PORT;
    LOG_DEBUG("HCD: Powered host port: 0x%08X\n", tempPort.Raw32);
    
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Reset = true;
    host->registers.PORT = tempPort;
    co_await Async::Delay(60ms);
    tempPort = *host->registers.PORT;
    LOG_DEBUG("HCD: Reset host port: 0x%08X\n", tempPort.Raw32);

    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Reset = false;
    host->registers.PORT = tempPort;

    host->registers.INTERRUPT = 0xFFFFFFFF;
    host->registers.INTERRUPTMASK = 0;

    LOG_DEBUG("HCD: Host %p successfully started.\n", host.get());

    host->m_NumChannels = numChannels > MaxChannels ? MaxChannels : numChannels;

    auto const dmaBuffer = static_cast<std::byte*>(Mmu::AllocateGpuMemory((host->m_NumChannels * HCDChannel::MaxPacketSize + Mmu::PageSize + 1) / Mmu::PageSize));

    for (uint8_t channel = 0; channel < host->m_NumChannels; ++channel)
    {
        LOG_DEBUG("HCD: Initializing channel %u at %p\n", channel, reinterpret_cast<void*>(baseAddress + 0x100u + 0x20u * channel));
        std::span<std::byte, HCDChannel::MaxPacketSize> channelDmaBuffer{ dmaBuffer + channel * HCDChannel::MaxPacketSize, HCDChannel::MaxPacketSize };
        host->m_Channels[channel] = std::make_unique<HCDChannel>(*host, baseAddress + 0x100u + 0x20u * channel, channel, channelDmaBuffer);
        host->m_freeChannels[channel] = host->m_Channels[channel].get();
    }

    LOG_DEBUG("HCD: Initialized %u channels\n", m_NumChannels);

    /*co_await Async*/ Cpu::Delay(1ms);								// Wait 1 millisecond to allow the USB bus to stabilize

    tempPort = *host->registers.PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Power = true;
    host->registers.PORT = tempPort;

    LOG("HCD: Root HUB powered on.\n");
    /*co_await Async*/ Cpu::Delay(2ms);

    tempPort = *host->registers.PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Suspend = false;
    tempPort.Reset = true;
    tempPort.Power = true;
    host->registers.PORT = tempPort;

    co_await Async::Delay(60ms);

    tempPort = *host->registers.PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Reset = false;
    host->registers.PORT = tempPort;

    /*co_await Async*/ Cpu::Delay(1ms);

    co_return host;
}

HCDHost::HCDHost(Registers& r) : registers(r) {}

HCDHost::~HCDHost() = default;


HCDHost::LockedChannel HCDHost::GetChannel()
{
    for (;;)
    {
        for (uint8_t i = 0; i < m_NumChannels; ++i)
        {
            auto& channelSlot = m_freeChannels[i];

            auto currentChannel = channelSlot.load(std::memory_order_relaxed);
            if (currentChannel != nullptr && channelSlot.compare_exchange_strong(currentChannel, nullptr, std::memory_order_acquire))
            {
                registers.INTERRUPTMASK |= 1u << currentChannel->GetNumber();
                return LockedChannel{ currentChannel };
            }
        }
        Cpu::Delay(10ms);
    }
}

void HCDHost::ReleaseChannel::operator()(HCDChannel* channel) const noexcept
{
    if (channel == nullptr)
    {
        return;
    }

    //LOG_DEBUG("HCD: Releasing channel %u\n", channel->GetNumber());

    channel->GetHost().registers.INTERRUPTMASK &= ~(1u << channel->GetNumber());

    auto& channelSlot = channel->GetHost().m_freeChannels[channel->GetNumber()];

    auto const nullChannel{ channelSlot.exchange(channel, std::memory_order_release) };
    if (nullChannel != nullptr)
    {
        LOG("HCD: Attempted to release an already released channel %u\n", channel->GetNumber());
        return;
    }
}
