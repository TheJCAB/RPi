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
#include "DesignWareUsb.h"

#include "Async.h"

#include <stdlib.h>                // C standard needed for NULL
#include <stdint.h>                // C standard needed for uint8_t, uint32_t, uint64_t etc
#include <string.h>                // C standard needed for memset
#include <wchar.h>                // C standard needed for UTF for unicode descriptor support

#include "DesignWareUsbHost.h"
#include "DesignWareUsbChannel.h"

#include <BootLib/RegisterProxy.h>

#include "Cpu.h"
#include "Mmio.h"
#include "Mailbox.h"
#include "Timer.h"
#include "Processor.h"
#include "Interrupts.h"

#include "emb-stdio.h"                // Needed for printf

#include <concepts>
#include <fmt/format.h>

//#define LOG(...)
#define LOG(...) fmt::print(__VA_ARGS__)
#define LOG_DEBUG(...)
//#define LOG_DEBUG(...) printf(__VA_ARGS__)

// Explicitly packing bitfields of different types keeps VSCode's IntelliSense happier.
#define PACKED __attribute__((__packed__))

#define ReceiveFifoSize 20480 /* 16 to 32768 */
#define NonPeriodicFifoSize 20480 /* 16 to 32768 */
#define PeriodicFifoSize 20480 /* 16 to 32768 */

std::shared_ptr<HCDHost> Host;

/***************************************************************************}
{                         PRIVATE INTERNAL ENUMERATIONS                        }
****************************************************************************/

/*--------------------------------------------------------------------------}
{           FLUSH TYPE ENUMERATION FOR FIFO ON THE DESIGNWARE 2.0            }
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

/*--------------------------------------------------------------------------}
{       FIFOSIZE STRUCTURE .. THERE ARE A FEW OF THESE ON DESIGNWARE 2.0     }
{--------------------------------------------------------------------------*/
union FifoSize
{
    struct PACKED
    {
        unsigned StartAddress : 16; //  @0
        unsigned Depth        : 16; // @16
    };
    uint32_t Raw32;
};

/*--------------------------------------------------------------------------}
{                       USB CORE OTG CONTROL STRUCTURE                        }
{--------------------------------------------------------------------------*/
union CoreOtgControl
{
    struct PACKED
    {
        bool     sesreqscs        : 1; //  @0
        bool     sesreq           : 1; //  @1
        bool     vbvalidoven      : 1; //  @2
        bool     vbvalidovval     : 1; //  @3
        bool     avalidoven       : 1; //  @4
        bool     avalidovval      : 1; //  @5
        bool     bvalidoven       : 1; //  @6
        bool     bvalidovval      : 1; //  @7
        bool     hstnegscs        : 1; //  @8
        bool     hnpreq           : 1; //  @9
        bool     HostSetHnpEnable : 1; // @10
        bool     devhnpen         : 1; // @11
        unsigned _reserved12_15   : 4; // @12-15
        bool     conidsts         : 1; // @16
        unsigned dbnctime         : 1; // @17
        bool     ASessionValid    : 1; // @18
        bool     BSessionValid    : 1; // @19
        unsigned OtgVersion       : 1; // @20
        unsigned _reserved21      : 1; // @21
        unsigned multvalidbc      : 5; // @22-26
        bool     chirpen          : 1; // @27
        unsigned _reserved28_31   : 4; // @28-31
    };
    uint32_t Raw32;
};

static_assert(sizeof(CoreOtgControl) == 4, "CoreOtgControl must be 4 bytes");


/*--------------------------------------------------------------------------}
{                     USB CORE OTG INTERRUPT STRUCTURE                        }
{--------------------------------------------------------------------------*/
union CoreOtgInterrupt
{
    struct PACKED
    {
        unsigned _reserved0_1                       :  2; //  @0
        bool     SessionEndDetected                 :  1; //  @2
        unsigned _reserved3_7                       :  5; //  @3
        bool     SessionRequestSuccessStatusChange  :  1; //  @8
        bool     HostNegotiationSuccessStatusChange :  1; //  @9
        unsigned _reserved10_16                     :  7; // @10
        bool     HostNegotiationDetected            :  1; // @17
        bool     ADeviceTimeoutChange               :  1; // @18
        bool     DebounceDone                       :  1; // @19
        unsigned _reserved20_31                     : 12; // @20-31
    };
    uint32_t Raw32;
};

enum AxiBurstLength {
    Length4 = 0,
    Length3 = 1,
    Length2 = 2,
    Length1 = 3,
};

enum EmptyLevel {
    Empty = 1,
    Half = 0,
};

enum DmaRemainderMode {
    Incremental = 0,
    Single = 1, // (default)
};

union CoreAhb
{
    struct PACKED
    {
        bool             InterruptEnable            :  1; //  @0
        AxiBurstLength   AxiBurstLength             :  2; //  @1
        unsigned         _reserved3                 :  1; //  @3
        bool             WaitForAxiWrites           :  1; //  @4
        bool             DmaEnable                  :  1; //  @5
        unsigned         _reserved6                 :  1; //  @6
        EmptyLevel       TransferEmptyLevel         :  1; //  @7
        EmptyLevel       PeriodicTransferEmptyLevel :  1; //  @8
        unsigned         _reserved9_20              : 12; //  @9
        bool             remmemsupp                 :  1; // @21
        bool             notialldmawrit             :  1; // @22
        DmaRemainderMode DmaRemainderMode           :  1; // @23
        unsigned         _reserved24_31             :  8; // @24-31
    };
    uint32_t Raw32;                                    // Union to access all 32 bits as a uint32_t
};

static_assert(sizeof(CoreAhb) == 4, "CoreAhb must be 4 bytes");


enum UMode {
    ULPI,
    UTMI,
};

union UsbControl
{
    struct __attribute__((__packed__))
    {
        unsigned toutcal                 : 3; //  @0
        bool     PhyInterface            : 1; //  @3
        UMode    ModeSelect              : 1; //  @4
        bool     fsintf                  : 1; //  @5
        bool     physel                  : 1; //  @6
        bool     ddrsel                  : 1; //  @7
        bool     SrpCapable              : 1; //  @8
        bool     HnpCapable              : 1; //  @9
        unsigned usbtrdtim               : 4; // @10
        unsigned reserved1               : 1; // @14
        bool     phy_lpm_clk_sel         : 1; // @15
        bool     otgutmifssel            : 1; // @16
        bool     UlpiFsls                : 1; // @17
        bool     ulpi_auto_res           : 1; // @18
        bool     ulpi_clk_sus_m          : 1; // @19
        bool     UlpiDriveExternalVbus   : 1; // @20
        bool     ulpi_int_vbus_indicator : 1; // @21
        bool     TsDlinePulseEnable      : 1; // @22
        bool     indicator_complement    : 1; // @23
        bool     indicator_pass_through  : 1; // @24
        bool     ulpi_int_prot_dis       : 1; // @25
        bool     ic_usb_capable          : 1; // @26
        bool     ic_traffic_pull_remove  : 1; // @27
        bool     tx_end_delay            : 1; // @28
        bool     force_host_mode         : 1; // @29
        bool     force_dev_mode          : 1; // @30
        unsigned _reserved31             : 1; // @31
    };
    uint32_t Raw32;                                    // Union to access all 32 bits as a uint32_t
};

static_assert(sizeof(UsbControl) == 4, "UsbControl must be 4 bytes");


/*--------------------------------------------------------------------------}
{                             USB CORE RESET STRUCTURE                        }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) CoreReset {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile bool CoreSoft : 1;                                // @0
            volatile bool HclkSoft : 1;                                // @1
            volatile bool HostFrameCounter : 1;                        // @2
            volatile bool InTokenQueueFlush : 1;                    // @3
            volatile bool ReceiveFifoFlush : 1;                        // @4
            volatile bool TransmitFifoFlush : 1;                    // @5
            volatile unsigned TransmitFifoFlushNumber : 5;            // @6
            volatile unsigned _reserved11_29 : 19;                    // @11
            volatile bool DmaRequestSignal : 1;                        // @30
            volatile bool AhbMasterIdle : 1;                        // @31
        };
        volatile uint32_t Raw32;                                    // Union to access all 32 bits as a uint32_t
    };
};

static_assert(sizeof(CoreReset) == 4, "CoreReset must be 4 bytes");


/*--------------------------------------------------------------------------}
{           INTERRUPT BITS ON THE USB CORE OF THE DESIGNWARE 2.0                }
{--------------------------------------------------------------------------*/
union CoreInterrupts
{
    struct PACKED
    {
        bool CurrentMode                  : 1; //  @0
        bool ModeMismatch                 : 1; //  @1
        bool Otg                          : 1; //  @2
        bool DmaStartOfFrame              : 1; //  @3
        bool ReceiveStatusLevel           : 1; //  @4
        bool NpTransmitFifoEmpty          : 1; //  @5
        bool ginnakeff                    : 1; //  @6
        bool goutnakeff                   : 1; //  @7
        bool ulpick                       : 1; //  @8
        bool I2c                          : 1; //  @9
        bool EarlySuspend                 : 1; // @10
        bool UsbSuspend                   : 1; // @11
        bool UsbReset                     : 1; // @12
        bool EnumerationDone              : 1; // @13
        bool IsochronousOutDrop           : 1; // @14
        bool eopframe                     : 1; // @15
        bool RestoreDone                  : 1; // @16
        bool EndPointMismatch             : 1; // @17
        bool InEndPoint                   : 1; // @18
        bool OutEndPoint                  : 1; // @19
        bool IncompleteIsochronousIn      : 1; // @20
        bool IncompleteIsochronousOut     : 1; // @21
        bool fetsetup                     : 1; // @22
        bool ResetDetect                  : 1; // @23
        bool Port                         : 1; // @24
        bool HostChannel                  : 1; // @25
        bool HpTransmitFifoEmpty          : 1; // @26
        bool LowPowerModeTransmitReceived : 1; // @27
        bool ConnectionIdStatusChange     : 1; // @28
        bool Disconnect                   : 1; // @29
        bool SessionRequest               : 1; // @30
        bool Wakeup                       : 1; // @31
    };
    volatile uint32_t Raw32;
};

static_assert(sizeof(CoreInterrupts) == 4, "CoreInterrupts must be 4 bytes");


/*--------------------------------------------------------------------------}
{                 USB CORE NON PERIODIC FIFO STATUS STRUCTURE                }
{--------------------------------------------------------------------------*/
enum TokenTypeT {
    InOut = 0,
    ZeroLengthOut = 1,
    PingCompleteSplit = 2,
    ChannelHalt = 3,
};
union NonPeriodicFifoStatus
{
    struct PACKED
    {
        unsigned   SpaceAvailable      : 16; //  @0
        unsigned   QueueSpaceAvailable :  8; // @16
        unsigned   Terminate           :  1; // @24
        TokenTypeT TokenType           :  2; // @25
        unsigned   Channel             :  4; // @27
        unsigned   Odd                 :  1; // @31
    };
    uint32_t Raw32;
};

union DwcVendorId
{
    struct {
        uint16_t Revision : 12;
        uint16_t Version  :  4;
        char     T;
        char     O;
    };
    uint32_t Raw32;
};

union DwcUserId
{
    struct {
        uint32_t Zeros : 12;
        uint32_t Id    : 20;
    };
    uint32_t Raw32;
};

static_assert(sizeof(DwcVendorId) == 4, "DwcVendorId must be 4 bytes");
static_assert(sizeof(DwcUserId) == 4, "DwcUserId must be 4 bytes");

/*--------------------------------------------------------------------------}
{                         USB CORE HARDWARE STRUCTURE                        }
{--------------------------------------------------------------------------*/
enum OperatingModeT {
    HNP_SRP_CAPABLE,
    SRP_ONLY_CAPABLE,
    NO_HNP_SRP_CAPABLE,
    SRP_CAPABLE_DEVICE,
    NO_SRP_CAPABLE_DEVICE,
    SRP_CAPABLE_HOST,
    NO_SRP_CAPABLE_HOST,
};
enum ArchitectureT {
    SlaveOnly,
    ExternalDma,
    InternalDma,
};
enum HighSpeedPhysicalT {
    NotSupported,
    Utmi,
    Ulpi,
    UtmiUlpi,
};
enum FullSpeedPhysicalT {
    Physical0,
    Dedicated,
    Physical2,
    Physcial3,
};
enum UtmiPhysicalDataWidthT {
    Width8bit,
    Width16bit,
    Width8or16bit,
};
struct __attribute__((__packed__, aligned(4))) CoreHardware0 {
    struct __attribute__((__packed__, aligned(1))) {
        volatile const unsigned Direction0 : 2;                        // @0
        volatile const unsigned Direction1 : 2;                        // @2
        volatile const unsigned Direction2 : 2;                        // @4
        volatile const unsigned Direction3 : 2;                        // @6
        volatile const unsigned Direction4 : 2;                        // @8
        volatile const unsigned Direction5 : 2;                        // @10
        volatile const unsigned Direction6 : 2;                        // @12
        volatile const unsigned Direction7 : 2;                        // @14
        volatile const unsigned Direction8 : 2;                        // @16
        volatile const unsigned Direction9 : 2;                        // @18
        volatile const unsigned Direction10 : 2;                    // @20
        volatile const unsigned Direction11 : 2;                    // @22
        volatile const unsigned Direction12 : 2;                    // @24
        volatile const unsigned Direction13 : 2;                    // @26
        volatile const unsigned Direction14 : 2;                    // @28
        volatile const unsigned Direction15 : 2;                    // @30
    };
};

struct __attribute__((__packed__, aligned(4))) CoreHardware1 {
    struct __attribute__((__packed__, aligned(1))) {
        volatile const OperatingModeT OperatingMode : 3;            // @32-34
        volatile const ArchitectureT Architecture : 2;                // @35
        volatile bool PointToPoint : 1;                                // @37
        volatile const HighSpeedPhysicalT HighSpeedPhysical : 2;    // @38-39
        volatile const FullSpeedPhysicalT FullSpeedPhysical : 2;    // @40-41
        volatile const unsigned DeviceEndPointCount : 4;            // @42
        volatile const unsigned HostChannelMax : 4;                // @46
        volatile const bool SupportsPeriodicEndpoints : 1;            // @50
        volatile const bool DynamicFifo : 1;                        // @51
        volatile const bool multi_proc_int : 1;                        // @52
        volatile const unsigned _reserver21 : 1;                    // @53
        volatile const unsigned NonPeriodicQueueDepth : 2;            // @54
        volatile const unsigned HostPeriodicQueueDepth : 2;            // @56
        volatile const unsigned DeviceTokenQueueDepth : 5;            // @58
        volatile const bool EnableIcUsb : 1;                        // @63
    };
};

struct __attribute__((__packed__, aligned(4))) CoreHardware2 {
    struct __attribute__((__packed__, aligned(1))) {
        volatile const unsigned TransferSizeControlWidth : 4;        // @64
        volatile const unsigned PacketSizeControlWidth : 3;            // @68
        volatile const bool otg_func : 1;                            // @71
        volatile const bool I2c : 1;                                // @72
        volatile const bool VendorControlInterface : 1;                // @73
        volatile const bool OptionalFeatures : 1;                    // @74
        volatile const bool SynchronousResetType : 1;                // @75
        volatile const bool AdpSupport : 1;                            // @76
        volatile const bool otg_enable_hsic : 1;                    // @77
        volatile const bool bc_support : 1;                            // @78
        volatile const bool LowPowerModeEnabled : 1;                // @79
        volatile const unsigned FifoDepth : 16;                        // @80
    };
};

struct __attribute__((__packed__, aligned(4))) CoreHardware3 {
    struct __attribute__((__packed__, aligned(1))) {
        volatile const unsigned PeriodicInEndpointCount : 4;        // @96
        volatile const bool PowerOptimisation : 1;                    // @100
        volatile const bool MinimumAhbFrequency : 1;                // @101
        volatile const bool PartialPowerOff : 1;                    // @102
        volatile const unsigned _reserved103_109 : 7;                // @103
        volatile const UtmiPhysicalDataWidthT UtmiPhysicalDataWidth : 2;    // @110
        volatile const unsigned ModeControlEndpointCount : 4;        // @112
        volatile const bool ValidFilterIddigEnabled : 1;            // @116
        volatile const bool VbusValidFilterEnabled : 1;                // @117
        volatile const bool ValidFilterAEnabled : 1;                // @118
        volatile const bool ValidFilterBEnabled : 1;                // @119
        volatile const bool SessionEndFilterEnabled : 1;            // @120
        volatile const bool ded_fifo_en : 1;                        // @121
        volatile const unsigned InEndpointCount : 4;                // @122
        volatile const bool DmaDescription : 1;                        // @126
        volatile const bool DmaDynamicDescription : 1;                // @127
    };
};

static_assert(sizeof(CoreHardware0) == 4, "CoreHardware0 must be 4 bytes");
static_assert(sizeof(CoreHardware1) == 4, "CoreHardware1 must be 4 bytes");
static_assert(sizeof(CoreHardware2) == 4, "CoreHardware2 must be 4 bytes");
static_assert(sizeof(CoreHardware3) == 4, "CoreHardware3 must be 4 bytes");

union PowerAndClockReg
{
    struct PACKED
    {
        bool     StopPClock             :  1; // @0
        bool     GateHClock             :  1; // @1
        bool     PowerClamp             :  1; // @2
        bool     PowerDownModules       :  1; // @3
        bool     PhySuspended           :  1; // @4
        bool     EnableSleepClockGating :  1; // @5
        bool     PhySleeping            :  1; // @6
        bool     DeepSleep              :  1; // @7
        unsigned _reserved8_31          : 24; // @8-31
    };
    uint32_t Raw32;
};

static_assert(sizeof(PowerAndClockReg) == 4, "PowerAndClockReg must be 4 bytes");


/***************************************************************************}
{    PRIVATE POINTERS TO ALL OUR DESIGNWARE 2.0 HOST REGISTER STRUCTURES    }
****************************************************************************/

#define USB_CORE_OFFSET  0x98'0000    // USB CORE OFFSET FROM PERIPHERAL IO BASE ADDRESS

/*--------------------------------------------------------------------------}
{                     DWC USB CORE REGISTER POINTERS                            }
{--------------------------------------------------------------------------*/
union CoreRegisters
{
    BootLib::Register<CoreOtgControl       ,  0x00> OTGCONTROL;
    BootLib::Register<CoreOtgInterrupt     ,  0x04> OTGINTERRUPT;
    BootLib::Register<CoreAhb              ,  0x08> AHB;
    BootLib::Register<UsbControl           ,  0x0C> CONTROL;
    BootLib::Register<CoreReset            ,  0x10> RESET;
    BootLib::Register<CoreInterrupts       ,  0x14> INTERRUPT;
    BootLib::Register<CoreInterrupts       ,  0x18> INTERRUPTMASK;
    BootLib::Register<uint32_t             ,  0x24> RECEIVESIZE;
    BootLib::Register<FifoSize             ,  0x28> NONPERIODICFIFO_SIZE;
    BootLib::Register<NonPeriodicFifoStatus,  0x2C> NONPERIODICFIFO_STATUS;
    BootLib::Register<const DwcUserId      ,  0x3C> USERID;
    BootLib::Register<const DwcVendorId    ,  0x40> VENDORID;
    BootLib::Register<const CoreHardware0  ,  0x44> HARDWARE0;
    BootLib::Register<const CoreHardware1  ,  0x48> HARDWARE1;
    BootLib::Register<const CoreHardware2  ,  0x4C> HARDWARE2;
    BootLib::Register<const CoreHardware3  ,  0x50> HARDWARE3;
    BootLib::Register<FifoSize             , 0x100> PERIODICINFO_HostSize;
    BootLib::RegisterArray<FifoSize        , 0x104, 15> PERIODICINFO_DataSize;
};

CoreRegisters* DWC_CORE;

constexpr Mmio::BaseRegisterProxy<PowerAndClockReg> DWC_POWER_AND_CLOCK{ USB_CORE_OFFSET + 0xE00 };


/***************************************************************************}
{                          PRIVATE INTERNAL VARIABLES                        }
****************************************************************************/

bool PhyInitialized = false;

/*==========================================================================}
{                       INTERNAL HOST CONTROL FUNCTIONS                        }
{==========================================================================*/

/*-INTERNAL: PowerOnUsb------------------------------------------------------
 Uses PI mailbox to turn power onto USB see website about command 0x28001
 https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
 11Feb17 LdB
 --------------------------------------------------------------------------*/
bool PowerOnUsb(void) {
    static uint32_t __attribute__((aligned(16))) volatile mailbox_message_buffer[8];
    auto mailbox_message = Mailbox::AsGpuPointer(mailbox_message_buffer);
    mailbox_message[0] = sizeof(mailbox_message);
    mailbox_message[1] = 0;
    mailbox_message[2] = (uint32_t)Mailbox::Tag::SET_POWER_STATE;
    mailbox_message[3] = 8;
    mailbox_message[4] = 8;
    mailbox_message[5] = 0x3;
    mailbox_message[6] = 0x1;
    mailbox_message[7] = 0x0;

    if (Mailbox::SendTags(std::span{ mailbox_message, 8 }) && (mailbox_message[4] == 0x80000008))
    {
        return true;
    }
    return false;
}

/*-INTERNAL: PowerOffUsb-----------------------------------------------------
 Uses PI mailbox to turn power onto USB see website about command 0x28001
 https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
 11Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT PowerOffUsb(void) {
    static uint32_t __attribute__((aligned(16))) volatile mailbox_message_buffer[8];
    auto mailbox_message = Mailbox::AsGpuPointer(mailbox_message_buffer);
    mailbox_message[0] = sizeof(mailbox_message);
    mailbox_message[1] = 0;
    mailbox_message[2] = (uint32_t)Mailbox::Tag::SET_POWER_STATE;
    mailbox_message[3] = 8;
    mailbox_message[4] = 8;
    mailbox_message[5] = 0x3;
    mailbox_message[6] = 0x0;
    mailbox_message[7] = 0x0;

    if (Mailbox::SendTags(std::span{ mailbox_message, 8 }) && (mailbox_message[4] == 0x80000008)) {
        return DWCRESULT::Ok;
    }
    return DWCRESULT::ErrorDevice;
}

/*-INTERNAL: HCDReset--------------------------------------------------------
 Does a softstart on core and uses ARM timer tick to timeout if neccessary.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT HCDReset(void) {

    if (!Cpu::WaitUntilWithTimeout(100ms, [&]{ return DWC_CORE->RESET->AhbMasterIdle; }))
    {
        return DWCRESULT::ErrorTimeout;
    }

    DWC_CORE->RESET = [](auto& r){ r.CoreSoft = true; };

    if (!Cpu::WaitUntilWithTimeout(100ms, [&]{
            CoreReset temp = *DWC_CORE->RESET;
            return !temp.CoreSoft && temp.AhbMasterIdle;
        }))
    {
        return DWCRESULT::ErrorTimeout;
    }

    return DWCRESULT::Ok;
}

/*-INTERNAL: HCDTransmitFifoFlush-------------------------------------------
 Flushes TX fifo buffers again uses ARM timer tick to timeout if neccessary.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT HCDTransmitFifoFlush(CoreFifoFlush fifo) {

    DWC_CORE->RESET = [=](auto& r){ r.TransmitFifoFlushNumber = fifo; };
    DWC_CORE->RESET = [](auto& r){ r.TransmitFifoFlush = true; };

    if (!Cpu::WaitUntilWithTimeout(100ms, [&]{ return !DWC_CORE->RESET->TransmitFifoFlush; }))
    {
        return DWCRESULT::ErrorTimeout;
    }

    return DWCRESULT::Ok;
}

/*-INTERNAL: HCDReceiveFifoFlush---------------------------------------------
 Flushes RX fifo buffers again uses ARM timer tick to timeout if neccessary.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT HCDReceiveFifoFlush(void) {

    DWC_CORE->RESET = [](auto& r){ r.ReceiveFifoFlush = true; };

    if (!Cpu::WaitUntilWithTimeout(100ms, [&]{ return !DWC_CORE->RESET->ReceiveFifoFlush; }))
    {
        return DWCRESULT::ErrorTimeout;
    }

    return DWCRESULT::Ok;
}

Interrupts::Spark InterruptHandler()
{
    // See what's what.
    CoreInterrupts interrupts = DWC_CORE->INTERRUPT;
    CoreInterrupts mask = DWC_CORE->INTERRUPTMASK;

    interrupts.Raw32 &= mask.Raw32; // Mask out the interrupts we are not interested in.

    if (interrupts.Port                        ) { Host->HandlePortInterrupt   (); }
    if (interrupts.HostChannel                 ) { Host->HandleChannelInterrupt(); }

    // During development, this helps us know what interrupts we don't know how to handle yet.
    CoreInterrupts todo = interrupts;
    todo.Port        = false;
    todo.HostChannel = false;

    if (todo.Raw32 != 0)
    {
        if (interrupts.CurrentMode                 ) { fmt::println("CurrentMode"                 ); }
        if (interrupts.ModeMismatch                ) { fmt::println("ModeMismatch"                ); }
        if (interrupts.Otg                         ) { fmt::println("Otg"                         ); }
        if (interrupts.DmaStartOfFrame             ) { fmt::println("DmaStartOfFrame"             ); }
        if (interrupts.ReceiveStatusLevel          ) { fmt::println("ReceiveStatusLevel"          ); }
        if (interrupts.NpTransmitFifoEmpty         ) { fmt::println("NpTransmitFifoEmpty"         ); }
        if (interrupts.ginnakeff                   ) { fmt::println("ginnakeff"                   ); }
        if (interrupts.goutnakeff                  ) { fmt::println("goutnakeff"                  ); }
        if (interrupts.ulpick                      ) { fmt::println("ulpick"                      ); }
        if (interrupts.I2c                         ) { fmt::println("I2c"                         ); }
        if (interrupts.EarlySuspend                ) { fmt::println("EarlySuspend"                ); }
        if (interrupts.UsbSuspend                  ) { fmt::println("UsbSuspend"                  ); }
        if (interrupts.UsbReset                    ) { fmt::println("UsbReset"                    ); }
        if (interrupts.EnumerationDone             ) { fmt::println("EnumerationDone"             ); }
        if (interrupts.IsochronousOutDrop          ) { fmt::println("IsochronousOutDrop"          ); }
        if (interrupts.eopframe                    ) { fmt::println("eopframe"                    ); }
        if (interrupts.RestoreDone                 ) { fmt::println("RestoreDone"                 ); }
        if (interrupts.EndPointMismatch            ) { fmt::println("EndPointMismatch"            ); }
        if (interrupts.InEndPoint                  ) { fmt::println("InEndPoint"                  ); }
        if (interrupts.OutEndPoint                 ) { fmt::println("OutEndPoint"                 ); }
        if (interrupts.IncompleteIsochronousIn     ) { fmt::println("IncompleteIsochronousIn"     ); }
        if (interrupts.IncompleteIsochronousOut    ) { fmt::println("IncompleteIsochronousOut"    ); }
        if (interrupts.fetsetup                    ) { fmt::println("fetsetup"                    ); }
        if (interrupts.ResetDetect                 ) { fmt::println("ResetDetect"                 ); }
        if (interrupts.HpTransmitFifoEmpty         ) { fmt::println("HpTransmitFifoEmpty"         ); }
        if (interrupts.LowPowerModeTransmitReceived) { fmt::println("LowPowerModeTransmitReceived"); }
        if (interrupts.ConnectionIdStatusChange    ) { fmt::println("ConnectionIdStatusChange"    ); }
        if (interrupts.Disconnect                  ) { fmt::println("Disconnect"                  ); }
        if (interrupts.SessionRequest              ) { fmt::println("SessionRequest"              ); }
        if (interrupts.Wakeup                      ) { fmt::println("Wakeup"                      ); }
    }

    DWC_CORE->INTERRUPT = interrupts;

    return {};
}

uint8_t HCDGetHostChannelCount()
{
    return DWC_CORE->HARDWARE1->HostChannelMax + 1;
}

/*==========================================================================}
{                       INTERNAL HOST CONTROL FUNCTIONS                        }
{==========================================================================*/

Async::task<std::expected<std::shared_ptr<HCDHost>, DWCRESULT>> HCDInitialize()
{
    auto const registersAddress = Mmio::Base + USB_CORE_OFFSET;
    DWC_CORE = reinterpret_cast<CoreRegisters*>(registersAddress);

    auto vendorId = *DWC_CORE->VENDORID;
    auto userId   = *DWC_CORE->USERID;
    LOG("HCD: Hardware: {}{}{:x}.{:03x} (BCM{:05x}).\n",
        static_cast<char>(vendorId.O), static_cast<char>(vendorId.T), static_cast<unsigned>(vendorId.Version), static_cast<unsigned>(vendorId.Revision),
        static_cast<unsigned>(userId.Id)
    );

    if (vendorId.O != 'O' || vendorId.T != 'T' || vendorId.Version != 2)
    {
        LOG("Driver incompatible. Expected OT2.xxx (BCM2708x).\n");
        co_return std::unexpected{ DWCRESULT::ErrorIncompatible };
    }

    if (DWC_CORE->HARDWARE1->Architecture != InternalDma)
    {
        // We only allow DMA transfer
        LOG("HCD: Host architecture does not support Internal DMA\n");
        co_return std::unexpected{ DWCRESULT::ErrorIncompatible };
    }

    if (DWC_CORE->HARDWARE1->HighSpeedPhysical == NotSupported)
    {
        // We need high speed transfers
        LOG("HCD: High speed physical unsupported\n");
        co_return std::unexpected{ DWCRESULT::ErrorIncompatible };
    }

    // Clear and disable all interrupts.
    DWC_CORE->AHB           = [](auto&reg){ reg.InterruptEnable = false; };
    DWC_CORE->INTERRUPTMASK = 0;
    DWC_CORE->INTERRUPT     = 0xFFFF'FFFFu;

    if (!PowerOnUsb())
    {
        LOG("HCD: Failed to power on USB Host Controller.\n");
        co_return std::unexpected{ DWCRESULT::ErrorIncompatible };
    }

    // And now we set it up.

    DWCRESULT result;

    UsbControl coreUsb = *DWC_CORE->CONTROL;
    coreUsb.UlpiDriveExternalVbus = 0;
    coreUsb.TsDlinePulseEnable = 0;
    DWC_CORE->CONTROL = coreUsb;

    /*co_await Async*/ Cpu::Delay(1ms);

    LOG_DEBUG("HCD: Master reset.\n");                                
    if ((result = HCDReset()) != DWCRESULT::Ok) {
        LOG("FATAL ERROR: Could not do a Master reset on HCD.\n");
        co_return std::unexpected{ result };
    }

    /*co_await Async*/ Cpu::Delay(1ms);

    if (!PhyInitialized) {
        LOG_DEBUG("HCD: One time phy initialisation.\n");
        PhyInitialized = true;
        coreUsb = *DWC_CORE->CONTROL;
        coreUsb.ModeSelect = UTMI;
        LOG_DEBUG("HCD: Interface: UTMI+.\n");                        
        coreUsb.PhyInterface = false;
        DWC_CORE->CONTROL = coreUsb;
        if ((result = HCDReset()) != DWCRESULT::Ok) {
            LOG("FATAL ERROR: Could not do a Master reset on HCD.\n");
            co_return std::unexpected{ result };
        }
    }

    /*co_await Async*/ Cpu::Delay(1ms);

    coreUsb = *DWC_CORE->CONTROL;
    if ((*DWC_CORE->HARDWARE1).HighSpeedPhysical == Ulpi
        && (*DWC_CORE->HARDWARE1).FullSpeedPhysical == Dedicated) {
        LOG_DEBUG("HCD: ULPI FSLS configuration: enabled.\n");    
        coreUsb.UlpiFsls = true;                                
        coreUsb.ulpi_clk_sus_m = true;
    } else {
        LOG_DEBUG("HCD: ULPI FSLS configuration: disabled.\n");
        coreUsb.UlpiFsls = false;
        coreUsb.ulpi_clk_sus_m = false;
    }
    DWC_CORE->CONTROL = coreUsb;

    /*co_await Async*/ Cpu::Delay(1ms);

    CoreAhb tempAhb = *DWC_CORE->AHB;
    tempAhb.DmaEnable = true;
    tempAhb.DmaRemainderMode = Incremental;
    DWC_CORE->AHB = tempAhb;

    /*co_await Async*/ Cpu::Delay(1ms);

    coreUsb = *DWC_CORE->CONTROL;
    switch ((*DWC_CORE->HARDWARE1).OperatingMode) {
    case HNP_SRP_CAPABLE:
        LOG_DEBUG("HCD: HNP/SRP configuration: HNP, SRP.\n");
        coreUsb.HnpCapable = true;
        coreUsb.SrpCapable = true;
        break;
    case SRP_ONLY_CAPABLE:
    case SRP_CAPABLE_DEVICE:
    case SRP_CAPABLE_HOST:
        LOG_DEBUG("HCD: HNP/SRP configuration: SRP.\n");
        coreUsb.HnpCapable = false;
        coreUsb.SrpCapable = true;
        break;
    case NO_HNP_SRP_CAPABLE:
    case NO_SRP_CAPABLE_DEVICE:
    case NO_SRP_CAPABLE_HOST:
        LOG_DEBUG("HCD: HNP/SRP configuration: none.\n");
        coreUsb.HnpCapable = false;
        coreUsb.SrpCapable = false;
        break;
    }
    DWC_CORE->CONTROL = coreUsb;
    LOG_DEBUG("HCD: Core started.\n");
    LOG_DEBUG("HCD: Starting host.\n");

    /*co_await Async*/ Cpu::Delay(1ms);

    DWC_POWER_AND_CLOCK = {};

    /*co_await Async*/ Cpu::Delay(1ms);

    DWC_CORE->RECEIVESIZE = ReceiveFifoSize;

    /*co_await Async*/ Cpu::Delay(1ms);

    DWC_CORE->NONPERIODICFIFO_SIZE = [](auto& r)
    {
        r.Depth        = NonPeriodicFifoSize;
        r.StartAddress = ReceiveFifoSize;
    };

    /*co_await Async*/ Cpu::Delay(1ms);

    DWC_CORE->PERIODICINFO_HostSize = [](auto& r)
    {
        r.Depth        = PeriodicFifoSize;
        r.StartAddress = ReceiveFifoSize + NonPeriodicFifoSize;
    };

    /*co_await Async*/ Cpu::Delay(1ms);

    LOG_DEBUG("HCD: Set HNP: enabled.\n");

    CoreOtgControl tempOtgControl = *DWC_CORE->OTGCONTROL;
    tempOtgControl.HostSetHnpEnable = true;
    DWC_CORE->OTGCONTROL = tempOtgControl;
    //DWC_CORE->OTGINTERRUPT = 0xFFFF'FFFFu; // Clear all OTG interrupts

    /*co_await Async*/ Cpu::Delay(1ms);

    if ((result = HCDTransmitFifoFlush(FlushAll)) != DWCRESULT::Ok)
        co_return std::unexpected{ result };
    /*co_await Async*/ Cpu::Delay(1ms);

    if ((result = HCDReceiveFifoFlush()) != DWCRESULT::Ok)
        co_return std::unexpected{ result };
    /*co_await Async*/ Cpu::Delay(1ms);

    HCDHost::ClockRate clockRate;

    if (DWC_CORE->HARDWARE1->HighSpeedPhysical == Ulpi
        && DWC_CORE->HARDWARE1->FullSpeedPhysical == Dedicated
        && coreUsb.UlpiFsls) {
        LOG_DEBUG("HCD: Host clock: 48Mhz.\n");
        clockRate = HCDHost::ClockRate::Clock48MHz;
    } else {
        LOG_DEBUG("HCD: Host clock: 30-60Mhz.\n");
        clockRate = HCDHost::ClockRate::Clock30_60MHz;
    }

    Host = co_await HCDHost::Make(registersAddress + 0x400, clockRate, DWC_CORE->HARDWARE1->HostChannelMax + 1);

    Interrupts::EnableUsb(InterruptHandler);

    DWC_CORE->AHB = [](auto& ahb){ ahb.InterruptEnable = true; };
    DWC_CORE->INTERRUPT = 0xFFFFFFFF; // Setting bits clears interrupts
    DWC_CORE->INTERRUPTMASK = {
        //.ReceiveStatusLevel = true, // This causes a lot of spurious interrupts without the status bit getting set.
        //.EnumerationDone = true,
        //.InEndPoint = true,
        //.OutEndPoint = true,
        //.Port = true, // Port status changes (connections, disconnections). Handling them prevents the work of the initial enumeration. TODO: Fix this.
        .HostChannel = true,
        //.ConnectionIdStatusChange = true,
        //.Disconnect = true,
        //.SessionRequest = true,
    }

    LOG_DEBUG("HCD: Interrupts enabled.\n");
    LOG_DEBUG("HCD: Successfully started.\n");

    co_return Host;
}
