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
#include <stdbool.h>			// C standard needed for bool
#include <stdlib.h>				// C standard needed for NULL
#include <stdint.h>				// C standard needed for uint8_t, uint32_t, uint64_t etc
#include <string.h>				// C standard needed for memset
#include <wchar.h>				// C standard needed for UTF for unicode descriptor support
#include "DesignWareUsb.h"			// This units header

#include "Cpu.h"
#include "Mmio.h"
#include "Mailbox.h"
#include "Timer.h"
#include "Processor.h"
#include "Interrupts.h"

#include "emb-stdio.h"				// Needed for printf

#include <concepts>

#define LOG(...)
//#define LOG(...) printf2(__VA_ARGS__)
#define LOG_DEBUG(...)
//#define LOG_DEBUG(...) printf2(__VA_ARGS__)



#define RPi_IO_Base_Addr Mmio::Base

#define ReceiveFifoSize 20480 /* 16 to 32768 */
#define NonPeriodicFifoSize 20480 /* 16 to 32768 */
#define PeriodicFifoSize 20480 /* 16 to 32768 */

/***************************************************************************}
{						 PRIVATE INTERNAL ENUMERATIONS					    }
****************************************************************************/

/*--------------------------------------------------------------------------}
{	       FLUSH TYPE ENUMERATION FOR FIFO ON THE DESIGNWARE 2.0		    }
{--------------------------------------------------------------------------*/
enum CoreFifoFlush {
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
{	      INTERRUPT BITS ON THE USB CHANNELS ON THE DESIGNWARE 2.0		    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) ChannelInterrupts {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile bool TransferComplete : 1;						// @0
            volatile bool Halt : 1;									// @1
            volatile bool AhbError : 1;								// @2
            volatile bool Stall : 1;								// @3
            volatile bool NegativeAcknowledgement : 1;				// @4
            volatile bool Acknowledgement : 1;						// @5
            volatile bool NotYet : 1;								// @6
            volatile bool TransactionError : 1;						// @7
            volatile bool BabbleError : 1;							// @8
            volatile bool FrameOverrun : 1;							// @9
            volatile bool DataToggleError : 1;						// @10
            volatile bool BufferNotAvailable : 1;					// @11
            volatile bool ExcessiveTransmission : 1;				// @12
            volatile bool FrameListRollover : 1;					// @13
            unsigned _reserved14_31 : 18;							// @14-31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{	   FIFOSIZE STRUCTURE .. THERE ARE A FEW OF THESE ON DESIGNWARE 2.0     }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) FifoSize {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile unsigned StartAddress : 16;					// @0
            volatile unsigned Depth : 16;							// @16
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{					   USB CORE OTG CONTROL STRUCTURE					    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) CoreOtgControl {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile bool sesreqscs : 1;							// @0
            volatile bool sesreq : 1;								// @1
            volatile bool vbvalidoven : 1;							// @2
            volatile bool vbvalidovval : 1;							// @3
            volatile bool avalidoven : 1;							// @4
            volatile bool avalidovval : 1;							// @5
            volatile bool bvalidoven : 1;							// @6
            volatile bool bvalidovval : 1;							// @7
            volatile bool hstnegscs : 1;							// @8
            volatile bool hnpreq : 1;								// @9
            volatile bool HostSetHnpEnable : 1;						// @10
            volatile bool devhnpen : 1;								// @11
            volatile unsigned _reserved12_15 : 4;					// @12-15
            volatile bool conidsts : 1;								// @16
            volatile unsigned dbnctime : 1;							// @17
            volatile bool ASessionValid : 1;						// @18
            volatile bool BSessionValid : 1;						// @19
            volatile unsigned OtgVersion : 1;						// @20
            volatile unsigned _reserved21 : 1;						// @21
            volatile unsigned multvalidbc : 5;						// @22-26
            volatile bool chirpen : 1;								// @27
            volatile unsigned _reserved28_31 : 4;					// @28-31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{					 USB CORE OTG INTERRUPT STRUCTURE					    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) CoreOtgInterrupt {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile unsigned _reserved0_1 : 2;						// @0
            volatile bool SessionEndDetected : 1;					// @2
            volatile unsigned _reserved3_7 : 5;						// @3
            volatile bool SessionRequestSuccessStatusChange : 1;	// @8
            volatile bool HostNegotiationSuccessStatusChange : 1;	// @9
            volatile unsigned _reserved10_16 : 7;					// @10
            volatile bool HostNegotiationDetected : 1;				// @17
            volatile bool ADeviceTimeoutChange : 1;					// @18
            volatile bool DebounceDone : 1;							// @19
            volatile unsigned _reserved20_31 : 12;					// @20-31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{	 USB CORE AHB STRUCTURE ... CARE WRITE WHOLE REGISTER .. NO BIT OPS	    }
{--------------------------------------------------------------------------*/
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
struct __attribute__((__packed__, aligned(4))) CoreAhb {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile bool InterruptEnable : 1;						// @0
            volatile AxiBurstLength AxiBurstLength : 2;						// @1
            volatile unsigned _reserved3 : 1;						// @3
            volatile bool WaitForAxiWrites : 1;						// @4
            volatile bool DmaEnable : 1;							// @5
            volatile unsigned _reserved6 : 1;						// @6
            volatile EmptyLevel TransferEmptyLevel : 1;				// @7
            volatile EmptyLevel PeriodicTransferEmptyLevel : 1;		// @8
            volatile unsigned _reserved9_20 : 12;					// @9
            volatile bool remmemsupp : 1;							// @21
            volatile bool notialldmawrit : 1;						// @22
            volatile DmaRemainderMode DmaRemainderMode : 1;			// @23
            volatile unsigned _reserved24_31 : 8;					// @24-31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{	USB CORE CONTROL STRUCTURE	.. CARE WRITE WHOLE REGISTER .. NO BIT OPS  }
{--------------------------------------------------------------------------*/
enum UMode {
    ULPI,
    UTMI,
};
struct __attribute__((__packed__, aligned(4))) UsbControl {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile unsigned toutcal : 3;							// @0
            volatile bool PhyInterface : 1;							// @3
            volatile UMode ModeSelect : 1;							// @4
            volatile bool fsintf : 1;								// @5
            volatile bool physel : 1;								// @6
            volatile bool ddrsel : 1;								// @7
            volatile bool SrpCapable : 1;							// @8
            volatile bool HnpCapable : 1;							// @9
            volatile unsigned usbtrdtim : 4;						// @10
            volatile unsigned reserved1 : 1;						// @14
            volatile bool phy_lpm_clk_sel : 1;						// @15
            volatile bool otgutmifssel : 1;							// @16
            volatile bool UlpiFsls : 1;								// @17
            volatile bool ulpi_auto_res : 1;						// @18
            volatile bool ulpi_clk_sus_m : 1;						// @19
            volatile bool UlpiDriveExternalVbus : 1;				// @20
            volatile bool ulpi_int_vbus_indicator : 1;				// @21
            volatile bool TsDlinePulseEnable : 1;					// @22
            volatile bool indicator_complement : 1;					// @23
            volatile bool indicator_pass_through : 1;				// @24
            volatile bool ulpi_int_prot_dis : 1;					// @25
            volatile bool ic_usb_capable : 1;						// @26
            volatile bool ic_traffic_pull_remove : 1;				// @27
            volatile bool tx_end_delay : 1;							// @28
            volatile bool force_host_mode : 1;						// @29
            volatile bool force_dev_mode : 1;						// @30
            volatile unsigned _reserved31 : 1;						// @31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{							 USB CORE RESET STRUCTURE					    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) CoreReset {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile bool CoreSoft : 1;								// @0
            volatile bool HclkSoft : 1;								// @1
            volatile bool HostFrameCounter : 1;						// @2
            volatile bool InTokenQueueFlush : 1;					// @3
            volatile bool ReceiveFifoFlush : 1;						// @4
            volatile bool TransmitFifoFlush : 1;					// @5
            volatile unsigned TransmitFifoFlushNumber : 5;			// @6
            volatile unsigned _reserved11_29 : 19;					// @11
            volatile bool DmaRequestSignal : 1;						// @30
            volatile bool AhbMasterIdle : 1;						// @31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{	       INTERRUPT BITS ON THE USB CORE OF THE DESIGNWARE 2.0		        }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) CoreInterrupts {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile bool CurrentMode : 1;							// @0
            volatile bool ModeMismatch : 1;							// @1
            volatile bool Otg : 1;									// @2
            volatile bool DmaStartOfFrame : 1;						// @3
            volatile bool ReceiveStatusLevel : 1;					// @4
            volatile bool NpTransmitFifoEmpty : 1;					// @5
            volatile bool ginnakeff : 1;							// @6
            volatile bool goutnakeff : 1;							// @7
            volatile bool ulpick : 1;								// @8
            volatile bool I2c : 1;									// @9
            volatile bool EarlySuspend : 1;							// @10
            volatile bool UsbSuspend : 1;							// @11
            volatile bool UsbReset : 1;								// @12
            volatile bool EnumerationDone : 1;						// @13
            volatile bool IsochronousOutDrop : 1;					// @14
            volatile bool eopframe : 1;								// @15
            volatile bool RestoreDone : 1;							// @16
            volatile bool EndPointMismatch : 1;						// @17
            volatile bool InEndPoint : 1;							// @18
            volatile bool OutEndPoint : 1;							// @19
            volatile bool IncompleteIsochronousIn : 1;				// @20
            volatile bool IncompleteIsochronousOut : 1;				// @21
            volatile bool fetsetup : 1;								// @22
            volatile bool ResetDetect : 1;							// @23
            volatile bool Port : 1;									// @24
            volatile bool HostChannel : 1;							// @25
            volatile bool HpTransmitFifoEmpty : 1;					// @26
            volatile bool LowPowerModeTransmitReceived : 1;			// @27
            volatile bool ConnectionIdStatusChange : 1;				// @28
            volatile bool Disconnect : 1;							// @29
            volatile bool SessionRequest : 1;						// @30
            volatile bool Wakeup : 1;								// @31
        };
        volatile uint32_t Raw32;									// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{				 USB CORE NON PERIODIC FIFO STATUS STRUCTURE			    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) NonPeriodicFifoStatus {
    enum TokenTypeT {
        InOut = 0,
        ZeroLengthOut = 1,
        PingCompleteSplit = 2,
        ChannelHalt = 3,
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
{						 USB CORE HARDWARE STRUCTURE					    }
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
        volatile const unsigned Direction0 : 2;						// @0
        volatile const unsigned Direction1 : 2;						// @2
        volatile const unsigned Direction2 : 2;						// @4
        volatile const unsigned Direction3 : 2;						// @6
        volatile const unsigned Direction4 : 2;						// @8
        volatile const unsigned Direction5 : 2;						// @10
        volatile const unsigned Direction6 : 2;						// @12
        volatile const unsigned Direction7 : 2;						// @14
        volatile const unsigned Direction8 : 2;						// @16
        volatile const unsigned Direction9 : 2;						// @18
        volatile const unsigned Direction10 : 2;					// @20
        volatile const unsigned Direction11 : 2;					// @22
        volatile const unsigned Direction12 : 2;					// @24
        volatile const unsigned Direction13 : 2;					// @26
        volatile const unsigned Direction14 : 2;					// @28
        volatile const unsigned Direction15 : 2;					// @30
    };
};

struct __attribute__((__packed__, aligned(4))) CoreHardware1 {
    struct __attribute__((__packed__, aligned(1))) {
        volatile const OperatingModeT OperatingMode : 3;			// @32-34
        volatile const ArchitectureT Architecture : 2;				// @35
        volatile bool PointToPoint : 1;								// @37
        volatile const HighSpeedPhysicalT HighSpeedPhysical : 2;	// @38-39
        volatile const FullSpeedPhysicalT FullSpeedPhysical : 2;	// @40-41
        volatile const unsigned DeviceEndPointCount : 4;			// @42
        volatile const unsigned HostChannelCount : 4;				// @46
        volatile const bool SupportsPeriodicEndpoints : 1;			// @50
        volatile const bool DynamicFifo : 1;						// @51
        volatile const bool multi_proc_int : 1;						// @52
        volatile const unsigned _reserver21 : 1;					// @53
        volatile const unsigned NonPeriodicQueueDepth : 2;			// @54
        volatile const unsigned HostPeriodicQueueDepth : 2;			// @56
        volatile const unsigned DeviceTokenQueueDepth : 5;			// @58
        volatile const bool EnableIcUsb : 1;						// @63
    };
};

struct __attribute__((__packed__, aligned(4))) CoreHardware2 {
    struct __attribute__((__packed__, aligned(1))) {
        volatile const unsigned TransferSizeControlWidth : 4;		// @64
        volatile const unsigned PacketSizeControlWidth : 3;			// @68
        volatile const bool otg_func : 1;							// @71
        volatile const bool I2c : 1;								// @72
        volatile const bool VendorControlInterface : 1;				// @73
        volatile const bool OptionalFeatures : 1;					// @74
        volatile const bool SynchronousResetType : 1;				// @75
        volatile const bool AdpSupport : 1;							// @76
        volatile const bool otg_enable_hsic : 1;					// @77
        volatile const bool bc_support : 1;							// @78
        volatile const bool LowPowerModeEnabled : 1;				// @79
        volatile const unsigned FifoDepth : 16;						// @80
    };
};

struct __attribute__((__packed__, aligned(4))) CoreHardware3 {
    struct __attribute__((__packed__, aligned(1))) {
        volatile const unsigned PeriodicInEndpointCount : 4;		// @96
        volatile const bool PowerOptimisation : 1;					// @100
        volatile const bool MinimumAhbFrequency : 1;				// @101
        volatile const bool PartialPowerOff : 1;					// @102
        volatile const unsigned _reserved103_109 : 7;				// @103
        volatile const UtmiPhysicalDataWidthT UtmiPhysicalDataWidth : 2;	// @110
        volatile const unsigned ModeControlEndpointCount : 4;		// @112
        volatile const bool ValidFilterIddigEnabled : 1;			// @116
        volatile const bool VbusValidFilterEnabled : 1;				// @117
        volatile const bool ValidFilterAEnabled : 1;				// @118
        volatile const bool ValidFilterBEnabled : 1;				// @119
        volatile const bool SessionEndFilterEnabled : 1;			// @120
        volatile const bool ded_fifo_en : 1;						// @121
        volatile const unsigned InEndpointCount : 4;				// @122
        volatile const bool DmaDescription : 1;						// @126
        volatile const bool DmaDynamicDescription : 1;				// @127
    };
};

/*--------------------------------------------------------------------------}
{                       USB CORE PERIODIC INFO STRUCTURE				    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) CorePeriodicInfo {
    volatile __attribute__((aligned(4))) FifoSize HostSize;	// +0x100
    volatile __attribute__((aligned(4))) FifoSize DataSize[15];// +0x104
};

/***************************************************************************}
{         PRIVATE INTERNAL DESIGNWARE 2.0 HOST REGISTER STRUCTURES          }
****************************************************************************/

enum ClockRate : unsigned {
    Clock30_60MHz,													// 30-60Mhz clock to USB
    Clock48MHz,														// 48Mhz clock to USB
    Clock6MHz,														// 6Mhz clock to USB
};

/*--------------------------------------------------------------------------}
{                          USB HOST CONFIG STRUCTURE					    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) HostConfig {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            volatile ::ClockRate ClockRate : 2;						// @0
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
            volatile bool ConnectChanged : 1;						// @1
            volatile bool Enable : 1;								// @2
            volatile bool EnableChanged : 1;						// @3
            volatile bool OverCurrent : 1;							// @4
            volatile bool OverCurrentChanged : 1;					// @5
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
{                USB HOST CHANNEL CHARACTERISTIC STRUCTURE				    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(1))) HostChannelCharacteristic {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            unsigned max_packet_size : 11;					// @0-10	Maximum packet size the endpoint is capable of sending or receiving
            unsigned endpoint_number : 4;					// @11-14	Endpoint number (low 4 bits of bEndpointAddress)
            unsigned endpoint_direction : 1;				// @15		Endpoint direction 1=IN, 0=OUT
            unsigned _reserved : 1;							// @16
            unsigned low_speed : 1;							// @17		1 when the device being communicated with is at low speed, 0 otherwise
            unsigned endpoint_type : 2;						// @18-19	Endpoint type (low 2 bits of bmAttributes)
            unsigned packets_per_frame : 2;					// @20-21	Maximum number of transactions that can be executed per microframe
            unsigned device_address : 7;					// @22-28	USB device address of the device on which the endpoint is located
            unsigned odd_frame : 1;							// @29		Before enabling channel must be set to opposite of low bit of host_frame_number
            unsigned channel_disable : 1;					// @30		Software can set this to 1 to halt the channel
            unsigned channel_enable : 1;					// @31		Software can set this to 1 to enable the channel
        };
        volatile uint32_t Raw32;							// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{                USB HOST CHANNEL SPLIT CONTROL STRUCTURE				    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(1))) HostChannelSplitControl {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            unsigned port_address : 7;						// @0-6		0-based index of the port on the high-speed hub Transaction Translator occurs
            unsigned hub_address : 7;						// @7-13	USB device address of the high-speed hub that acts as Transaction Translator
            unsigned transaction_position : 2;				// @14-15	If we are processing isochronous OUT split the transation position Begin=2,End=1,Middle=0,All=3
            unsigned complete_split : 1;					// @16		1 to complete a Split transaction, 0 = normal transaction
            unsigned _reserved : 14;						// @17-30
            unsigned split_enable : 1;						// @31		Set to 1 to enable Split Transactions
        };
        volatile uint32_t Raw32;							// Union to access all 32 bits as a uint32_t
    };
};

/*--------------------------------------------------------------------------}
{                USB HOST CHANNEL TRANSFER SIZE STRUCTURE				    }
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(1))) HostTransferSize {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            unsigned size : 19;								// @0-18	Size of data to send or receive, in bytes and can be greater than maximum packet length
            unsigned packet_count : 10;						// @19-28   Number of packets left to transmit or maximum number of packets left to receive
            PacketId packet_id : 2;							// @29		Various packet phase ID
            unsigned do_ping : 1;							// @31		
        };
        volatile uint32_t Raw32;							// Union to access all 32 bits as a uint32_t
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

/*--------------------------------------------------------------------------}
{ 				USB control used solely by internal routines				}
{--------------------------------------------------------------------------*/
struct __attribute__((__packed__, aligned(4))) UsbSendControl {
    union {
        struct __attribute__((__packed__, aligned(1))) {
            unsigned SplitTries : 8;								// @0  Count of attempts to send packet as a split
            unsigned PacketTries : 8;								// @8  Count of attempts to send current packet
            unsigned GlobalTries : 8;								// @16 Count of global tries (more serious errors increment)
            unsigned reserved : 3;									// @24 Padding to make 32 bit
            bool	 LongerDelay : 1;								// @27 Longer delay .. not yet was response
            bool	 ActionResendSplit : 1;							// @28 Resend split packet
            bool	 ActionRetry : 1;								// @29 Retry sending 
            bool	 ActionFatalError : 1;							// @30 Some fatal error occured ... so bail
            bool	 Success : 1;									// @31 Success .. tansfer complete
        };
        uint32_t Raw32;												// Union to access all 32 bits as a uint32_t
    };
};

static_assert(sizeof(struct UsbSendControl) == 0x04, "Structure should be 32bits (4 bytes)");


/***************************************************************************}
{    PRIVATE POINTERS TO ALL OUR DESIGNWARE 2.0 HOST REGISTER STRUCTURES    }
****************************************************************************/

#define USB_CORE_OFFSET  0x980000	// USB CORE OFFSET FROM PERIPHERAL IO BASE ADDRESS

/*--------------------------------------------------------------------------}
{					 DWC USB CORE REGISTER POINTERS						    }
{--------------------------------------------------------------------------*/
constexpr Mmio::BaseRegisterProxy<CoreOtgControl       > DWC_CORE_OTGCONTROL            { USB_CORE_OFFSET +  0x00 };
constexpr Mmio::BaseRegisterProxy<CoreOtgInterrupt     > DWC_CORE_OTGINTERRUPT          { USB_CORE_OFFSET +  0x04 };
constexpr Mmio::BaseRegisterProxy<CoreAhb              > DWC_CORE_AHB                   { USB_CORE_OFFSET +  0x08 };
constexpr Mmio::BaseRegisterProxy<UsbControl           > DWC_CORE_CONTROL               { USB_CORE_OFFSET +  0x0C };
constexpr Mmio::BaseRegisterProxy<CoreReset            > DWC_CORE_RESET                 { USB_CORE_OFFSET +  0x10 };
constexpr Mmio::BaseRegisterProxy<CoreInterrupts       > DWC_CORE_INTERRUPT             { USB_CORE_OFFSET +  0x14 };
constexpr Mmio::BaseRegisterProxy<CoreInterrupts       > DWC_CORE_INTERRUPTMASK         { USB_CORE_OFFSET +  0x18 };
constexpr Mmio::BaseRegisterProxy<uint32_t             > DWC_CORE_RECEIVESIZE           { USB_CORE_OFFSET +  0x24 };
constexpr Mmio::BaseRegisterProxy<FifoSize             > DWC_CORE_NONPERIODICFIFO_SIZE  { USB_CORE_OFFSET +  0x28 };
constexpr Mmio::BaseRegisterProxy<NonPeriodicFifoStatus> DWC_CORE_NONPERIODICFIFO_STATUS{ USB_CORE_OFFSET +  0x2C };
constexpr Mmio::BaseRegisterProxy<uint32_t             > DWC_CORE_USERID                { USB_CORE_OFFSET +  0x3C };
constexpr Mmio::BaseRegisterProxy<const uint32_t       > DWC_CORE_VENDORID              { USB_CORE_OFFSET +  0x40 };
constexpr Mmio::BaseRegisterProxy<const CoreHardware0  > DWC_CORE_HARDWARE0             { USB_CORE_OFFSET +  0x44 };
constexpr Mmio::BaseRegisterProxy<const CoreHardware1  > DWC_CORE_HARDWARE1             { USB_CORE_OFFSET +  0x48 };
constexpr Mmio::BaseRegisterProxy<const CoreHardware2  > DWC_CORE_HARDWARE2             { USB_CORE_OFFSET +  0x4C };
constexpr Mmio::BaseRegisterProxy<const CoreHardware3  > DWC_CORE_HARDWARE3             { USB_CORE_OFFSET +  0x50 };
constexpr Mmio::BaseRegisterProxy<FifoSize             > DWC_CORE_PERIODICINFO_HostSize { USB_CORE_OFFSET + 0x100 };

/*--------------------------------------------------------------------------}
{					DWC USB HOST REGISTER POINTERS						    }
{--------------------------------------------------------------------------*/
constexpr Mmio::BaseRegisterProxy<HostConfig               > DWC_HOST_CONFIG                { USB_CORE_OFFSET + 0x400 };
constexpr Mmio::BaseRegisterProxy<HostFrameInterval        > DWC_HOST_FRAMEINTERVAL         { USB_CORE_OFFSET + 0x404 };
constexpr Mmio::BaseRegisterProxy<HostFrameControl         > DWC_HOST_FRAMECONTROL          { USB_CORE_OFFSET + 0x408 };
constexpr Mmio::BaseRegisterProxy<HostFifoStatus           > DWC_HOST_FIFOSTATUS            { USB_CORE_OFFSET + 0x410 };
constexpr Mmio::BaseRegisterProxy<uint32_t                 > DWC_HOST_INTERRUPT             { USB_CORE_OFFSET + 0x414 };
constexpr Mmio::BaseRegisterProxy<uint32_t                 > DWC_HOST_INTERRUPTMASK         { USB_CORE_OFFSET + 0x418 };
constexpr Mmio::BaseRegisterProxy<uint32_t                 > DWC_HOST_FRAMELIST             { USB_CORE_OFFSET + 0x41C };
constexpr Mmio::BaseRegisterProxy<HostPort                 > DWC_HOST_PORT                  { USB_CORE_OFFSET + 0x440 };
constexpr Mmio::BaseRegisterArrayProxy<HostChannelCharacteristic, 8, 8, USB_CORE_OFFSET + 0x500 > DWC_HOST_CHANNEL_Characteristic;
constexpr Mmio::BaseRegisterArrayProxy<HostChannelSplitControl  , 8, 8, USB_CORE_OFFSET + 0x504 > DWC_HOST_CHANNEL_SplitCtrl;
constexpr Mmio::BaseRegisterArrayProxy<ChannelInterrupts        , 8, 8, USB_CORE_OFFSET + 0x508 > DWC_HOST_CHANNEL_Interrupt;
constexpr Mmio::BaseRegisterArrayProxy<ChannelInterrupts        , 8, 8, USB_CORE_OFFSET + 0x50C > DWC_HOST_CHANNEL_InterruptMask;
constexpr Mmio::BaseRegisterArrayProxy<HostTransferSize         , 8, 8, USB_CORE_OFFSET + 0x510 > DWC_HOST_CHANNEL_TransferSize;
constexpr Mmio::BaseRegisterArrayProxy<uint32_t                 , 8, 8, USB_CORE_OFFSET + 0x514 > DWC_HOST_CHANNEL_DmaAddr;

/*--------------------------------------------------------------------------}
{					DWC POWER AND CLOCK REGISTER POINTER				    }
{--------------------------------------------------------------------------*/
constexpr Mmio::BaseRegisterProxy<PowerReg> DWC_POWER_AND_CLOCK{ USB_CORE_OFFSET + 0xE00 };



/***************************************************************************}
{					      PRIVATE INTERNAL CONSTANTS	                    }
****************************************************************************/

/**
 * Number of DWC host channels, each of which can be used for an independent
 * USB transfer.  On the BCM2835 (Raspberry Pi), 8 are available.  This is
 * documented on page 201 of the BCM2835 ARM Peripherals document.
 */
#define DWC_NUM_CHANNELS 8

/**
 * Maximum packet size of any USB endpoint.  1024 is the maximum allowed by USB
 * 2.0.  Most endpoints will provide maximum packet sizes much smaller than
 * this.
 */
#define USB2_MAX_PACKET_SIZE 1024

/***************************************************************************}
{					      PRIVATE INTERNAL VARIABLES	                    }
****************************************************************************/

/* Aligned buffers for DMA which need to also be multiple of 4 bytes */
/* Fortunately max packet size under USB2 is 1024 so that is a given */
// Aligning to cache line size so we can flush/invalidate with impunity.
alignas(64) static uint8_t aligned_bufs[DWC_NUM_CHANNELS][USB2_MAX_PACKET_SIZE];


bool PhyInitialised = false;


/***************************************************************************}
{						 PRIVATE INTERNAL VARIABLES			                }
****************************************************************************/

/** Bitmap of channel free (1) or in-use (0) statuses.  */
static uint32_t chfree = 0;

/***************************************************************************}
{						 PRIVATE INTERNAL FUNCTIONS						    }
****************************************************************************/

/*-[INTERNAL: first_set_bit ]------------------------------------------------
. Find index of first set bit in a nonzero uint32_t
.--------------------------------------------------------------------------*/
static inline unsigned int first_set_bit (uint32_t word)
{
    return (31 - __builtin_clz(word));
}

/*-[INTERNAL: dwc_get_free_channel ]-----------------------------------------
. Finds and reserves an unused DWC USB host channel. This is blocking and
. will wait until a channel is available if all in use.
. RETURN: Index of the free channel
.--------------------------------------------------------------------------*/
unsigned int dwc_get_free_channel(void)
{
    unsigned int chan;
    //wait(chfree_sema);
    //ENTER_KERNEL_CRITICAL_SECTION();
    chan = first_set_bit(chfree);
    chfree &= ~((uint32_t)1 << chan);
    //EXIT_KERNEL_CRITICAL_SECTION();
    return chan;
}

/*-[INTERNAL: dwc_release_channel ]-----------------------------------------
. Releases the given DWC USB host channel that was in use and marks as free.
.--------------------------------------------------------------------------*/
void dwc_release_channel(unsigned int chan)
{
    //ENTER_KERNEL_CRITICAL_SECTION();
    chfree |= ((uint32_t)1 << chan);
    //EXIT_KERNEL_CRITICAL_SECTION();
    //signal(chfree_sema);
}

/*==========================================================================}
{			    INTERNAL FAKE ROOT HUB MESSAGE HANDLER FUNCTIONS		    }
{==========================================================================*/

void DwcClearEnable()
{
    auto tempPort = *DWC_HOST_PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Enable = true;
    DWC_HOST_PORT = tempPort;
}

void DwcResume()
{
    DWC_POWER_AND_CLOCK = 0;
    Cpu::DelayInMicroseconds(5000);
    auto tempPort = *DWC_HOST_PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Resume = true;
    DWC_HOST_PORT = tempPort;
    Cpu::DelayInMicroseconds(100000);
    tempPort = *DWC_HOST_PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Suspend = false;
    tempPort.Resume = false;
    DWC_HOST_PORT = tempPort;
}

void DwcPowerOff()
{
    auto tempPort = *DWC_HOST_PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Power = false;
    DWC_HOST_PORT = tempPort;
}

void DwcConnectionChange()
{
    auto tempPort = *DWC_HOST_PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.ConnectChanged = true;
    DWC_HOST_PORT = tempPort;
}

void DwcEnableChange()
{
    auto tempPort = *DWC_HOST_PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.EnableChanged = true;
    DWC_HOST_PORT = tempPort;
}

void DwcOverCurrentChange()
{
    auto tempPort = *DWC_HOST_PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.OverCurrentChanged = true;
    DWC_HOST_PORT = tempPort;
}

void DwcReset()
{
    auto tempPower = *DWC_POWER_AND_CLOCK;
    tempPower.EnableSleepClockGating = false;
    tempPower.StopPClock = false;
    DWC_POWER_AND_CLOCK = tempPower;
    Cpu::DelayInMicroseconds(10000);
    DWC_POWER_AND_CLOCK = 0;

    auto tempPort = *DWC_HOST_PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Suspend = false;
    tempPort.Reset = true;
    tempPort.Power = true;
    DWC_HOST_PORT = tempPort;
    Cpu::DelayInMicroseconds(60000);
    tempPort = *DWC_HOST_PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Reset = false;
    DWC_HOST_PORT = tempPort;
}

void DwcPowerOn()
{
    auto tempPort = *DWC_HOST_PORT;
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Power = true;
    DWC_HOST_PORT = tempPort;
}

HubPortFullStatus DwcGetPortStatus()
{
    auto tempPort = *DWC_HOST_PORT;
    LOG_DEBUG("DwcGetPortStatus: Port 1 status: 0x%08x %032b\n", tempPort.Raw32, tempPort.Raw32);
    HubPortFullStatus replyPort{};
    replyPort.Status.Connected = tempPort.Connect;
    replyPort.Status.Enabled = tempPort.Enable;
    replyPort.Status.Suspended = tempPort.Suspend;
    replyPort.Status.OverCurrent = tempPort.OverCurrent;
    replyPort.Status.Reset = tempPort.Reset;
    replyPort.Status.Power = tempPort.Power;
    if (tempPort.Speed == USB_SPEED_HIGH)
        replyPort.Status.HighSpeedAttatched = true;
    else if (tempPort.Speed == USB_SPEED_LOW)
        replyPort.Status.LowSpeedAttatched = true;
    replyPort.Status.TestMode = tempPort.TestControl;
    replyPort.Change.ConnectedChanged = tempPort.ConnectChanged;
    replyPort.Change.EnabledChanged = false;
    replyPort.Change.OverCurrentChanged = tempPort.OverCurrentChanged;
    replyPort.Change.ResetChanged = false;
    return replyPort;
}

/*==========================================================================}
{					   INTERNAL HOST CONTROL FUNCTIONS					    }
{==========================================================================*/

/*-INTERNAL: PowerOnUsb------------------------------------------------------
 Uses PI mailbox to turn power onto USB see website about command 0x28001
 https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
 11Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT PowerOnUsb(void) {
    uint32_t __attribute__((aligned(16))) volatile mailbox_message_buffer[8];
    auto mailbox_message = Mailbox::AsGpuPointer(mailbox_message_buffer);
    mailbox_message[0] = sizeof(mailbox_message);
    mailbox_message[1] = 0;
    mailbox_message[2] = (uint32_t)Mailbox::Tag::SET_POWER_STATE;
    mailbox_message[3] = 8;
    mailbox_message[4] = 8;
    mailbox_message[5] = 0x3;
    mailbox_message[6] = 0x1;
    mailbox_message[7] = 0x0;

    if (Mailbox::SendTags(std::span{ mailbox_message, 8 }) && (mailbox_message[4] == 0x80000008)) {
        return DWCRESULT::Ok;
    }
    return DWCRESULT::ErrorDevice;
}

/*-INTERNAL: PowerOffUsb-----------------------------------------------------
 Uses PI mailbox to turn power onto USB see website about command 0x28001
 https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
 11Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT PowerOffUsb(void) {
    uint32_t __attribute__((aligned(16))) volatile mailbox_message_buffer[8];
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

    uint64_t ticks100ms = Cpu::GetPerformanceTicksForUs(100'000);
    uint64_t original_tick = Cpu::GetPerformanceCounter();
    do {
        if (Cpu::GetPerformanceCounter() - original_tick > ticks100ms) {
            return DWCRESULT::ErrorTimeout;
        }
    } while ((*DWC_CORE_RESET).AhbMasterIdle == false);

    DWC_CORE_RESET = [](auto& r){ r.CoreSoft = true; };

    struct CoreReset temp;
    original_tick = Cpu::GetPerformanceCounter();
    do {
        if (Cpu::GetPerformanceCounter() - original_tick > ticks100ms) {
            return DWCRESULT::ErrorTimeout;
        }
        temp = *DWC_CORE_RESET;
    } while (temp.CoreSoft == true || temp.AhbMasterIdle == false);

    return DWCRESULT::Ok;
}

/*-INTERNAL: HCDTransmitFifoFlush-------------------------------------------
 Flushes TX fifo buffers again uses ARM timer tick to timeout if neccessary.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT HCDTransmitFifoFlush(enum CoreFifoFlush fifo) {

    DWC_CORE_RESET = [=](auto& r){ r.TransmitFifoFlushNumber = fifo; };
    DWC_CORE_RESET = [](auto& r){ r.TransmitFifoFlush = true; };

    uint64_t ticks100ms = Cpu::GetPerformanceTicksForUs(100'000);
    uint64_t original_tick = Cpu::GetPerformanceCounter();
    do {
        if (Cpu::GetPerformanceCounter() - original_tick > ticks100ms) {
            return DWCRESULT::ErrorTimeout;
        }
    } while ((*DWC_CORE_RESET).TransmitFifoFlush == true);

    return DWCRESULT::Ok;
}

/*-INTERNAL: HCDReceiveFifoFlush---------------------------------------------
 Flushes RX fifo buffers again uses ARM timer tick to timeout if neccessary.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT HCDReceiveFifoFlush(void) {

    DWC_CORE_RESET = [](auto& r){ r.ReceiveFifoFlush = true; };

    uint64_t ticks100ms = Cpu::GetPerformanceTicksForUs(100'000);
    uint64_t original_tick = Cpu::GetPerformanceCounter();
    do {
        if (Cpu::GetPerformanceCounter() - original_tick > ticks100ms) {
            return DWCRESULT::ErrorTimeout;
        }
    } while ((*DWC_CORE_RESET).ReceiveFifoFlush == true);

    return DWCRESULT::Ok;
}

void HandleOtgInterrupt()
{
    CoreOtgInterrupt otg = DWC_CORE_OTGINTERRUPT;

    Uart::Raw::Puts("OTG Interrupt: ");
    Uart::Raw::PutBin(otg.Raw32);
    Uart::Raw::Puts("\n");

    if (otg.SessionEndDetected                ) { Uart::Raw::Puts("SessionEndDetected"                );  }
    if (otg.SessionRequestSuccessStatusChange ) { Uart::Raw::Puts("SessionRequestSuccessStatusChange" );  }
    if (otg.HostNegotiationSuccessStatusChange) { Uart::Raw::Puts("HostNegotiationSuccessStatusChange");  }
    if (otg.HostNegotiationDetected           ) { Uart::Raw::Puts("HostNegotiationDetected"           );  }
    if (otg.ADeviceTimeoutChange              ) { Uart::Raw::Puts("ADeviceTimeoutChange"              );  }
    if (otg.DebounceDone                      ) { Uart::Raw::Puts("DebounceDone"                      );  }

    DWC_CORE_OTGINTERRUPT = otg;
}

void HandlePortInterrupt()
{
    HostPort port = DWC_HOST_PORT;
    Uart::Raw::Puts("DWC_HOST_PORT: ");
    Uart::Raw::PutBin(port.Raw32);
    Uart::Raw::Puts("\n");

    if (port.ConnectChanged    ) { Uart::Raw::Puts("ConnectChanged    : "); Uart::Raw::PutDec(port.Connect    ); Uart::Raw::Puts("\n"); }
    if (port.EnableChanged     ) { Uart::Raw::Puts("EnableChanged     : "); Uart::Raw::PutDec(port.Enable     ); Uart::Raw::Puts("\n"); }
    if (port.OverCurrentChanged) { Uart::Raw::Puts("OverCurrentChanged: "); Uart::Raw::PutDec(port.OverCurrent); Uart::Raw::Puts("\n"); }

    port.Enable = false;

    DWC_HOST_PORT = port;
}

void InterruptHandler()
{
    // See what's what.
    CoreInterrupts interrupts = DWC_CORE_INTERRUPT;
    CoreInterrupts const mask = DWC_CORE_INTERRUPTMASK;

    Uart::Raw::Puts("DWC_CORE_INTERRUPT    : 0x");
    Uart::Raw::PutBin(DWC_CORE_INTERRUPT.get().Raw32);
    Uart::Raw::Puts("\n");

    Uart::Raw::Puts("DWC_CORE_INTERRUPTMASK: 0x");
    Uart::Raw::PutBin(DWC_CORE_INTERRUPTMASK.get().Raw32);
    Uart::Raw::Puts("\n");

    interrupts.Raw32 &= mask.Raw32; // Mask out the interrupts we are not interested in.

    if (interrupts.CurrentMode                 ) { Uart::Raw::Puts("CurrentMode\n"                 ); DWC_CORE_INTERRUPT = CoreInterrupts{ .CurrentMode                  = true }; }
    if (interrupts.ModeMismatch                ) { Uart::Raw::Puts("ModeMismatch\n"                ); DWC_CORE_INTERRUPT = CoreInterrupts{ .ModeMismatch                 = true }; }
    if (interrupts.Otg                         ) { HandleOtgInterrupt(); }
    if (interrupts.DmaStartOfFrame             ) { Uart::Raw::Puts("DmaStartOfFrame\n"             ); DWC_CORE_INTERRUPT = CoreInterrupts{ .DmaStartOfFrame              = true }; }
    if (interrupts.ReceiveStatusLevel          ) { Uart::Raw::Puts("ReceiveStatusLevel\n"          ); DWC_CORE_INTERRUPT = CoreInterrupts{ .ReceiveStatusLevel           = true }; }
    if (interrupts.NpTransmitFifoEmpty         ) { Uart::Raw::Puts("NpTransmitFifoEmpty\n"         ); DWC_CORE_INTERRUPT = CoreInterrupts{ .NpTransmitFifoEmpty          = true }; }
    if (interrupts.ginnakeff                   ) { Uart::Raw::Puts("ginnakeff\n"                   ); DWC_CORE_INTERRUPT = CoreInterrupts{ .ginnakeff                    = true }; }
    if (interrupts.goutnakeff                  ) { Uart::Raw::Puts("goutnakeff\n"                  ); DWC_CORE_INTERRUPT = CoreInterrupts{ .goutnakeff                   = true }; }
    if (interrupts.ulpick                      ) { Uart::Raw::Puts("ulpick\n"                      ); DWC_CORE_INTERRUPT = CoreInterrupts{ .ulpick                       = true }; }
    if (interrupts.I2c                         ) { Uart::Raw::Puts("I2c\n"                         ); DWC_CORE_INTERRUPT = CoreInterrupts{ .I2c                          = true }; }
    if (interrupts.EarlySuspend                ) { Uart::Raw::Puts("EarlySuspend\n"                ); DWC_CORE_INTERRUPT = CoreInterrupts{ .EarlySuspend                 = true }; }
    if (interrupts.UsbSuspend                  ) { Uart::Raw::Puts("UsbSuspend\n"                  ); DWC_CORE_INTERRUPT = CoreInterrupts{ .UsbSuspend                   = true }; }
    if (interrupts.UsbReset                    ) { Uart::Raw::Puts("UsbReset\n"                    ); DWC_CORE_INTERRUPT = CoreInterrupts{ .UsbReset                     = true }; }
    if (interrupts.EnumerationDone             ) { Uart::Raw::Puts("EnumerationDone\n"             ); DWC_CORE_INTERRUPT = CoreInterrupts{ .EnumerationDone              = true }; }
    if (interrupts.IsochronousOutDrop          ) { Uart::Raw::Puts("IsochronousOutDrop\n"          ); DWC_CORE_INTERRUPT = CoreInterrupts{ .IsochronousOutDrop           = true }; }
    if (interrupts.eopframe                    ) { Uart::Raw::Puts("eopframe\n"                    ); DWC_CORE_INTERRUPT = CoreInterrupts{ .eopframe                     = true }; }
    if (interrupts.RestoreDone                 ) { Uart::Raw::Puts("RestoreDone\n"                 ); DWC_CORE_INTERRUPT = CoreInterrupts{ .RestoreDone                  = true }; }
    if (interrupts.EndPointMismatch            ) { Uart::Raw::Puts("EndPointMismatch\n"            ); DWC_CORE_INTERRUPT = CoreInterrupts{ .EndPointMismatch             = true }; }
    if (interrupts.InEndPoint                  ) { Uart::Raw::Puts("InEndPoint\n"                  ); DWC_CORE_INTERRUPT = CoreInterrupts{ .InEndPoint                   = true }; }
    if (interrupts.OutEndPoint                 ) { Uart::Raw::Puts("OutEndPoint\n"                 ); DWC_CORE_INTERRUPT = CoreInterrupts{ .OutEndPoint                  = true }; }
    if (interrupts.IncompleteIsochronousIn     ) { Uart::Raw::Puts("IncompleteIsochronousIn\n"     ); DWC_CORE_INTERRUPT = CoreInterrupts{ .IncompleteIsochronousIn      = true }; }
    if (interrupts.IncompleteIsochronousOut    ) { Uart::Raw::Puts("IncompleteIsochronousOut\n"    ); DWC_CORE_INTERRUPT = CoreInterrupts{ .IncompleteIsochronousOut     = true }; }
    if (interrupts.fetsetup                    ) { Uart::Raw::Puts("fetsetup\n"                    ); DWC_CORE_INTERRUPT = CoreInterrupts{ .fetsetup                     = true }; }
    if (interrupts.ResetDetect                 ) { Uart::Raw::Puts("ResetDetect\n"                 ); DWC_CORE_INTERRUPT = CoreInterrupts{ .ResetDetect                  = true }; }
    if (interrupts.Port                        ) { HandlePortInterrupt(); }
    if (interrupts.HostChannel                 ) { Uart::Raw::Puts("HostChannel\n"                 ); DWC_CORE_INTERRUPT = CoreInterrupts{ .HostChannel                  = true }; }
    if (interrupts.HpTransmitFifoEmpty         ) { Uart::Raw::Puts("HpTransmitFifoEmpty\n"         ); DWC_CORE_INTERRUPT = CoreInterrupts{ .HpTransmitFifoEmpty          = true }; }
    if (interrupts.LowPowerModeTransmitReceived) { Uart::Raw::Puts("LowPowerModeTransmitReceived\n"); DWC_CORE_INTERRUPT = CoreInterrupts{ .LowPowerModeTransmitReceived = true }; }
    if (interrupts.ConnectionIdStatusChange    ) { Uart::Raw::Puts("ConnectionIdStatusChange\n"    ); DWC_CORE_INTERRUPT = CoreInterrupts{ .ConnectionIdStatusChange     = true }; }
    if (interrupts.Disconnect                  ) { Uart::Raw::Puts("Disconnect\n"                  ); DWC_CORE_INTERRUPT = CoreInterrupts{ .Disconnect                   = true }; }
    if (interrupts.SessionRequest              ) { Uart::Raw::Puts("SessionRequest\n"              ); DWC_CORE_INTERRUPT = CoreInterrupts{ .SessionRequest               = true }; }
    if (interrupts.Wakeup                      ) { Uart::Raw::Puts("Wakeup\n"                      ); DWC_CORE_INTERRUPT = CoreInterrupts{ .Wakeup                       = true }; }
}

/*-INTERNAL: HCDStart--------------------------------------------------------
 Starts the HCD system once completed this routiune the system is operational.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
/* BackGround:  ULPI was developed by a group of USB industry leaders to   */
/* address the need for low - cost USB and OTG. Existing specifications    */
/* including UTMI and UTMI + were developed primarily for Macrocell(IP)    */
/* development, and are not optimized for use as an external PHY.          */
/* Using the existing UTMI + specification as a starting point, the ULPI   */
/* working group reduced the number of interface signals to 12 pins, with  */
/* an optional implementation of 8 pins.The package size of PHY and Link   */
/* IC's are drastically reduced. This not only lowers the cost of Link and */
/* PHY IC's, but also makes for a smaller PCB.							   */
/*-------------------------------------------------------------------------*/
DWCRESULT HCDStart (void) {
    DWCRESULT result;
    struct UsbControl coreUsb;

    coreUsb = *DWC_CORE_CONTROL;
    coreUsb.UlpiDriveExternalVbus = 0;
    coreUsb.TsDlinePulseEnable = 0;
    DWC_CORE_CONTROL = coreUsb;

    Cpu::DelayInMicroseconds(1000);

    LOG_DEBUG("HCD: Master reset.\n");								
    if ((result = HCDReset()) != DWCRESULT::Ok) {
        LOG("FATAL ERROR: Could not do a Master reset on HCD.\n");
        return result;
    }

    Cpu::DelayInMicroseconds(1000);

    if (!PhyInitialised) {
        LOG_DEBUG("HCD: One time phy initialisation.\n");
        PhyInitialised = true;
        coreUsb = *DWC_CORE_CONTROL;
        coreUsb.ModeSelect = UTMI;
        LOG_DEBUG("HCD: Interface: UTMI+.\n");						
        coreUsb.PhyInterface = false;
        DWC_CORE_CONTROL = coreUsb;
        if ((result = HCDReset()) != DWCRESULT::Ok) {
            LOG("FATAL ERROR: Could not do a Master reset on HCD.\n");
            return result;
        }
    }

    Cpu::DelayInMicroseconds(1000);

    coreUsb = *DWC_CORE_CONTROL;
    if ((*DWC_CORE_HARDWARE1).HighSpeedPhysical == Ulpi
        && (*DWC_CORE_HARDWARE1).FullSpeedPhysical == Dedicated) {
        LOG_DEBUG("HCD: ULPI FSLS configuration: enabled.\n");	
        coreUsb.UlpiFsls = true;								
        coreUsb.ulpi_clk_sus_m = true;
    } else {
        LOG_DEBUG("HCD: ULPI FSLS configuration: disabled.\n");
        coreUsb.UlpiFsls = false;
        coreUsb.ulpi_clk_sus_m = false;
    }
    DWC_CORE_CONTROL = coreUsb;

    Cpu::DelayInMicroseconds(1000);

    struct CoreAhb tempAhb;
    tempAhb = *DWC_CORE_AHB;
    tempAhb.DmaEnable = true;
    tempAhb.DmaRemainderMode = Incremental;
    DWC_CORE_AHB = tempAhb;

    Cpu::DelayInMicroseconds(1000);

    coreUsb = *DWC_CORE_CONTROL;
    switch ((*DWC_CORE_HARDWARE1).OperatingMode) {
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
    DWC_CORE_CONTROL = coreUsb;
    LOG_DEBUG("HCD: Core started.\n");
    LOG_DEBUG("HCD: Starting host.\n");

    Cpu::DelayInMicroseconds(1000);

    DWC_POWER_AND_CLOCK = {};

    if ((*DWC_CORE_HARDWARE1).HighSpeedPhysical == Ulpi
        && (*DWC_CORE_HARDWARE1).FullSpeedPhysical == Dedicated
        && coreUsb.UlpiFsls) {
        LOG_DEBUG("HCD: Host clock: 48Mhz.\n");
        DWC_HOST_CONFIG = [](auto& r) { r.ClockRate = Clock48MHz; };
    } else {
        LOG_DEBUG("HCD: Host clock: 30-60Mhz.\n");
        DWC_HOST_CONFIG = [](auto& r) { r.ClockRate = Clock30_60MHz; };
    }

    // ULPI FsLs Host mode, I assume other mode is ULPI only  .. documentation would be nice
    DWC_HOST_CONFIG = [](auto& r){ r.FslsOnly = true; };

    Cpu::DelayInMicroseconds(1000);

    DWC_CORE_RECEIVESIZE = ReceiveFifoSize;

    Cpu::DelayInMicroseconds(1000);

    DWC_CORE_NONPERIODICFIFO_SIZE = [](auto& r)
    {
        r.Depth        = NonPeriodicFifoSize;
        r.StartAddress = ReceiveFifoSize;
    };

    Cpu::DelayInMicroseconds(1000);

    DWC_CORE_PERIODICINFO_HostSize = [](auto& r)
    {
        r.Depth        = PeriodicFifoSize;
        r.StartAddress = ReceiveFifoSize + NonPeriodicFifoSize;
    };

    Cpu::DelayInMicroseconds(1000);

    LOG_DEBUG("HCD: Set HNP: enabled.\n");

    struct CoreOtgControl tempOtgControl;
    tempOtgControl = *DWC_CORE_OTGCONTROL;
    tempOtgControl.HostSetHnpEnable = true;
    DWC_CORE_OTGCONTROL = tempOtgControl;
    //DWC_CORE_OTGINTERRUPT = 0xFFFF'FFFFu; // Clear all OTG interrupts

    Cpu::DelayInMicroseconds(1000);

    if ((result = HCDTransmitFifoFlush(FlushAll)) != DWCRESULT::Ok)
        return result;
    Cpu::DelayInMicroseconds(1000);

    if ((result = HCDReceiveFifoFlush()) != DWCRESULT::Ok)
        return result;
    Cpu::DelayInMicroseconds(1000);


    printf2("DWC_HOST_CONFIG: 0x%08X\n", (*DWC_HOST_CONFIG).Raw32);
    printf2("DWC_CORE_HARDWARE1: 0x%08X %b\n", (*DWC_CORE_HARDWARE1), (*DWC_CORE_HARDWARE1));
    for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
        printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, (*DWC_HOST_CHANNEL_Characteristic[channel]).Raw32);
    }
    printf2("\n");

    if (!(*DWC_HOST_CONFIG).EnableDmaDescriptor) {
/*
        for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
            struct HostChannelCharacteristic tempChar;
            tempChar = *DWC_HOST_CHANNEL_Characteristic[channel];
            printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, tempChar.Raw32);
            tempChar.channel_enable = false;
            tempChar.channel_disable = true;
            tempChar.endpoint_direction = USB_DIRECTION_IN;
            DWC_HOST_CHANNEL_Characteristic[channel] = tempChar;
        }
        printf2("\n");

        Cpu::DelayInMicroseconds(100'000);

        //for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
        //	struct HostChannelCharacteristic tempChar;
        //	uint64_t ticks100ms = Cpu::GetPerformanceTicksForUs(100'000);
        //	uint64_t original_tick = Cpu::GetPerformanceCounter();
        //	do {
        //		tempChar = *DWC_HOST_CHANNEL_Characteristic[channel];
        //		if (Cpu::GetPerformanceCounter() - original_tick > ticks100ms) {
        //			LOG("HCD: Unable to set halt on channel %i %X.\n", channel, tempChar.Raw32);
        //			original_tick = Cpu::GetPerformanceCounter();
        //			//Processor::Halt();
        //		}
        //	} while (!tempChar.channel_disable || !tempChar.channel_enable);
        //}
        for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
            printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, (*DWC_HOST_CHANNEL_Characteristic[channel]).Raw32);
        }
        printf2("\n");
*/

        for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
            struct HostChannelCharacteristic tempChar;
            tempChar = *DWC_HOST_CHANNEL_Characteristic[channel];
            printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, tempChar.Raw32);
            tempChar.channel_enable = true;
            tempChar.channel_disable = false;
            tempChar.endpoint_direction = USB_DIRECTION_IN;
            DWC_HOST_CHANNEL_Characteristic[channel] = tempChar;
        }
        printf2("\n");

        Cpu::DelayInMicroseconds(100'000);

        for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
            printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, (*DWC_HOST_CHANNEL_Characteristic[channel]).Raw32);
        }

        for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
            struct HostChannelCharacteristic tempChar;
            uint64_t ticks100ms = Cpu::GetPerformanceTicksForUs(100'000);
            uint64_t original_tick = Cpu::GetPerformanceCounter();
            do {
                tempChar = *DWC_HOST_CHANNEL_Characteristic[channel];
                if (Cpu::GetPerformanceCounter() - original_tick > ticks100ms) {
                    LOG("HCD: Unable to clear halt on channel %i %X.\n", channel, tempChar.Raw32);
                    original_tick = Cpu::GetPerformanceCounter();
                    //Processor::Halt();
                }
            } while (tempChar.channel_disable || !tempChar.channel_enable);
        }
        printf2("\n");
        for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
            printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, (*DWC_HOST_CHANNEL_Characteristic[channel]).Raw32);
        }
    }

    Cpu::DelayInMicroseconds(1000);

    struct HostPort tempPort;
    tempPort = *DWC_HOST_PORT;
    LOG_DEBUG("HCD: Initial host port: 0x%08X\n", tempPort.Raw32);
    if (!tempPort.Power) {
        LOG_DEBUG("HCD: Initial power physical host up.\n");
        tempPort.Raw32 &= HOSTPORTMASK;
        tempPort.Power = true;
        DWC_HOST_PORT = tempPort;
    }

    Cpu::DelayInMicroseconds(1000);

    LOG_DEBUG("HCD: Initial resetting physical host.\n");
    tempPort = *DWC_HOST_PORT;
    LOG_DEBUG("HCD: Powered host port: 0x%08X\n", tempPort.Raw32);
    
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Reset = true;
    DWC_HOST_PORT = tempPort;
    Cpu::DelayInMicroseconds(60000);
    tempPort = *DWC_HOST_PORT;
    LOG_DEBUG("HCD: Reset host port: 0x%08X\n", tempPort.Raw32);
    
    tempPort.Raw32 &= HOSTPORTMASK;
    tempPort.Reset = false;
    DWC_HOST_PORT = tempPort;
    
    Cpu::DelayInMicroseconds(1000);

    LOG_DEBUG("HCD: Reset host port: 0x%08X\n", tempPort.Raw32);

    Interrupts::EnableUsb(InterruptHandler);

    DWC_CORE_AHB = [](auto& ahb){ ahb.InterruptEnable = true; };
    CoreInterrupts mask{
        .ReceiveStatusLevel = true,
        .EnumerationDone = true,
        .InEndPoint = true,
        .OutEndPoint = true,
        //.Port = true, // Port status changes (connections, disconnections). Handling them prevents the work of the initial enumeration. TODO: Fix this.
        .HostChannel = true,
        .ConnectionIdStatusChange = true,
        .Disconnect = true,
        .SessionRequest = true,
    };
    DWC_CORE_INTERRUPT = 0xFFFFFFFF; // Setting bits clears interrupts
    DWC_CORE_INTERRUPTMASK = mask;
    
    LOG_DEBUG("HCD: Successfully started.\n");

    return DWCRESULT::Ok;
}


/*==========================================================================}
{				   INTERNAL HOST TRANSMISSION ROUTINES					    }
{==========================================================================*/

/*-INTERNAL: HCDCheckErrorAndAction -----------------------------------------
 Given a channel interrupt flags and whether packet was complete (not split)
 it will set sendControl structure with what to do next.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT HCDCheckErrorAndAction(struct ChannelInterrupts interrupts, bool packetSplit, struct UsbSendControl* sendCtrl) {
    sendCtrl->ActionResendSplit = false;
    sendCtrl->ActionRetry = false;
    sendCtrl->LongerDelay = false;
    // First deal with all the fatal errors .. no use dealing with trivial errors if these are set
    if (interrupts.AhbError) {
        sendCtrl->ActionFatalError = true;
        return DWCRESULT::ErrorDevice;
    }
    if (interrupts.DataToggleError) {
        sendCtrl->ActionFatalError = true;
        return DWCRESULT::ErrorTransmission;
    }
    // Next deal with the fully successful case
    if (interrupts.Acknowledgement) {
        if (interrupts.TransferComplete) sendCtrl->Success = true;
            else sendCtrl->ActionResendSplit = true;
        sendCtrl->GlobalTries = 0;
        return DWCRESULT::Ok;
    }
    // Everything else is minor error invoking a retry .. so first update counts
    if (packetSplit) {
        sendCtrl->SplitTries++;
        if (sendCtrl->SplitTries == 5) {
            // Ridiculous number of split resends reached .. fatal error
            sendCtrl->ActionFatalError = true;
            return DWCRESULT::ErrorTransmission;
        }
        sendCtrl->ActionResendSplit = true;
    } else {
        sendCtrl->PacketTries++;
        if (sendCtrl->PacketTries == 3) {
            // Ridiculous number of packet resends reached .. fatal error
            sendCtrl->ActionFatalError = true;
            return DWCRESULT::ErrorTransmission;
        }
        sendCtrl->ActionRetry = true;
    }
    // Check no transmission errors and if so deal with minor cases
    if (!interrupts.Stall && !interrupts.BabbleError &&
        !interrupts.FrameOverrun) {
        // If endpoint NAK nothing wrong just demanding a retry
        if (interrupts.NegativeAcknowledgement)
            return DWCRESULT::ErrorTransmission;
        if (interrupts.NotYet)
        {
            // Device is not yet ready for this
            // Note: This seems to be exactly what we need to do.
            // But the caller was putting on a 10 ms wait? :-P
            //sendCtrl->LongerDelay = true;
            return DWCRESULT::ErrorTransmission;
        }
        return DWCRESULT::ErrorTimeout;
    }
    // Everything else updates global count as it is serious
    sendCtrl->GlobalTries++;
    if (sendCtrl->GlobalTries == 3) {
        sendCtrl->ActionRetry = false;
        sendCtrl->ActionResendSplit = false;
        sendCtrl->ActionFatalError = true;
        return DWCRESULT::ErrorTransmission;
    }
    // Stall is usually recoverable with a wait and retry.
    if (interrupts.Stall)
    {
        return DWCRESULT::ErrorStall;
    }
    // Deal with true transmission errors
    if ((interrupts.BabbleError) ||
        (interrupts.FrameOverrun) ||
        (interrupts.TransactionError))
    {
        return DWCRESULT::ErrorTransmission;
    }
    return DWCRESULT::ErrorGeneral;
}

/*-INTERNAL: HCDWaitOnTransmissionResult------------------------------------
 When not using Interrupts, Timers or OS this is the good old polling wait
 around for transmission packet sucess or timeout. HCD supports multiple
 options on sending the packets this static polled is just one way.
 19Feb17 LdB
 --------------------------------------------------------------------------*/
ChannelInterrupts HCDWaitOnTransmissionResult(uint32_t timeout, uint8_t channel)
{
    ChannelInterrupts tempInt;
    uint64_t ticksTimeout = Cpu::GetPerformanceTicksForUs(timeout);
    uint64_t original_tick = Cpu::GetPerformanceCounter();
    for (;;) {
        Cpu::DelayInMicroseconds(100);
        tempInt = *DWC_HOST_CHANNEL_Interrupt[channel];
        if (tempInt.Halt || Cpu::GetPerformanceCounter() - original_tick > ticksTimeout)
        {
            return tempInt;
        }
    }
}

using InCallback = bool (*)(uintptr_t context, uint32_t channel);

struct Channel
{
    bool              Prepared;
    bool              InTransfer;
    bool              OutTransfer;
    bool              SplitEnabled;
    uint32_t          Size;
    UsbPipe           Pipe;
    usb_transfer_type Type;
    UsbDirection      Direction;
    InCallback        Callback;
    uintptr_t         Context;
};

Channel ChannelData[DWC_NUM_CHANNELS]{};

void HCDPrepareChannel(
    UsbPipe const&    pipe, // Endpoint information
    uint32_t          channel,
    usb_transfer_type Type,
    UsbDirection      Direction,
    PacketId          packetId,
    uint32_t          transferSize,
    InCallback        callback, // Callback to call when transfer is complete
    uintptr_t         context
)
{
    LOG_DEBUG("HCD: Channel %u %s transfer, length %d, packetId %d, address %u, endpoint %u, type %u, speed %u\n",
        channel, Direction == USB_DIRECTION_IN ? "in" : "out", pipe.Number, pipe.EndPoint, Type, pipe.Speed);

    uint32_t offset = 0;
    if (channel > (*DWC_CORE_HARDWARE1).HostChannelCount) {
        LOG("HCD: Channel %d is not available on this host.\n", channel);
        return;
    }

    // Program the channel.
    DWC_HOST_CHANNEL_Interrupt[channel] = 0xFFFFFFFF;
    DWC_HOST_CHANNEL_InterruptMask[channel] = 0x0;

    struct HostChannelCharacteristic tempChar = { 0 };
    tempChar.device_address = pipe.Number;
    tempChar.endpoint_number = pipe.EndPoint;
    tempChar.endpoint_direction = Direction;
    tempChar.low_speed = pipe.Speed == USB_SPEED_LOW ? true : false;
    tempChar.endpoint_type = Type;
    tempChar.max_packet_size = pipe.MaxPacketSizeInBits;
    tempChar.channel_enable = false;
    tempChar.channel_disable = false;
    DWC_HOST_CHANNEL_Characteristic[channel] = tempChar;

    // Clear and setup split control to low speed devices
    struct HostChannelSplitControl tempSplit = { 0 };
    if (pipe.Speed != USB_SPEED_HIGH) {
        LOG_DEBUG("Setting split control, addr: %i port: %i, packetSize: PacketSize: %u\n",
            pipe.splitNodePoint, pipe.splitNodePort, pipe.MaxPacketSizeInBits);
        tempSplit.split_enable = true;
        tempSplit.hub_address = pipe.splitNodePoint;
        tempSplit.port_address = pipe.splitNodePort;
        tempSplit.transaction_position = 0;//3;
    }
    DWC_HOST_CHANNEL_SplitCtrl[channel] = tempSplit;

    // Set transfer size
    HostTransferSize tempXfer{};
    tempXfer.size = transferSize;
    if (pipe.Speed == USB_SPEED_LOW) tempXfer.packet_count = (transferSize + 7) / 8;
    else                             tempXfer.packet_count = (transferSize + pipe.MaxPacketSizeInBits - 1) / pipe.MaxPacketSizeInBits;
    if (tempXfer.packet_count == 0) tempXfer.packet_count = 1;
    tempXfer.packet_id = packetId;
    DWC_HOST_CHANNEL_TransferSize[channel] = tempXfer;

    ChannelData[channel].Prepared = true; // Mark channel as prepared
    ChannelData[channel].InTransfer = false;
    ChannelData[channel].OutTransfer = false;
    ChannelData[channel].SplitEnabled = (pipe.Speed != USB_SPEED_HIGH);
    ChannelData[channel].Size = transferSize;
    ChannelData[channel].Pipe = pipe;
    ChannelData[channel].Type = Type;
    ChannelData[channel].Direction = Direction;
    ChannelData[channel].Callback = nullptr;
    ChannelData[channel].Context = 0;
}

void HCDStartInTransfer(
    uint32_t channel
)
{
    ChannelData[channel].InTransfer = true;

    LOG_DEBUG("HCD: Channel %u transfer size set to %#08X bytes.\n", pipectrl.Channel, tempXfer.Raw32);

    // Clear any left over channel interrupts
    DWC_HOST_CHANNEL_Interrupt[channel] = 0xFFFFFFFF;
    DWC_HOST_CHANNEL_InterruptMask[channel] = ChannelInterrupts
    {
        .TransferComplete        = true,
        .Halt                    = true,
        .Stall                   = true,
        .NegativeAcknowledgement = true,
    };

    DWC_HOST_INTERRUPTMASK = 1u << channel; // Enable channel interrupts

    // Clear any left over split
    DWC_HOST_CHANNEL_SplitCtrl[channel] = [](auto& reg){ reg.complete_split = false; };

    uint8_t* dmaBuffer  = Mailbox::AsGpuPointer(aligned_bufs[channel]);

    DWC_HOST_CHANNEL_DmaAddr[channel] = Mailbox::AsGpuAddress(dmaBuffer) | 0xC000'0000u;
    
    auto nextFrame = (*DWC_HOST_FRAMECONTROL).FrameNumber + 1;

    /* Launch transmission */
    DWC_HOST_CHANNEL_Characteristic[channel] = [nextFrame](auto& reg)
    {
        reg.channel_enable    = true;
        reg.odd_frame         = nextFrame & 1;
        reg.packets_per_frame = 1;
    };
}

void HCDHandleInTransferInterrupt(uint32_t channel)
{
    ChannelInterrupts interrupts = DWC_HOST_CHANNEL_Interrupt[channel];
    if (interrupts.TransferComplete)
    {
        HostTransferSize size = DWC_HOST_CHANNEL_TransferSize[channel];
        if (size.packet_count > 0)
        {

            LOG_DEBUG("HCD: Channel %u transfer size is zero, no data transferred.\n", channel);
            return;
        }

        LOG_DEBUG("HCD: Channel %u transfer complete.\n", channel);
        if (ChannelData[channel].Callback)
        {
            if (ChannelData[channel].Callback(ChannelData[channel].Context, channel))
            {
                LOG_DEBUG("HCD: Callback for channel %u returned true.\n", channel);
            }
            else
            {
                LOG_DEBUG("HCD: Callback for channel %u returned false.\n", channel);
            }
        }
    }
    else if (interrupts.Stall)
    {
        // Must retry later.
        LOG_DEBUG("HCD: Channel %u stalled.\n", channel);
    }
    else if (interrupts.NegativeAcknowledgement)
    {
        // Rejected by the device.
        LOG_DEBUG("HCD: Channel %u NAKed.\n", channel);
    }
    else if (interrupts.Halt)
    {
        if (ChannelData[channel].SplitEnabled)
        {
            DWC_HOST_CHANNEL_SplitCtrl[channel] = [](auto& reg)
            {
                reg.complete_split = true; // Mark split as complete
            };
        }
        else
        {
            LOG_DEBUG("HCD: Channel %u halted.\n", channel);
        }
    }
    else
    {
        LOG_DEBUG("HCD: Channel %u unknown interrupt.\n", channel);
    }
}

/*-INTERNAL: HCDChannelTransfer----------------------------------------------
 Sends/recieves data from the given buffer and size directed by pipe settings.
 19Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT HCDChannelTransfer(const struct UsbPipe pipe, const struct UsbPipeControl pipectrl, uint8_t* buffer, uint32_t& bufferLength, PacketId packetId) 
{
    LOG_DEBUG("HCD: Channel %u %s transfer, length %d, packetId %d, address %u, endpoint %u, type %u, speed %u%s",
        pipectrl.Channel, pipectrl.Direction == USB_DIRECTION_IN ? "in" : "out", bufferLength, packetId, pipe.Number, pipe.EndPoint, pipectrl.Type, pipe.Speed,
        pipectrl.Direction == USB_DIRECTION_IN ? "\n" : ", "
    );
    if (bufferLength >= 8 && pipectrl.Direction == USB_DIRECTION_OUT)
    {
        LOG_DEBUG("Data = 0x%08X'%08X\n", ((uint32_t*)buffer)[1], ((uint32_t*)buffer)[0]);
    }

    DWCRESULT result;
    ChannelInterrupts tempInt;
    UsbSendControl sendCtrl = { 0 };
    uint32_t offset = 0;
    if (pipectrl.Channel > (*DWC_CORE_HARDWARE1).HostChannelCount) {
        LOG("HCD: Channel %d is not available on this host.\n", pipectrl.Channel);
        return DWCRESULT::ErrorArgument;
    }

    // Program the channel.
    DWC_HOST_CHANNEL_Interrupt[pipectrl.Channel] = 0xFFFFFFFF;
    DWC_HOST_CHANNEL_InterruptMask[pipectrl.Channel] = 0x0;

    struct HostChannelCharacteristic tempChar = { 0 };
    tempChar.device_address = pipe.Number;
    tempChar.endpoint_number = pipe.EndPoint;
    tempChar.endpoint_direction = pipectrl.Direction;
    tempChar.low_speed = pipe.Speed == USB_SPEED_LOW ? true : false;
    tempChar.endpoint_type = pipectrl.Type;
    tempChar.max_packet_size = pipe.MaxPacketSizeInBits;
    tempChar.channel_enable = false;
    tempChar.channel_disable = false;
    DWC_HOST_CHANNEL_Characteristic[pipectrl.Channel] = tempChar;

    // Clear and setup split control to low speed devices
    struct HostChannelSplitControl tempSplit = { 0 };
    if (pipe.Speed != USB_SPEED_HIGH) {
        LOG_DEBUG("Setting split control, addr: %i port: %i, packetSize: PacketSize: %u\n",
            pipe.splitNodePoint, pipe.splitNodePort, pipe.MaxPacketSizeInBits);
        tempSplit.split_enable = true;
        tempSplit.hub_address = pipe.splitNodePoint;
        tempSplit.port_address = pipe.splitNodePort;
        tempSplit.transaction_position = 0;//3;
    }
    DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel] = tempSplit;

    // Set transfer size
    struct HostTransferSize tempXfer = { 0 };
    tempXfer.size = bufferLength;
    if (pipe.Speed == USB_SPEED_LOW) tempXfer.packet_count = (bufferLength + 7) / 8;
    else tempXfer.packet_count = (bufferLength + pipe.MaxPacketSizeInBits - 1) / pipe.MaxPacketSizeInBits;
    if (tempXfer.packet_count == 0) tempXfer.packet_count = 1;
    tempXfer.packet_id = packetId;
    DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel] = tempXfer;

    LOG_DEBUG("HCD: Channel %u transfer size set to %#08X bytes.\n", pipectrl.Channel, tempXfer.Raw32);

    sendCtrl.PacketTries = 0;
    do {

        // Clear any left over channel interrupts
        DWC_HOST_CHANNEL_Interrupt[pipectrl.Channel] = 0xFFFFFFFF;
        DWC_HOST_CHANNEL_InterruptMask[pipectrl.Channel] = 0x0;

        // Clear any left over split
        tempSplit = *DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel];
        tempSplit.complete_split = false;
        DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel] = tempSplit;

        uint8_t* dmaBuffer  = Mailbox::AsGpuPointer(aligned_bufs[pipectrl.Channel]);

        // Since our buffer is unaligned for OUT endpoints, copy the data
        // From the buffer to the aligned buffer
        if (pipectrl.Direction == USB_DIRECTION_OUT)
        {
            for (int i = 0; i < bufferLength-offset; ++i)
            {
                dmaBuffer[i] = buffer[offset + i];
            }
            //memcpy(&aligned_bufs[pipectrl.Channel], &buffer[offset], bufferLength-offset);
        }

        //Processor::FlushDataCache(dmaBuffer, bufferLength - offset);

        DWC_HOST_CHANNEL_DmaAddr[pipectrl.Channel] = Mailbox::AsGpuAddress(dmaBuffer) | 0xC000'0000u;
        
        auto nextFrame = (*DWC_HOST_FRAMECONTROL).FrameNumber + 1;

        /* Launch transmission */
        tempChar = *DWC_HOST_CHANNEL_Characteristic[pipectrl.Channel];// Read host channel characteristic
        tempChar.odd_frame = nextFrame & 1;
        tempChar.packets_per_frame = 1;								// Set 1 frame per packet
        tempChar.channel_enable = true;								// Set enable channel
        tempChar.channel_disable = false;							// Clear channel disable
        DWC_HOST_CHANNEL_Characteristic[pipectrl.Channel] = tempChar;// Write channel characteristic

        // Polling wait on transmission only option right now .. other options soon :-)
        tempInt = HCDWaitOnTransmissionResult(5000, pipectrl.Channel);
        if (!tempInt.Halt)
        {
            LOG("HCD: Request on channel %i has timed out.\n", pipectrl.Channel);// Log the error
            return DWCRESULT::ErrorTimeout;									// Return timeout error
        }
        LOG_DEBUG("HCD: Channel %u transmission result: 0x%08X\n", pipectrl.Channel, tempInt.Raw32);

        tempSplit = *DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel];	// Fetch the split details
        result = HCDCheckErrorAndAction(tempInt,
            tempSplit.split_enable, &sendCtrl);						// Check transmisson DWCRESULT and set action flags
        if (result != DWCRESULT::Ok) LOG_DEBUG("Result: %i Action: 0x%08x tempInt: 0x%08x tempSplit: 0x%08x Bytes sent: %i\n",
            result, (unsigned int)sendCtrl.Raw32, (unsigned int)tempInt.Raw32, 
            (unsigned int)tempSplit.Raw32, result != DWCRESULT::Ok ? 0 : (*DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel]).size);
        if (sendCtrl.ActionFatalError) return result;				// Fatal error occured we need to bail

        sendCtrl.SplitTries = 0;
        while (sendCtrl.ActionResendSplit) {						// Decision was made to resend split
            Cpu::DelayInMicroseconds(250);
            // Clear channel interrupts
            DWC_HOST_CHANNEL_Interrupt[pipectrl.Channel] = 0xFFFFFFFF;
            DWC_HOST_CHANNEL_InterruptMask[pipectrl.Channel] = 0x0;

            // Set we are completing the split
            tempSplit = *DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel];
            tempSplit.complete_split = true;						// Set complete split flag
            DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel] = tempSplit;

            // Launch transmission
            tempChar = *DWC_HOST_CHANNEL_Characteristic[pipectrl.Channel];
            tempChar.channel_enable = true;
            tempChar.channel_disable = false;
            DWC_HOST_CHANNEL_Characteristic[pipectrl.Channel] = tempChar;

            // Polling wait on transmission only option right now .. other options soon :-)
            tempInt = HCDWaitOnTransmissionResult(5000, pipectrl.Channel);
            if (!tempInt.Halt)
            {
                LOG("HCD: Request split completion on channel:%i has timed out.\n", pipectrl.Channel);// Log error
                return DWCRESULT::ErrorTimeout;								// Return timeout error
            }
            LOG_DEBUG("HCD: Channel %u transmission result: 0x%08X\n", pipectrl.Channel, tempInt.Raw32);

            tempSplit = *DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel];// Fetch the split details again
            result = HCDCheckErrorAndAction(tempInt,
                tempSplit.split_enable, &sendCtrl);					// Check DWCRESULT of split resend and set action flags
            LOG_DEBUG("Result: %i Action: 0x%08x tempInt: 0x%08x tempSplit: 0x%08x Bytes sent: %i\n",
                result, (unsigned int)sendCtrl.Raw32, (unsigned int)tempInt.Raw32, 
                (unsigned int)tempSplit.Raw32, result != DWCRESULT::Ok ? 0 : (*DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel]).size);
            if (sendCtrl.ActionFatalError) return result;			// Fatal error occured bail
            if (sendCtrl.LongerDelay) Cpu::DelayInMicroseconds(10000);			// Not yet response slower delay
                else Cpu::DelayInMicroseconds(2500);								// Small delay between split resends
        }

        if (sendCtrl.Success) {										// Send successful adjust buffer position
            // BUGBUG: In an out transfer, this_transfer doesn't mean what we think it means.
            uint32_t const this_transfer = (*DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel]).size;
            uint32_t const transferred = bufferLength - this_transfer - offset;
            LOG_DEBUG("Transferred %u bytes on channel %u. Remaining: %u\n", transferred, pipectrl.Channel, this_transfer);

            // Since our buffer is unaligned for IN endpoints
            // Copy the data from the the aligned buffer to the buffer
            // We know the aligned buffer was used because it is unaligned
            if (pipectrl.Direction == USB_DIRECTION_IN)
            {
                //Processor::InvalidateDataCache(dmaBuffer, transferred);
                for (int i = 0; i < transferred; ++i)
                {
                    buffer[offset + i] = dmaBuffer[i];
                }
                //memcpy(&buffer[offset], aligned_bufs[pipectrl.Channel], this_transfer);
                if (transferred >= 8)
                {
                    LOG_DEBUG("Data = 0x%08X'%08X at %8p\n", ((uint32_t*)&buffer[offset])[1], ((uint32_t*)&buffer[offset])[0], &buffer[offset]);
                    LOG_DEBUG("Data = 0x%08X'%08X at %8p\n", ((uint32_t*)dmaBuffer)[1], ((uint32_t*)dmaBuffer)[0], dmaBuffer);
                }
            }

            offset += transferred;
        }

    } // Loop if packets remain.
    while ((*DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel]).packet_count > 0);

    if (pipectrl.Direction == USB_DIRECTION_IN)
    {
        bufferLength -= (*DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel]).size;
    }

    return DWCRESULT::Ok;
}

/*==========================================================================}
{					   INTERNAL HOST CONTROL FUNCTIONS					    }
{==========================================================================*/

/*-INTERNAL: HCDInitialise---------------------------------------------------
 Initialises the hardware that is in use. This usually means powering up that
 hardware and it may therefore need a set delay between this call and  the
 HCDStart routine after which you can use the system.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
DWCRESULT HCDInitialise() {
    chfree = (1 << DWC_NUM_CHANNELS) - 1;

    uint32_t VendorId = *DWC_CORE_VENDORID;
    uint32_t UserId = *DWC_CORE_USERID;
    if ((VendorId & 0xfffff000) != 0x4f542000) // 'OT'2
    {
        LOG("HCD: Hardware: %c%c%x.%x%x%x (BCM%.5x). Driver incompatible. Expected OT2.xxx (BCM2708x).\n",
            (char)((VendorId >> 24) & 0xff), (char)((VendorId >> 16) & 0xff),
            (unsigned int)((VendorId >> 12) & 0xf), (unsigned int)((VendorId >> 8) & 0xf),
            (unsigned int)((VendorId >> 4) & 0xf), (unsigned int)((VendorId >> 0) & 0xf),
            (unsigned int)((UserId >> 12) & 0xFFFFF));
        return DWCRESULT::ErrorIncompatible;
    } else {
        LOG("HCD: Hardware: %c%c%x.%x%x%x (BCM%.5x).\n",
            (char)((VendorId >> 24) & 0xff),(char)((VendorId >> 16) & 0xff),
            (unsigned int)((VendorId >> 12) & 0xf), (unsigned int)((VendorId >> 8) & 0xf),
            (unsigned int)((VendorId >> 4) & 0xf), (unsigned int)((VendorId >> 0) & 0xf),
            (unsigned int)((UserId >> 12) & 0xFFFFF));
    }

    if ((*DWC_CORE_HARDWARE1).Architecture != InternalDma) {			// We only allow DMA transfer
        LOG("HCD: Host architecture does not support Internal DMA\n");
        return DWCRESULT::ErrorIncompatible;									// Return hardware incompatible
    }

    if ((*DWC_CORE_HARDWARE1).HighSpeedPhysical == NotSupported) {		// We need high speed transfers
        LOG("HCD: High speed physical unsupported\n");
        return DWCRESULT::ErrorIncompatible;									// Return hardware incompatible
    }

    struct CoreAhb tempAhb = *DWC_CORE_AHB;							// Read the AHB register to temp
    tempAhb.InterruptEnable = false;								// Clear interrupt enable bit
    DWC_CORE_AHB = tempAhb;										// Write temp back to AHB register
    DWC_CORE_INTERRUPTMASK = 0;								// Clear all interrupt masks

    if (PowerOnUsb() != DWCRESULT::Ok) {										// Power up the USB hardware
        LOG("HCD: Failed to power on USB Host Controller.\n");		// Log failed to start power up
        return DWCRESULT::ErrorIncompatible;									// Return hardware incompatible
    }
    return DWCRESULT::Ok;														// Return success
}
