/******************************************************************************
  Complete redux of CSUD (Chadderz's Simple USB Driver) by Alex Chadwick
  by Leon de Boer(LdB) 2017, 2018

  CSUD was overly complex in both it's coding and it's implementation for what
  it actually did. At it's heart CSUD simply provides the CONTROL pipe operation
  of a USB bus. That provides all the functionality to enumerate the USB bus 
  and control devices on the BUS.

*******************************************************************************/
#include <stdbool.h>			// C standard needed for bool
#include <stdlib.h>				// C standard needed for NULL
#include <stdint.h>				// C standard needed for uint8_t, uint32_t, uint64_t etc
#include <string.h>				// C standard needed for memset
#include <wchar.h>				// C standard needed for UTF for unicode descriptor support
#include "rpi-usb.h"			// This units header

#include "Mmio.h"
#include "Mailbox.h"
#include "Timer.h"
#include "Processor.h"

#include "UsbDevices.h"		// for RESULT // TODO: Fix this circular dependency

#include <concepts>

#define RPi_IO_Base_Addr MMIO_BASE

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
	volatile __attribute__((aligned(4))) struct FifoSize HostSize;	// +0x100
	volatile __attribute__((aligned(4))) struct FifoSize DataSize[15];// +0x104
};

/***************************************************************************}
{         PRIVATE INTERNAL DESIGNWARE 2.0 HOST REGISTER STRUCTURES          }
****************************************************************************/

enum ClockRate {
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
			volatile unsigned ClockRate : 2;						// @0
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
			unsigned transaction_position : 2;				// @14-15	If we are processing split the transation position Begin=2,End=1,Middle=0,All=3
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

/***************************************************************************}
{    PRIVATE POINTERS TO ALL OUR DESIGNWARE 2.0 HOST REGISTER STRUCTURES    }
****************************************************************************/

#define USB_CORE_OFFSET  0x980000	// USB CORE OFFSET FROM PERIPHERAL IO BASE ADDRESS

template < typename T > requires (sizeof(T) == 4)
struct RegisterProxy
{
	uint32_t Offset;

	constexpr RegisterProxy(uint32_t offs) : Offset(offs) {}

	void operator=(std::integral auto value) const requires (!std::is_const_v<T>) && (sizeof(value) <= 4)
	{
		*reinterpret_cast<volatile uint32_t*>(RPi_IO_Base_Addr + Offset) = static_cast<uint32_t>(value);
	}

	void operator=(const T& value) const requires (!std::is_const_v<T>)
	{
		*reinterpret_cast<volatile uint32_t*>(RPi_IO_Base_Addr + Offset) = reinterpret_cast<uint32_t const&>(value);
	}

	void operator=(std::invocable<T&> auto&& modify) const requires (!std::is_const_v<T>)
	{
		T value = **this;
		modify(value);
		*this = value;
	}

	const T operator*() const
	{
		uint32_t const result = *reinterpret_cast<volatile uint32_t*>(RPi_IO_Base_Addr + Offset);
		return reinterpret_cast<T const&>(result);
	}
};

/*--------------------------------------------------------------------------}
{					 DWC USB CORE REGISTER POINTERS						    }
{--------------------------------------------------------------------------*/
constexpr RegisterProxy<CoreOtgControl       > DWC_CORE_OTGCONTROL     		  { USB_CORE_OFFSET +  0x00 };
constexpr RegisterProxy<CoreOtgInterrupt     > DWC_CORE_OTGINTERRUPT   		  { USB_CORE_OFFSET +  0x04 };
constexpr RegisterProxy<CoreAhb              > DWC_CORE_AHB            		  { USB_CORE_OFFSET +  0x08 };
constexpr RegisterProxy<UsbControl           > DWC_CORE_CONTROL        		  { USB_CORE_OFFSET +  0x0C };
constexpr RegisterProxy<CoreReset            > DWC_CORE_RESET          		  { USB_CORE_OFFSET +  0x10 };
constexpr RegisterProxy<CoreInterrupts       > DWC_CORE_INTERRUPT      		  { USB_CORE_OFFSET +  0x14 };
constexpr RegisterProxy<CoreInterrupts       > DWC_CORE_INTERRUPTMASK  		  { USB_CORE_OFFSET +  0x18 };
constexpr RegisterProxy<uint32_t             > DWC_CORE_RECEIVESIZE    		  { USB_CORE_OFFSET +  0x24 };
constexpr RegisterProxy<FifoSize             > DWC_CORE_NONPERIODICFIFO_SIZE  { USB_CORE_OFFSET +  0x28 };
constexpr RegisterProxy<NonPeriodicFifoStatus> DWC_CORE_NONPERIODICFIFO_STATUS{ USB_CORE_OFFSET +  0x2C };
constexpr RegisterProxy<uint32_t             > DWC_CORE_USERID                { USB_CORE_OFFSET +  0x3C };
constexpr RegisterProxy<const uint32_t       > DWC_CORE_VENDORID              { USB_CORE_OFFSET +  0x40 };
constexpr RegisterProxy<const CoreHardware0  > DWC_CORE_HARDWARE0             { USB_CORE_OFFSET +  0x44 };
constexpr RegisterProxy<const CoreHardware1  > DWC_CORE_HARDWARE1             { USB_CORE_OFFSET +  0x48 };
constexpr RegisterProxy<const CoreHardware2  > DWC_CORE_HARDWARE2             { USB_CORE_OFFSET +  0x4C };
constexpr RegisterProxy<const CoreHardware3  > DWC_CORE_HARDWARE3             { USB_CORE_OFFSET +  0x50 };
constexpr RegisterProxy<FifoSize		     > DWC_CORE_PERIODICINFO_HostSize { USB_CORE_OFFSET + 0x100 };

/*--------------------------------------------------------------------------}
{					DWC USB HOST REGISTER POINTERS						    }
{--------------------------------------------------------------------------*/
constexpr RegisterProxy<HostConfig               > DWC_HOST_CONFIG                { USB_CORE_OFFSET + 0x400 };
constexpr RegisterProxy<HostFrameInterval        > DWC_HOST_FRAMEINTERVAL         { USB_CORE_OFFSET + 0x404 };
constexpr RegisterProxy<HostFrameControl         > DWC_HOST_FRAMECONTROL          { USB_CORE_OFFSET + 0x408 };
constexpr RegisterProxy<HostFifoStatus           > DWC_HOST_FIFOSTATUS            { USB_CORE_OFFSET + 0x410 };
constexpr RegisterProxy<uint32_t                 > DWC_HOST_INTERRUPT             { USB_CORE_OFFSET + 0x414 };
constexpr RegisterProxy<uint32_t                 > DWC_HOST_INTERRUPTMASK         { USB_CORE_OFFSET + 0x418 };
constexpr RegisterProxy<uint32_t                 > DWC_HOST_FRAMELIST             { USB_CORE_OFFSET + 0x41C };
constexpr RegisterProxy<HostPort                 > DWC_HOST_PORT                  { USB_CORE_OFFSET + 0x440 };
constexpr RegisterProxy<HostChannelCharacteristic> DWC_HOST_CHANNEL_Characteristic[16]
{
	{ USB_CORE_OFFSET + 0x500 },
	{ USB_CORE_OFFSET + 0x520 },
	{ USB_CORE_OFFSET + 0x540 },
	{ USB_CORE_OFFSET + 0x560 },
	{ USB_CORE_OFFSET + 0x580 },
	{ USB_CORE_OFFSET + 0x5A0 },
	{ USB_CORE_OFFSET + 0x5C0 },
	{ USB_CORE_OFFSET + 0x5E0 },
	{ USB_CORE_OFFSET + 0x600 },
	{ USB_CORE_OFFSET + 0x620 },
	{ USB_CORE_OFFSET + 0x640 },
	{ USB_CORE_OFFSET + 0x660 },
	{ USB_CORE_OFFSET + 0x680 },
	{ USB_CORE_OFFSET + 0x6A0 },
	{ USB_CORE_OFFSET + 0x6C0 },
	{ USB_CORE_OFFSET + 0x6E0 },
};
constexpr RegisterProxy<HostChannelSplitControl  > DWC_HOST_CHANNEL_SplitCtrl     [16]
{
	{ USB_CORE_OFFSET + 0x504 },
	{ USB_CORE_OFFSET + 0x524 },
	{ USB_CORE_OFFSET + 0x544 },
	{ USB_CORE_OFFSET + 0x564 },
	{ USB_CORE_OFFSET + 0x584 },
	{ USB_CORE_OFFSET + 0x5A4 },
	{ USB_CORE_OFFSET + 0x5C4 },
	{ USB_CORE_OFFSET + 0x5E4 },
	{ USB_CORE_OFFSET + 0x604 },
	{ USB_CORE_OFFSET + 0x624 },
	{ USB_CORE_OFFSET + 0x644 },
	{ USB_CORE_OFFSET + 0x664 },
	{ USB_CORE_OFFSET + 0x684 },
	{ USB_CORE_OFFSET + 0x6A4 },
	{ USB_CORE_OFFSET + 0x6C4 },
	{ USB_CORE_OFFSET + 0x6E4 },
};
constexpr RegisterProxy<ChannelInterrupts        > DWC_HOST_CHANNEL_Interrupt     [16]
{
	{ USB_CORE_OFFSET + 0x508 },
	{ USB_CORE_OFFSET + 0x528 },
	{ USB_CORE_OFFSET + 0x548 },
	{ USB_CORE_OFFSET + 0x568 },
	{ USB_CORE_OFFSET + 0x588 },
	{ USB_CORE_OFFSET + 0x5A8 },
	{ USB_CORE_OFFSET + 0x5C8 },
	{ USB_CORE_OFFSET + 0x5E8 },
	{ USB_CORE_OFFSET + 0x608 },
	{ USB_CORE_OFFSET + 0x628 },
	{ USB_CORE_OFFSET + 0x648 },
	{ USB_CORE_OFFSET + 0x668 },
	{ USB_CORE_OFFSET + 0x688 },
	{ USB_CORE_OFFSET + 0x6A8 },
	{ USB_CORE_OFFSET + 0x6C8 },
	{ USB_CORE_OFFSET + 0x6E8 },
};
constexpr RegisterProxy<ChannelInterrupts        > DWC_HOST_CHANNEL_InterruptMask [16]
{
	{ USB_CORE_OFFSET + 0x50C },
	{ USB_CORE_OFFSET + 0x52C },
	{ USB_CORE_OFFSET + 0x54C },
	{ USB_CORE_OFFSET + 0x56C },
	{ USB_CORE_OFFSET + 0x58C },
	{ USB_CORE_OFFSET + 0x5AC },
	{ USB_CORE_OFFSET + 0x5CC },
	{ USB_CORE_OFFSET + 0x5EC },
	{ USB_CORE_OFFSET + 0x60C },
	{ USB_CORE_OFFSET + 0x62C },
	{ USB_CORE_OFFSET + 0x64C },
	{ USB_CORE_OFFSET + 0x66C },
	{ USB_CORE_OFFSET + 0x68C },
	{ USB_CORE_OFFSET + 0x6AC },
	{ USB_CORE_OFFSET + 0x6CC },
	{ USB_CORE_OFFSET + 0x6EC },
};
constexpr RegisterProxy<HostTransferSize         > DWC_HOST_CHANNEL_TransferSize  [16]
{
	{ USB_CORE_OFFSET + 0x510 },
	{ USB_CORE_OFFSET + 0x530 },
	{ USB_CORE_OFFSET + 0x550 },
	{ USB_CORE_OFFSET + 0x570 },
	{ USB_CORE_OFFSET + 0x590 },
	{ USB_CORE_OFFSET + 0x5B0 },
	{ USB_CORE_OFFSET + 0x5D0 },
	{ USB_CORE_OFFSET + 0x5F0 },
	{ USB_CORE_OFFSET + 0x610 },
	{ USB_CORE_OFFSET + 0x630 },
	{ USB_CORE_OFFSET + 0x650 },
	{ USB_CORE_OFFSET + 0x670 },
	{ USB_CORE_OFFSET + 0x690 },
	{ USB_CORE_OFFSET + 0x6B0 },
	{ USB_CORE_OFFSET + 0x6D0 },
	{ USB_CORE_OFFSET + 0x6F0 },
};
constexpr RegisterProxy<uint32_t                 > DWC_HOST_CHANNEL_DmaAddr       [16]
{
	{ USB_CORE_OFFSET + 0x514 },
	{ USB_CORE_OFFSET + 0x534 },
	{ USB_CORE_OFFSET + 0x554 },
	{ USB_CORE_OFFSET + 0x574 },
	{ USB_CORE_OFFSET + 0x594 },
	{ USB_CORE_OFFSET + 0x5B4 },
	{ USB_CORE_OFFSET + 0x5D4 },
	{ USB_CORE_OFFSET + 0x5F4 },
	{ USB_CORE_OFFSET + 0x614 },
	{ USB_CORE_OFFSET + 0x634 },
	{ USB_CORE_OFFSET + 0x654 },
	{ USB_CORE_OFFSET + 0x674 },
	{ USB_CORE_OFFSET + 0x694 },
	{ USB_CORE_OFFSET + 0x6B4 },
	{ USB_CORE_OFFSET + 0x6D4 },
	{ USB_CORE_OFFSET + 0x6F4 },
};

/*--------------------------------------------------------------------------}
{					DWC POWER AND CLOCK REGISTER POINTER				    }
{--------------------------------------------------------------------------*/
constexpr RegisterProxy<PowerReg> DWC_POWER_AND_CLOCK{ USB_CORE_OFFSET + 0xE00 };


/*--------------------------------------------------------------------------}
{				 INTERNAL USB STRUCTURE COMPILE TIME CHECKS		            }
{--------------------------------------------------------------------------*/
/* GIVEN THE AMOUNT OF PRECISE PACKING OF THESE STRUCTURES .. IT'S PRUDENT */
/* TO CHECK THEM AT COMPILE TIME. USE IS POINTLESS IF THE SIZES ARE WRONG. */
/*-------------------------------------------------------------------------*/
/* If you have never seen compile time assertions it's worth google search */
/* on "Compile Time Assertions". It is part of the C11++ specification and */
/* all compilers that support the standard will have them (GCC, MSC inc)   */
/*-------------------------------------------------------------------------*/

/* DESIGNWARE 2.0 REGISTERS */
static_assert(sizeof(struct CoreOtgControl) == 0x04, "Register/Structure should be 32bits (4 bytes)");
static_assert(sizeof(struct CoreOtgInterrupt) == 0x04, "Register/Structure should be 32bits (4 bytes)");
static_assert(sizeof(struct CoreAhb) == 0x04, "Register/Structure should be 32bits (4 bytes)");
static_assert(sizeof(struct UsbControl) == 0x04, "Register/Structure should be 32bits (4 bytes)");
static_assert(sizeof(struct CoreReset) == 0x04, "Register/Structure should be 32bits (4 bytes)");
static_assert(sizeof(struct CoreInterrupts) == 0x04, "Register/Structure should be 32bits (4 bytes)");

static_assert(sizeof(FifoSize) == 0x04, "Register/Structure should be 32bits (4 bytes)");
static_assert(sizeof(NonPeriodicFifoStatus) == 0x04, "Register/Structure should be 32bits (4 bytes)");

//static_assert(sizeof(struct CoreHardware) == 0x10, "Register/Structure should be 4x32bits (16 bytes)");

/* USB SPECIFICATION STRUCTURES */
static_assert(sizeof(struct HubPortFullStatus) == 0x04, "Structure should be 32bits (4 bytes)");
static_assert(sizeof(struct HubFullStatus) == 0x04, "Structure should be 32bits (4 bytes)");
static_assert(sizeof(struct UsbDescriptorHeader) == 0x02, "Structure should be 2 bytes");
static_assert(sizeof(struct UsbEndpointDescriptor) == 0x07, "Structure should be 7 bytes");
static_assert(sizeof(struct UsbDeviceRequest) == 0x08, "Structure should be 8 bytes");
static_assert(sizeof(struct HubDescriptor) == 0x09, "Structure should be 9 bytes");
static_assert(sizeof(struct UsbInterfaceDescriptor) == 0x09, "Structure should be 9 bytes");
static_assert(sizeof(struct ConfigurationDescriptor) == 0x09, "Structure should be 9 bytes");
static_assert(sizeof(struct DeviceDescriptor) == 0x12, "Structure should be 18 bytes");

/* INTERNAL STRUCTURES */
static_assert(sizeof(struct UsbSendControl) == 0x04, "Structure should be 32bits (4 bytes)");

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
	return (31 - __builtin_clz(word));								// Return index of first set bit
}

/*-[INTERNAL: dwc_get_free_channel ]-----------------------------------------
. Finds and reserves an unused DWC USB host channel. This is blocking and
. will wait until a channel is available if all in use.
. RETURN: Index of the free channel
.--------------------------------------------------------------------------*/
unsigned int dwc_get_free_channel(void)
{
	unsigned int chan;
	//wait(chfree_sema);												// Wait for a free channel	
	//ENTER_KERNEL_CRITICAL_SECTION();								// Must disable scheduler as we play with the free channels
	chan = first_set_bit(chfree);									// Find the first free channel .. there must be one because of semaphore
	chfree &= ~((uint32_t)1 << chan);								// Mark the channel as no longer free										
	//EXIT_KERNEL_CRITICAL_SECTION();									// Exit the critical section
	return chan;													// Return the channel
}

/*-[INTERNAL: dwc_release_channel ]-----------------------------------------
. Releases the given DWC USB host channel that was in use and marks as free.
.--------------------------------------------------------------------------*/
void dwc_release_channel(unsigned int chan)
{
	//ENTER_KERNEL_CRITICAL_SECTION();								// Entering a critical section
	chfree |= ((uint32_t)1 << chan);								// Mark channel as free
	//EXIT_KERNEL_CRITICAL_SECTION();									// Exit the critical section
	//signal(chfree_sema);											// Signal channel free
}

/*==========================================================================}
{			    INTERNAL FAKE ROOT HUB MESSAGE HANDLER FUNCTIONS		    }
{==========================================================================*/

void DwcClearEnable()
{
	auto tempPort = *DWC_HOST_PORT;						// Read the host port
	tempPort.Raw32 &= HOSTPORTMASK;					// Cleave off all the triggers
	tempPort.Enable = true;							// Set enable change bit ... This is one of those set bit to write bits (bit 2)
	DWC_HOST_PORT = tempPort;						// Write the value back
}

void DwcResume()
{
	DWC_POWER_AND_CLOCK = 0;
	Timer::Delay(5000);
	auto tempPort = *DWC_HOST_PORT;						// Read the host port
	tempPort.Raw32 &= HOSTPORTMASK;					// Cleave off all the triggers
	tempPort.Resume = true;							// Set the bit we want
	DWC_HOST_PORT = tempPort;						// Write the value back
	Timer::Delay(100000);
	tempPort = *DWC_HOST_PORT;						// Read the host port
	tempPort.Raw32 &= HOSTPORTMASK;					// Cleave off all the triggers
	tempPort.Suspend = false;						// Clear the bit we want
	tempPort.Resume = false;						// Clear the bit we want
	DWC_HOST_PORT = tempPort;						// Write the value back
}

void DwcPowerOff()
{
	LOG("Physical host power off\n");
	auto tempPort = *DWC_HOST_PORT;						// Read the host port
	tempPort.Raw32 &= HOSTPORTMASK;					// Cleave off all the triggers
	tempPort.Power = false;							// Clear the bit we want
	DWC_HOST_PORT = tempPort;						// Write the value back
}

void DwcConnectionChange()
{
	auto tempPort = *DWC_HOST_PORT;						// Read the host port
	tempPort.Raw32 &= HOSTPORTMASK;					// Cleave off all the triggers
	tempPort.ConnectChanged = true;					// Set connect change bit ... This is one of those set bit to write bits (bit 1)
	DWC_HOST_PORT = tempPort;						// Write the value back
}

void DwcEnableChange()
{
	auto tempPort = *DWC_HOST_PORT;						// Read the host port
	tempPort.Raw32 &= HOSTPORTMASK;					// Cleave off all the triggers
	tempPort.EnableChanged = true;					// Set enable change bit ... This is one of those set bit to write bits (bit 3)
	DWC_HOST_PORT = tempPort;						// Write the value back
}

void DwcOverCurrentChange()
{
	auto tempPort = *DWC_HOST_PORT;						// Read the host port
	tempPort.Raw32 &= HOSTPORTMASK;					// Cleave off all the triggers
	tempPort.OverCurrentChanged = true;				// Set overcurrent change bit ... This is one of those set bit to write bits (bit 5)
	DWC_HOST_PORT = tempPort;						// Write the value back
}

void DwcReset()
{
	auto tempPower = *DWC_POWER_AND_CLOCK;				// read power and clock
	tempPower.EnableSleepClockGating = false;		// Turn off sleep clock gating if on
	tempPower.StopPClock = false;					// Turn off stop clock
	DWC_POWER_AND_CLOCK = tempPower;				// Write back to register
	Timer::Delay(10000);							// Small delay
	DWC_POWER_AND_CLOCK = 0;						// Now clear everything

	auto tempPort = *DWC_HOST_PORT;						// Read the host port
	tempPort.Raw32 &= HOSTPORTMASK;					// Cleave off all the triggers
	tempPort.Suspend = false;						// Clear the bit we want
	tempPort.Reset = true;							// Set bit we want
	tempPort.Power = true;							// Set the bit we want
	DWC_HOST_PORT = tempPort;						// Write the value back
	Timer::Delay(60000);
	tempPort = *DWC_HOST_PORT;						// Read the host port
	tempPort.Raw32 &= HOSTPORTMASK;					// Cleave off all the triggers
	tempPort.Reset = false;							// Clear bit we want
	DWC_HOST_PORT = tempPort;						// Write the value back
}

void DwcPowerOn()
{
	auto tempPort = *DWC_HOST_PORT;					// Read the host port
	tempPort.Raw32 &= HOSTPORTMASK;					// Cleave off all the triggers
	tempPort.Power = true;							// Set the bit we want
	DWC_HOST_PORT = tempPort;						// Write the value back
}

HubPortFullStatus DwcGetPortStatus()
{
	auto tempPort = *DWC_HOST_PORT;							// Read the host port
	LOG_DEBUG("GetStatus: Port %u status: 0x%08x %032b\n", request->Index, tempPort.Raw32, tempPort.Raw32);
	HubPortFullStatus replyPort{};
	replyPort.Status.Connected = tempPort.Connect;	// Transfer connect state
	replyPort.Status.Enabled = tempPort.Enable;// Transfer enabled state
	replyPort.Status.Suspended = tempPort.Suspend;	// Transfer suspend state
	replyPort.Status.OverCurrent = tempPort.OverCurrent;// Transfer overcurrent state
	replyPort.Status.Reset = tempPort.Reset;	// Transfer reset state
	replyPort.Status.Power = tempPort.Power;	// Transfer power state
	if (tempPort.Speed == USB_SPEED_HIGH)
		replyPort.Status.HighSpeedAttatched = true;// Set high speed state
	else if (tempPort.Speed == USB_SPEED_LOW)
		replyPort.Status.LowSpeedAttatched = true;	// Set low speed state
	replyPort.Status.TestMode = tempPort.TestControl;// Transfer test mode state
	replyPort.Change.ConnectedChanged = tempPort.ConnectChanged;// Transfer Connect changed state
	replyPort.Change.EnabledChanged = false;	// Always send back as zero .. dorky DWC2.0 doesn't have you have to monitor
	replyPort.Change.OverCurrentChanged = tempPort.OverCurrentChanged;// Transfer overcurrent changed state
	replyPort.Change.ResetChanged = false;		// Always send back as zero .. dorky DWC2.0 doesn't have you have to monitor
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
RESULT PowerOnUsb(void) {
	uint32_t __attribute__((aligned(16))) volatile mailbox_message_buffer[8];
	auto mailbox_message = Mailbox::AsGpuPointer(mailbox_message_buffer);
	mailbox_message[0] = sizeof(mailbox_message);
	mailbox_message[1] = 0;
	mailbox_message[2] = (uint32_t)Mailbox::Tag::SET_POWER_STATE;
	mailbox_message[3] = 8;
	mailbox_message[4] = 8;
	mailbox_message[5] = 0x3;    // device = USB
	mailbox_message[6] = 0x1;	 // 1 = on	
	mailbox_message[7] = 0x0;

	if (Mailbox::SendTags(std::span{ mailbox_message, 8 }) && (mailbox_message[4] == 0x80000008)) {
		return OK;
	}
	return ErrorDevice;												// Failed to turn on
}

/*-INTERNAL: PowerOffUsb-----------------------------------------------------
 Uses PI mailbox to turn power onto USB see website about command 0x28001
 https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT PowerOffUsb(void) {
	uint32_t __attribute__((aligned(16))) mailbox_message_buffer[8];
	auto mailbox_message = Mailbox::AsGpuPointer(mailbox_message_buffer);
	mailbox_message[0] = sizeof(mailbox_message);
	mailbox_message[1] = 0;
	mailbox_message[2] = (uint32_t)Mailbox::Tag::SET_POWER_STATE;
	mailbox_message[3] = 8;
	mailbox_message[4] = 8;
	mailbox_message[5] = 0x3;    // device = USB
	mailbox_message[6] = 0x0;	 // 1 = off	
	mailbox_message[7] = 0x0;

	if (Mailbox::SendTags(std::span{ mailbox_message, 8 }) && (mailbox_message[4] == 0x80000008)) {
		return OK;
	}
	return ErrorDevice;												// Failed to turn on
}

/*-INTERNAL: HCDReset--------------------------------------------------------
 Does a softstart on core and uses ARM timer tick to timeout if neccessary.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT HCDReset(void) {

	uint64_t ticks100ms = Timer::GetPerformanceTicksForUs(100'000);
	uint64_t original_tick = Timer::GetPerformanceCounter();							// Hold original tickcount
	do {
		if (Timer::GetPerformanceCounter() - original_tick > ticks100ms) {
			return ErrorTimeout;									// Return timeout error
		}
	} while ((*DWC_CORE_RESET).AhbMasterIdle == false);				// Keep looping until idle or timeout

	DWC_CORE_RESET = [](auto& r){ r.CoreSoft = true; };								// Reset the soft core

	struct CoreReset temp;
	original_tick = Timer::GetPerformanceCounter();							// Hold original tickcount
	do {
		if (Timer::GetPerformanceCounter() - original_tick > ticks100ms) {
			return ErrorTimeout;									// Return timeout error
		}
		temp = *DWC_CORE_RESET;										// Read reset register
	} while (temp.CoreSoft == true || temp.AhbMasterIdle == false); // Keep looping until soft reset low/idle high or timeout

	return OK;														// Return success
}

/*-INTERNAL: HCDTransmitFifoFlush-------------------------------------------
 Flushes TX fifo buffers again uses ARM timer tick to timeout if neccessary.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT HCDTransmitFifoFlush(enum CoreFifoFlush fifo) {

	DWC_CORE_RESET = [=](auto& r){ r.TransmitFifoFlushNumber = fifo; };					// Set fifo flush type
	DWC_CORE_RESET = [](auto& r){ r.TransmitFifoFlush = true; };						// Execute transmit flush

	uint64_t ticks100ms = Timer::GetPerformanceTicksForUs(100'000);
	uint64_t original_tick = Timer::GetPerformanceCounter();							// Hold original tick count
	do {
		if (Timer::GetPerformanceCounter() - original_tick > ticks100ms) {
			return ErrorTimeout;									// Return timeout error
		}
	} while ((*DWC_CORE_RESET).TransmitFifoFlush == true);			// Loop until flush signal low or timeout

	return OK;														// Return success
}

/*-INTERNAL: HCDReceiveFifoFlush---------------------------------------------
 Flushes RX fifo buffers again uses ARM timer tick to timeout if neccessary.
 11Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT HCDReceiveFifoFlush(void) {

	DWC_CORE_RESET = [](auto& r){ r.ReceiveFifoFlush = true; };						// Execute recieve flush

	uint64_t ticks100ms = Timer::GetPerformanceTicksForUs(100'000);
	uint64_t original_tick = Timer::GetPerformanceCounter();							// Hold original tick count
	do {
		if (Timer::GetPerformanceCounter() - original_tick > ticks100ms) {
			return ErrorTimeout;									// Return timeout error
		}
	} while ((*DWC_CORE_RESET).ReceiveFifoFlush == true);				// Loop until flush signal low or timeout

	return OK;														// Return success
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
/* IC’s are drastically reduced. This not only lowers the cost of Link and */
/* PHY IC’s, but also makes for a smaller PCB.							   */
/*-------------------------------------------------------------------------*/
RESULT HCDStart (void) {
	RESULT result;
	struct UsbControl coreUsb;

	coreUsb = *DWC_CORE_CONTROL;									// Read core control register
	coreUsb.UlpiDriveExternalVbus = 0;								// ULPI bit UseExternalVbusIndicator set to 0
	coreUsb.TsDlinePulseEnable = 0;									// Dline pulsing set to zero
	DWC_CORE_CONTROL = coreUsb;									// Write control register

	Timer::Delay(1000);

	LOG_DEBUG("HCD: Master reset.\n");								
	if ((result = HCDReset()) != OK) {								// Attempt a HCD reset which will soft reset the USB core
		LOG("FATAL ERROR: Could not do a Master reset on HCD.\n");	// Log the fatal error
		return result;												// Return fail result
	}

	Timer::Delay(1000);

	if (!PhyInitialised) {											// If physical interface hasn't been initialized
		LOG_DEBUG("HCD: One time phy initialisation.\n");
		PhyInitialised = true;										// Read that we have done this one time call
		coreUsb = *DWC_CORE_CONTROL;								// Read core control register
		coreUsb.ModeSelect = UTMI;									// We will bring up UTMI+ interface .. no ULPI
		LOG_DEBUG("HCD: Interface: UTMI+.\n");						
		coreUsb.PhyInterface = false;								// Take existing phy interface down .. I assume
		DWC_CORE_CONTROL = coreUsb;								// Write control register
		if ((result = HCDReset()) != OK) {							// You need to do a soft reset to make those settings happen
			LOG("FATAL ERROR: Could not do a Master reset on HCD.\n");// Log the fatal error
			return result;											// Return fail result
		}
	}

	Timer::Delay(1000);

	coreUsb = *DWC_CORE_CONTROL;									// Read control again after possible reset above									
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
	DWC_CORE_CONTROL = coreUsb;									// Write control register

	Timer::Delay(1000);

	struct CoreAhb tempAhb;
	tempAhb = *DWC_CORE_AHB;										// Read the AHB register
	tempAhb.DmaEnable = true;										// Set the DMA on
	tempAhb.DmaRemainderMode = Incremental;							// DMA remainders that aren't aligned use incremental 
	DWC_CORE_AHB = tempAhb;										// Write the AHB register

	Timer::Delay(1000);

	coreUsb = *DWC_CORE_CONTROL;									// Read control register ... again	
	switch ((*DWC_CORE_HARDWARE1).OperatingMode) {					// Switch based on capabilities read from hardware
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
	DWC_CORE_CONTROL = coreUsb;									// Write control register 
	LOG_DEBUG("HCD: Core started.\n");
	LOG_DEBUG("HCD: Starting host.\n");

	Timer::Delay(1000);

	DWC_POWER_AND_CLOCK = {};									// Release any power or clock halts given the bit names 

	if ((*DWC_CORE_HARDWARE1).HighSpeedPhysical == Ulpi
		&& (*DWC_CORE_HARDWARE1).FullSpeedPhysical == Dedicated
		&& coreUsb.UlpiFsls) {										// ULPI FsLs Host mode must have 48Mhz clock
		LOG_DEBUG("HCD: Host clock: 48Mhz.\n");
		DWC_HOST_CONFIG = [](auto& r) { r.ClockRate = Clock48MHz; };					// Select 48Mhz clock
	} else {
		LOG_DEBUG("HCD: Host clock: 30-60Mhz.\n");
		DWC_HOST_CONFIG = [](auto& r) { r.ClockRate = Clock30_60MHz; };					// Select 30-60Mhz clock
	}

	DWC_HOST_CONFIG = [](auto& r){ r.FslsOnly = true; };								// ULPI FsLs Host mode, I assume other mode is ULPI only  .. documentation would be nice

	Timer::Delay(1000);

	DWC_CORE_RECEIVESIZE = ReceiveFifoSize;						// Set recieve fifo size

	Timer::Delay(1000);

	DWC_CORE_NONPERIODICFIFO_SIZE = [](auto& r){ r.Depth = NonPeriodicFifoSize; };		// Set non-periodic fifo depth
	DWC_CORE_NONPERIODICFIFO_SIZE = [](auto& r){ r.StartAddress = ReceiveFifoSize; };	// Set non-periodic start address

	Timer::Delay(1000);

	DWC_CORE_PERIODICINFO_HostSize = [](auto& r){ r.Depth = PeriodicFifoSize; };		// Set periodic fifo depth
	DWC_CORE_PERIODICINFO_HostSize = [](auto& r){ r.StartAddress = ReceiveFifoSize + NonPeriodicFifoSize; }; // Set periodic start address

	Timer::Delay(1000);

	LOG_DEBUG("HCD: Set HNP: enabled.\n");

	struct CoreOtgControl tempOtgControl;
	tempOtgControl = *DWC_CORE_OTGCONTROL;							// Read the OTG register
	tempOtgControl.HostSetHnpEnable = true;							// Enable the host
	DWC_CORE_OTGCONTROL = tempOtgControl;							// Write the Otg register

	Timer::Delay(1000);

	if ((result = HCDTransmitFifoFlush(FlushAll)) != OK)			// Flush the transmit FIFO
		return result;												// Return error source if fatal fail
	Timer::Delay(1000);

	if ((result = HCDReceiveFifoFlush()) != OK)						// Flush the recieve FIFO
		return result;												// Return error source if fatal fail
	Timer::Delay(1000);


	printf2("DWC_HOST_CONFIG: 0x%08X\n", (*DWC_HOST_CONFIG).Raw32);	// Debug print the host config register
	printf2("DWC_CORE_HARDWARE1: 0x%08X %b\n", (*DWC_CORE_HARDWARE1), (*DWC_CORE_HARDWARE1));	// Debug print the hardware1 register
	for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
		printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, (*DWC_HOST_CHANNEL_Characteristic[channel]).Raw32); // Debug print the host channel characteristics
	}
	printf2("\n");

	if (!(*DWC_HOST_CONFIG).EnableDmaDescriptor) {
/*
		for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
			struct HostChannelCharacteristic tempChar;
			tempChar = *DWC_HOST_CHANNEL_Characteristic[channel];	// Read and hold characteristic	
			printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, tempChar.Raw32); // Debug print the host channel characteristics
			tempChar.channel_enable = false;						// Clear host channel enable
			tempChar.channel_disable = true;						// Set host channel disable
			tempChar.endpoint_direction = USB_DIRECTION_IN;			// Set direction to in/read
			DWC_HOST_CHANNEL_Characteristic[channel] = tempChar;	// Write the characteristics
		}
		printf2("\n");

		Timer::Delay(100'000);

		//for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
		//	struct HostChannelCharacteristic tempChar;
		//	uint64_t ticks100ms = Timer::GetPerformanceTicksForUs(100'000);
		//	uint64_t original_tick = Timer::GetPerformanceCounter();					// Hold original timertick
		//	do {
		//		tempChar = *DWC_HOST_CHANNEL_Characteristic[channel];
		//		if (Timer::GetPerformanceCounter() - original_tick > ticks100ms) {
		//			LOG("HCD: Unable to set halt on channel %i %X.\n", channel, tempChar.Raw32);
		//			original_tick = Timer::GetPerformanceCounter();
		//			//Processor::Halt();
		//		}
		//	} while (!tempChar.channel_disable || !tempChar.channel_enable);// Repeat until goes enabled or timeout
		//}
		for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
			printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, (*DWC_HOST_CHANNEL_Characteristic[channel]).Raw32); // Debug print the host channel characteristics
		}
		printf2("\n");
*/

		// Halt channels to put them into known state.
		for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
			struct HostChannelCharacteristic tempChar;
			tempChar = *DWC_HOST_CHANNEL_Characteristic[channel];	// Read and hold characteristic	
			printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, tempChar.Raw32); // Debug print the host channel characteristics
			tempChar.channel_enable = true;							// Set host channel enable
			tempChar.channel_disable = false;						// Set host channel disable
			tempChar.endpoint_direction = USB_DIRECTION_IN;			// Set direction to in/read
			DWC_HOST_CHANNEL_Characteristic[channel] = tempChar;	// Write the characteristics
		}
		printf2("\n");

		Timer::Delay(100'000);

		for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
			printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, (*DWC_HOST_CHANNEL_Characteristic[channel]).Raw32); // Debug print the host channel characteristics
		}

		for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
			struct HostChannelCharacteristic tempChar;
			uint64_t ticks100ms = Timer::GetPerformanceTicksForUs(100'000);
			uint64_t original_tick = Timer::GetPerformanceCounter();					// Hold original timertick
			do {
				tempChar = *DWC_HOST_CHANNEL_Characteristic[channel];
				if (Timer::GetPerformanceCounter() - original_tick > ticks100ms) {
					LOG("HCD: Unable to clear halt on channel %i %X.\n", channel, tempChar.Raw32);
					original_tick = Timer::GetPerformanceCounter();
					//Processor::Halt();
				}
			} while (tempChar.channel_disable || !tempChar.channel_enable);// Repeat until goes enabled or timeout
		}
		printf2("\n");
		for (int channel = 0; channel < (*DWC_CORE_HARDWARE1).HostChannelCount; channel++) {
			printf2("DWC_HOST_CHANNEL_Characteristic[%d]: 0x%08X\n", channel, (*DWC_HOST_CHANNEL_Characteristic[channel]).Raw32); // Debug print the host channel characteristics
		}
	}

	Timer::Delay(1000);

	struct HostPort tempPort;
	tempPort = *DWC_HOST_PORT;										// Fetch host port 
	LOG_DEBUG("HCD: Initial host port: 0x%08X\n", tempPort.Raw32);
	if (!tempPort.Power) {
		LOG_DEBUG("HCD: Initial power physical host up.\n");
		tempPort.Raw32 &= HOSTPORTMASK;								// Cleave off all the temp bits	
		tempPort.Power = true;										// Set the power bit
		DWC_HOST_PORT = tempPort;									// Write value to port
	}

	Timer::Delay(1000);

	LOG_DEBUG("HCD: Initial resetting physical host.\n");
	tempPort = *DWC_HOST_PORT;										// Fetch host port 
	LOG_DEBUG("HCD: Powered host port: 0x%08X\n", tempPort.Raw32);
	
	tempPort.Raw32 &= HOSTPORTMASK;									// Cleave off all the temp bits	
	tempPort.Reset = true;											// Set the reset bit
	DWC_HOST_PORT = tempPort;										// Write value to port
	Timer::Delay(60000);												// 60ms delay
	tempPort = *DWC_HOST_PORT;										// Fetch host port 
	LOG_DEBUG("HCD: Reset host port: 0x%08X\n", tempPort.Raw32);
	
	tempPort.Raw32 &= HOSTPORTMASK;									// Cleave off all the temp bits	
	tempPort.Reset = false;											// Clear the reset bit
	DWC_HOST_PORT = tempPort;										// Write value to port
	
	Timer::Delay(1000);

	LOG_DEBUG("HCD: Reset host port: 0x%08X\n", tempPort.Raw32);
	
	LOG_DEBUG("HCD: Successfully started.\n");

	return OK;														// Return success
}


/*==========================================================================}
{				   INTERNAL HOST TRANSMISSION ROUTINES					    }
{==========================================================================*/

/*-INTERNAL: HCDCheckErrorAndAction -----------------------------------------
 Given a channel interrupt flags and whether packet was complete (not split)
 it will set sendControl structure with what to do next.
 24Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT HCDCheckErrorAndAction(struct ChannelInterrupts interrupts, bool packetSplit, struct UsbSendControl* sendCtrl) {
	sendCtrl->ActionResendSplit = false;							// Make sure resend split flag is cleared
	sendCtrl->ActionRetry = false;									// Make sure retry flag is cleared
	/* First deal with all the fatal errors .. no use dealing with trivial errors if these are set */
	if (interrupts.AhbError) {										// Ahb error signalled .. which means packet size too large
		sendCtrl->ActionFatalError = true;							// This is a fatal error the packet size is all wrong
		return ErrorDevice;											// Return error device
	}
	if (interrupts.DataToggleError) {								// In bulk tranmission endpoint is supposed to toggle between data0/data1
		sendCtrl->ActionFatalError = true;							// Pretty sure this is a fatal error you can't fix it by resending
		return ErrorTransmission;									// Transmission error
	}
	/* Next deal with the fully successful case  ... we can return OK */
	if (interrupts.Acknowledgement) {								// Endpoint device acknowledge
		if (interrupts.TransferComplete) sendCtrl->Success = true;	// You can set the success flag
			else sendCtrl->ActionResendSplit = true;				// Action is to try sending split again
		sendCtrl->GlobalTries = 0;
		return OK;													// Return OK result
	}
	/* Everything else is minor error invoking a retry .. so first update counts */
	if (packetSplit) {
		sendCtrl->SplitTries++;										// Increment split tries as we have a split packet
		if (sendCtrl->SplitTries == 5) {							// Ridiculous number of split resends reached .. fatal error
			sendCtrl->ActionFatalError = true;						// This is a fatal error something is very wrong
			return ErrorTransmission;								// Transmission error
		}
		sendCtrl->ActionResendSplit = true;							// Action is to try sending split again
	} else {
		sendCtrl->PacketTries++;									// Increment packet tries as packet was not split
		if (sendCtrl->PacketTries == 3) {							// Ridiculous number of packet resends reached .. fatal error
			sendCtrl->ActionFatalError = true;						// This is a fatal error something is very wrong
			return ErrorTransmission;								// Transmission error
		}
		sendCtrl->ActionRetry = true;								// Action is to try sending the packet again
	}
	/* Check no transmission errors and if so deal with minor cases */
	if (!interrupts.Stall && !interrupts.BabbleError &&
		!interrupts.FrameOverrun) {									// No transmission error
		/* If endpoint NAK nothing wrong just demanding a retry */
		if (interrupts.NegativeAcknowledgement)						// Endpoint device NAK ..nothing wrong
			return ErrorTransmission;								// Simple tranmission error .. resend
		/* Next deal with device not ready case */
		if (interrupts.NotYet)
			return ErrorTransmission;								// Endpoint device not yet ... resend
		return ErrorTimeout;										// Let guess program just timed out
	}
	/* Everything else updates global count as it is serious */
	sendCtrl->GlobalTries++;										// Increment global tries
																	/* If global tries reaches 3 .. its a fatal error */
	if (sendCtrl->GlobalTries == 3) {								// Global tries has reached 3
		sendCtrl->ActionRetry = false;								// Clear retry action flag .. it's fatal
		sendCtrl->ActionResendSplit = false;						// Clear retyr sending split again .. it's fatal
		sendCtrl->ActionFatalError = true;							// This is a fatal error to many global errors
		return ErrorTransmission;									// Transmission error
	}
	/* Deal with stall */	
	if (interrupts.Stall) {											// Stall signalled .. device endpoint problem
		return ErrorStall;											// Return the stall error
	}
	/* Deal with true transmission errors */
	if ((interrupts.BabbleError) ||									// Bable error is a packet transmission problem
		(interrupts.FrameOverrun) ||								// Frame overrun error means stop bit failed at packet end
		(interrupts.TransactionError))								
	{
		return ErrorTransmission;									// Transmission error
	}
	return ErrorGeneral;											// If we get here then no idea why error occured (probably program error)
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
	uint64_t ticksTimeout = Timer::GetPerformanceTicksForUs(timeout);
	uint64_t original_tick = Timer::GetPerformanceCounter();
	for (;;) {
		Timer::Delay(100);
		tempInt = *DWC_HOST_CHANNEL_Interrupt[channel];
		if (tempInt.Halt || Timer::GetPerformanceCounter() - original_tick > ticksTimeout)
		{
			return tempInt;
		}
	}
}

/*-INTERNAL: HCDChannelTransfer----------------------------------------------
 Sends/recieves data from the given buffer and size directed by pipe settings.
 19Feb17 LdB
 --------------------------------------------------------------------------*/
RESULT HCDChannelTransfer(const struct UsbPipe pipe, const struct UsbPipeControl pipectrl, uint8_t* buffer, uint32_t& bufferLength, PacketId packetId) 
{
	LOG_DEBUG("HCD: Channel %u %s transfer, length %d, packetId %d, address %u, endpoint %u, type %u%s",
		pipectrl.Channel, pipectrl.Direction == USB_DIRECTION_IN ? "in" : "out", bufferLength, packetId, pipe.Number, pipe.EndPoint, pipectrl.Type,
		pipectrl.Direction == USB_DIRECTION_IN ? "\n" : ", "
	);
	if (bufferLength >= 8 && pipectrl.Direction == USB_DIRECTION_OUT)
	{
		LOG_DEBUG("Data = 0x%08X'%08X\n", ((uint32_t*)buffer)[1], ((uint32_t*)buffer)[0]);
	}

	RESULT result;
	struct ChannelInterrupts tempInt;
	struct UsbSendControl sendCtrl = { 0 };							// Zero send control structure
	uint32_t offset = 0;											// Zero transfer position 
	uint16_t maxPacketSize;
	if (pipectrl.Channel > (*DWC_CORE_HARDWARE1).HostChannelCount) {
		LOG("HCD: Channel %d is not available on this host.\n", pipectrl.Channel);
		return ErrorArgument;
	}
	// Convert to number
	maxPacketSize = SizeToNumber(pipe.MaxSize);						// Convert pipe packet size to integer

	/* Clear all existing interrupts. */
	DWC_HOST_CHANNEL_Interrupt[pipectrl.Channel] = 0xFFFFFFFF;// Clear all interrupts
	DWC_HOST_CHANNEL_InterruptMask[pipectrl.Channel] = 0x0;   // Clear all interrupt masks

	/* Program the channel. */
	struct HostChannelCharacteristic tempChar = { 0 };
	tempChar.device_address = pipe.Number;							// Set host channel address
	tempChar.endpoint_number = pipe.EndPoint;						// Set host channel endpoint
	tempChar.endpoint_direction = pipectrl.Direction;				// Set host channel direction
	tempChar.low_speed = pipe.Speed == USB_SPEED_LOW ? true : false;// Set host channel speed
	tempChar.endpoint_type = pipectrl.Type;							// Set host channel packet type
	tempChar.max_packet_size = maxPacketSize;						// Set host channel max packet size
	tempChar.channel_enable = false;								// Clear enable host channel
	tempChar.channel_disable = false;								// Clear disable host channel
	DWC_HOST_CHANNEL_Characteristic[pipectrl.Channel] = tempChar;	// Write those value to host characteristics

	/* Clear and setup split control to low speed devices */
	struct HostChannelSplitControl tempSplit = { 0 };
	if (pipe.Speed != USB_SPEED_HIGH) {								// If not high speed
		LOG_DEBUG("Setting split control, addr: %i port: %i, packetSize: PacketSize: %i\n",
			pipe.lowSpeedNodePoint, pipe.lowSpeedNodePort, maxPacketSize);
		tempSplit.split_enable = true;								// Enable split
		tempSplit.hub_address = pipe.lowSpeedNodePoint;				// Set the hub address to act as node
		tempSplit.port_address = pipe.lowSpeedNodePort;				// Set the hub port address
	}
	DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel] = tempSplit;		// Write channel split control

	/* Set transfer size. */
	struct HostTransferSize tempXfer = { 0 };
	tempXfer.size = bufferLength;									// Set transfer length
	if (pipe.Speed == USB_SPEED_LOW) tempXfer.packet_count = (bufferLength + 7) / 8;
	else tempXfer.packet_count = (bufferLength + maxPacketSize - 1) / maxPacketSize;
	if (tempXfer.packet_count == 0) tempXfer.packet_count = 1;		// Make sure packet count is not zero
	tempXfer.packet_id = packetId;									// Set the packet ID
	DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel] = tempXfer;		// Set the transfer size

	sendCtrl.PacketTries = 0;										// Zero packet tries
	do {

		// Clear any left over channel interrupts
		DWC_HOST_CHANNEL_Interrupt[pipectrl.Channel] = 0xFFFFFFFF;
		DWC_HOST_CHANNEL_InterruptMask[pipectrl.Channel] = 0x0;

		// Clear any left over split
		tempSplit = *DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel];	// Read split control register
		tempSplit.complete_split = false;							// Clear complete split
		DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel] = tempSplit;	// Write split register back

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
			return ErrorTimeout;									// Return timeout error
		}
		LOG_DEBUG("HCD: Channel %u transmission result: 0x%08X\n", pipectrl.Channel, tempInt.Raw32);

		tempSplit = *DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel];	// Fetch the split details
		result = HCDCheckErrorAndAction(tempInt,
			tempSplit.split_enable, &sendCtrl);						// Check transmisson RESULT and set action flags
		if (result) LOG("Result: %i Action: 0x%08x tempInt: 0x%08x tempSplit: 0x%08x Bytes sent: %i\n",
			result, (unsigned int)sendCtrl.Raw32, (unsigned int)tempInt.Raw32, 
			(unsigned int)tempSplit.Raw32, result ? 0 : (*DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel]).size);
		if (sendCtrl.ActionFatalError) return result;				// Fatal error occured we need to bail

		sendCtrl.SplitTries = 0;									// Zero split tries count
		while (sendCtrl.ActionResendSplit) {						// Decision was made to resend split
			/* Clear channel interrupts */
			DWC_HOST_CHANNEL_Interrupt[pipectrl.Channel] = 0xFFFFFFFF;
			DWC_HOST_CHANNEL_InterruptMask[pipectrl.Channel] = 0x0;

			/* Set we are completing the split */
			tempSplit = *DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel];
			tempSplit.complete_split = true;						// Set complete split flag
			DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel] = tempSplit;

			/* Launch transmission */
			tempChar = *DWC_HOST_CHANNEL_Characteristic[pipectrl.Channel];
			tempChar.channel_enable = true;
			tempChar.channel_disable = false;
			DWC_HOST_CHANNEL_Characteristic[pipectrl.Channel] = tempChar;

			// Polling wait on transmission only option right now .. other options soon :-)
			tempInt = HCDWaitOnTransmissionResult(5000, pipectrl.Channel);
			if (!tempInt.Halt)
			{
				LOG("HCD: Request split completion on channel:%i has timed out.\n", pipectrl.Channel);// Log error
				return ErrorTimeout;								// Return timeout error
			}
			LOG_DEBUG("HCD: Channel %u transmission result: 0x%08X\n", pipectrl.Channel, tempInt.Raw32);

			tempSplit = *DWC_HOST_CHANNEL_SplitCtrl[pipectrl.Channel];// Fetch the split details again
			result = HCDCheckErrorAndAction(tempInt,
				tempSplit.split_enable, &sendCtrl);					// Check RESULT of split resend and set action flags
			//if (result) LOG("Result: %i Action: 0x%08lx tempInt: 0x%08lx tempSplit: 0x%08lx Bytes sent: %i\n",
			//	result, sendCtrl.RawUsbSendContol, tempInt.RawInterrupt, tempSplit.RawSplitControl, RESULT ? 0 : DWC_HOST_CHANNEL[pipectrl.Channel].TransferSize.TransferSize);
			if (sendCtrl.ActionFatalError) return result;			// Fatal error occured bail
			if (sendCtrl.LongerDelay) Timer::Delay(10000);			// Not yet response slower delay
				else Timer::Delay(2500);								// Small delay between split resends
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

	} while ((*DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel]).packet_count > 0);// Full data not sent

	if (pipectrl.Direction == USB_DIRECTION_IN)
	{
		bufferLength -= (*DWC_HOST_CHANNEL_TransferSize[pipectrl.Channel]).size;
	}

	return OK;														// Return success as data must have been sent
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
RESULT HCDInitialise() {
	chfree = (1 << DWC_NUM_CHANNELS) - 1;							// Set the channel free bit masks

	uint32_t VendorId = *DWC_CORE_VENDORID;							// Read the vendor ID
	uint32_t UserId = *DWC_CORE_USERID;								// Read the user ID
	if ((VendorId & 0xfffff000) != 0x4f542000) {					// 'OT'2 
		LOG("HCD: Hardware: %c%c%x.%x%x%x (BCM%.5x). Driver incompatible. Expected OT2.xxx (BCM2708x).\n",
			(char)((VendorId >> 24) & 0xff), (char)((VendorId >> 16) & 0xff),
			(unsigned int)((VendorId >> 12) & 0xf), (unsigned int)((VendorId >> 8) & 0xf),
			(unsigned int)((VendorId >> 4) & 0xf), (unsigned int)((VendorId >> 0) & 0xf),
			(unsigned int)((UserId >> 12) & 0xFFFFF));
		return ErrorIncompatible;
	} else {
		LOG("HCD: Hardware: %c%c%x.%x%x%x (BCM%.5x).\n",
			(char)((VendorId >> 24) & 0xff),(char)((VendorId >> 16) & 0xff),
			(unsigned int)((VendorId >> 12) & 0xf), (unsigned int)((VendorId >> 8) & 0xf),
			(unsigned int)((VendorId >> 4) & 0xf), (unsigned int)((VendorId >> 0) & 0xf),
			(unsigned int)((UserId >> 12) & 0xFFFFF));
	}

	if ((*DWC_CORE_HARDWARE1).Architecture != InternalDma) {			// We only allow DMA transfer
		LOG("HCD: Host architecture does not support Internal DMA\n");
		return ErrorIncompatible;									// Return hardware incompatible
	}

	if ((*DWC_CORE_HARDWARE1).HighSpeedPhysical == NotSupported) {		// We need high speed transfers
		LOG("HCD: High speed physical unsupported\n");
		return ErrorIncompatible;									// Return hardware incompatible
	}

	struct CoreAhb tempAhb = *DWC_CORE_AHB;							// Read the AHB register to temp
	tempAhb.InterruptEnable = false;								// Clear interrupt enable bit
	DWC_CORE_AHB = tempAhb;										// Write temp back to AHB register
	DWC_CORE_INTERRUPTMASK = 0;								// Clear all interrupt masks

	if (PowerOnUsb() != OK) {										// Power up the USB hardware
		LOG("HCD: Failed to power on USB Host Controller.\n");		// Log failed to start power up
		return ErrorIncompatible;									// Return hardware incompatible
	}
	return OK;														// Return success
}
