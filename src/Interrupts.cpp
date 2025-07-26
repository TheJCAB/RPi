// The interrupt controller routes IRQs or FIQs to core 0

#include "Interrupts.h"
#include "Scheduler.h"


#include "Cpu.h"
#include "Mmio.h"
#include "Timer.h"
#include "Uart.h"
#include "Debugger.h"

#include <stdint.h>

namespace Interrupts
{

union BasicIrqPending
{
    struct
    {
        uint32_t ARM_Timer      : 1; // @0
        uint32_t ARM_Mailbox    : 1; // @1
        uint32_t ARM_Doorbell_0 : 1; // @2
        uint32_t ARM_Doorbell_1 : 1; // @3
        uint32_t GPU_0_Halted   : 1; // @4
        uint32_t GPU_1_Halted   : 1; // @5
        uint32_t Access_Error_1 : 1; // @6
        uint32_t Access_Error_0 : 1; // @7
        uint32_t IRQs1          : 1; // @8 Some IRQs are in table 1 that don't have a copy here.
        uint32_t IRQs2          : 1; // @9
        uint32_t Jpeg           : 1; // @10 Copy of IRQ 7 which is USB
        uint32_t USB            : 1; // @11 Copy of IRQ 9 which is USB
        uint32_t unused         : 20; // @12-31
    };
    uint32_t Raw32; // Union to access all 32 bits as a uint32_t

    explicit operator bool() const { return Raw32 != 0; }
};

// Like the pending ones above, but without any of the IRQ1/2 references.
// IRQ1/2 are enabled/disabled using the corresponding IRQ1/2 registers.
union BasicIrqEnables
{
    struct
    {
        uint32_t ARM_Timer      :  1; // @0
        uint32_t ARM_Mailbox    :  1; // @1
        uint32_t ARM_Doorbell_0 :  1; // @2
        uint32_t ARM_Doorbell_1 :  1; // @3
        uint32_t GPU_0_Halted   :  1; // @4
        uint32_t GPU_1_Halted   :  1; // @5
        uint32_t Access_Error_1 :  1; // @6
        uint32_t Access_Error_0 :  1; // @7
        uint32_t unused         : 24; // @12-31
    };
    uint32_t Raw32; // Union to access all 32 bits as a uint32_t

    explicit operator bool() const { return Raw32 != 0; }
};

union Irq1
{
    struct
    {
        bool ST_C0          : 1; // @0
        bool ST_C1          : 1; // @1
        bool ST_C2          : 1; // @2
        bool ST_C3          : 1; // @3
        bool Codec0         : 1; // @4 (vce? h264?)
        bool Codec1         : 1; // @5
        bool Codec2         : 1; // @6
        bool Jpeg           : 1; // @7
        bool Isp            : 1; // @8
        bool USB            : 1; // @9
        bool V3d            : 1; // @10
        bool Transposer     : 1; // @11
        bool MulticoreSync0 : 1; // @12
        bool MulticoreSync1 : 1; // @13
        bool MulticoreSync2 : 1; // @14
        bool MulticoreSync3 : 1; // @15
        bool Dma0           : 1; // @16
        bool Dma1           : 1; // @17
        bool Dma2           : 1; // @18
        bool Dma3           : 1; // @19
        bool Dma4           : 1; // @20
        bool Dma5           : 1; // @21
        bool Dma6           : 1; // @22
        bool Dma7           : 1; // @23
        bool Dma8           : 1; // @24
        bool Dma9           : 1; // @25
        bool Dma10          : 1; // @26
        bool Dma11_14       : 1; // @27
        bool DmaAll         : 1; // @28
        bool AuxInt         : 1; // @29
        bool ARM            : 1; // @30
        bool DmaVpu         : 1; // @31
    };
    uint32_t Raw32; // Union to access all 32 bits as a uint32_t

    explicit operator bool() const { return Raw32 != 0; }
};
    
union Irq2
{
    struct
    {
        bool Hostport     : 1; // @32                                    
        bool Videoscaler  : 1; // @33 (HVS)                                                       
        bool Ccp2tx       : 1; // @34                                
        bool Sdc          : 1; // @35                            
        bool Dsi0         : 1; // @36                                
        bool Axe          : 1; // @37                            
        bool Cam0         : 1; // @38                                
        bool Cam1         : 1; // @39                                
        bool Hdmi0        : 1; // @40                                
        bool Hdmi1        : 1; // @41                                
        bool Pixelvalve1  : 1; // @42 (PV2!!)                                           
        bool I2cSpiSlvInt : 1; // @43                                        
        bool Dsi1         : 1; // @44                                
        bool Pwa0         : 1; // @45 (PV0)                                       
        bool Pwa1         : 1; // @46 (PV1)                                       
        bool Cpr          : 1; // @47                            
        bool Smi          : 1; // @48                            
        bool GpioInt0     : 1; // @49                                    
        bool GpioInt1     : 1; // @50                                    
        bool GpioInt2     : 1; // @51                                    
        bool GpioInt3     : 1; // @52                                    
        bool I2cInt       : 1; // @53                                
        bool SpiInt       : 1; // @54                                
        bool I2sPcmInt    : 1; // @55                                    
        bool Sdio         : 1; // @56                                
        bool UartInt      : 1; // @57 (PL011?)                                           
        bool Slimbus      : 1; // @58                                
        bool Vec          : 1; // @59                            
        bool Cpf          : 1; // @60                            
        bool Rng          : 1; // @61                            
        bool Asdio        : 1; // @62                                
        bool Avspmon      : 1; // @63                                
    };
    uint32_t Raw32; // Union to access all 32 bits as a uint32_t

    explicit operator bool() const { return Raw32 != 0; }
};

struct FIQ_Control
{
    uint32_t source   : 7;  // Bits 0-6: IRQ source number (0-127)
    uint32_t enable   : 1;  // Bit 7: Enable FIQ (1=enable, 0=disable)
    uint32_t reserved : 24; // Bits 8-31: Reserved
};

constexpr uint32_t IRQ_basic_pending  = 0xB200u;
constexpr uint32_t IRQ_pending_1      = 0xB204u;
constexpr uint32_t IRQ_pending_2      = 0xB208u;
constexpr uint32_t FIQ_control        = 0xB20Cu;
constexpr uint32_t Enable_IRQs_1      = 0xB210u;
constexpr uint32_t Enable_IRQs_2      = 0xB214u;
constexpr uint32_t Enable_Basic_IRQs  = 0xB218u;
constexpr uint32_t Disable_IRQs_1     = 0xB21Cu;
constexpr uint32_t Disable_IRQs_2     = 0xB220u;
constexpr uint32_t Disable_Basic_IRQs = 0xB224u;

Mmio::BaseRegisterProxy<BasicIrqPending, IRQ_basic_pending > IrqBasicPending;
Mmio::BaseRegisterProxy<Irq1           , IRQ_pending_1     > IrqPending1;
Mmio::BaseRegisterProxy<Irq2           , IRQ_pending_2     > IrqPending2;
Mmio::BaseRegisterProxy<BasicIrqEnables, Enable_Basic_IRQs > EnableBasicIrq;
Mmio::BaseRegisterProxy<Irq1           , Enable_IRQs_1     > EnableIrq1;
Mmio::BaseRegisterProxy<Irq2           , Enable_IRQs_2     > EnableIrq2;
Mmio::BaseRegisterProxy<BasicIrqEnables, Disable_Basic_IRQs> DisableBasicIrq;
Mmio::BaseRegisterProxy<Irq1           , Disable_IRQs_1    > DisableIrq1;
Mmio::BaseRegisterProxy<Irq2           , Disable_IRQs_2    > DisableIrq2;

union CoreInterruptSource
{
    struct
    {
        bool CNTPSIRQ       : 1;
        bool CNTPNSIRQ      : 1;
        bool CNTHPIRQ       : 1;
        bool CNTVIRQ        : 1;
        bool Mailbox0       : 1;
        bool Mailbox1       : 1;
        bool Mailbox2       : 1;
        bool Mailbox3       : 1;
        bool GPU            : 1;
        bool PMU            : 1;
        bool AXIoutstanding : 1;
        bool LocalTimer     : 1;
        uint32_t Reserved   : 20; // The rest is unused.
    };
    uint32_t Raw32;

    explicit operator bool() const { return Raw32 != 0; }
};

union CoreInterruptRegisters
{
    BootLib::Register<CoreInterruptSource, 0x40> TimerControl;
    BootLib::Register<CoreInterruptSource, 0x50> MailboxControl;
    BootLib::Register<CoreInterruptSource, 0x60> IrqPending;
    BootLib::Register<CoreInterruptSource, 0x70> FiqPending;
};

HandlerFunction UsbHandler;

void EnableUsb(HandlerFunction handler)
{
    if (handler == nullptr)
    {
        DisableIrq1 = Irq1{ .USB = true };
    }
    else
    {
        EnableIrq1 = Irq1{ .USB = true };
    }
    UsbHandler = handler;
}

struct CoreInterrupts
{
    HandlerFunction VirtualTimerHandler;
};

CoreInterrupts CoreInterruptsData[4];

CoreInterruptRegisters& RefCoreInterruptRegisters(size_t coreId)
{
    return *reinterpret_cast<CoreInterruptRegisters*>(Mmio::QA7Base + 4 * coreId);
}

void EnableCoreVirtualTimerInterrupt(HandlerFunction handler)
{
    auto const coreId = Cpu::mpidr_el1->CoreId;

    CoreInterruptsData[coreId].VirtualTimerHandler = handler;

    // Enable the timer and unmask interrupt
    Cpu::cntv_ctl_el0 = {
        .Enable = 1,
        .IMASK = 0,
    };

    auto& registers = RefCoreInterruptRegisters(coreId);
    registers.TimerControl = [](auto&reg){
        reg.CNTVIRQ = true; // Enable the virtual timer interrupt
    };
}

void DisableCoreVirtualTimerInterrupt()
{
    auto const coreId = Cpu::mpidr_el1->CoreId;

    CoreInterruptsData[coreId].VirtualTimerHandler = nullptr;

    // Disable the timer and mask interrupt
    Cpu::cntv_ctl_el0 = {
        .Enable = 0,
        .IMASK = 1,
    };

    auto& registers = RefCoreInterruptRegisters(coreId);
    registers.TimerControl = [](auto&reg){
        reg.CNTVIRQ = false; // Disable the virtual timer interrupt
    };
}

extern "C" Spark InterruptDispatcher(ThreadContext* context, uint32_t code)
{
    auto const coreId = Cpu::mpidr_el1->CoreId;

    // If we get into a deadlock in the "user" code, we can use this to debug it.
    // This will tell us what is deadlocked. Just need to make sure it doesn't trigger until the deadlock happens.
    // Use contextCount in any way desired to achieve this goal.
    //static uint32_t contextCount = 0;
    //if (coreId == 0 && (++contextCount & 0x3F) == 0)
    //{
    //    Debugger::RawPrintThreadContext(context);
    //}

    auto& registers = RefCoreInterruptRegisters(coreId);

    uint32_t retryCount = 100;
    while (auto pendingCoreInterrupts = registers.IrqPending.get())
    {
        if (pendingCoreInterrupts.CNTVIRQ)
        {
            //Uart::Putc('@'); // Printf tracing of the handler.
            auto const spark = CoreInterruptsData[coreId].VirtualTimerHandler();
            if (spark)
            {
                return spark;
            }
        }

        // Ensure we don't just loop indefinitely if some interrupt is defined but not handled.
        if (--retryCount == 0)
        {
            Uart::Raw::Puts("Panic: Unhandled core interrupt: ");
            Uart::Raw::PutBin(pendingCoreInterrupts.Raw32);
            Uart::Raw::Puts("\n");
            Cpu::Halt();
        }
    }

    // If not core 0, return
    if (coreId > 0)
    {
        return {};
    }

    // Check for basic IRQs
    retryCount = 100;
    while (auto basicPending = IrqBasicPending.get())
    {
        Uart::Raw::Puts("Basic IRQs pending: ");
        Uart::Raw::PutHex(basicPending.Raw32);
        Uart::Raw::Puts("\n");

        // Handle basic IRQs
        if (basicPending.USB)
        {
            // Handle USB IRQ
            if (UsbHandler)
            {
                UsbHandler();
            }
        }

        // Check for IRQ1
        if (basicPending.IRQs1)
        {
            auto irq1 = IrqPending1.get();
            Uart::Raw::Puts("IRQs 1 pending: ");
            Uart::Raw::PutHex(irq1.Raw32);
            Uart::Raw::Puts("\n");

            // Handle IRQ1
        }

        // Check for IRQ2
        if (basicPending.IRQs2)
        {
            auto irq2 = IrqPending2.get();
            Uart::Raw::Puts("IRQs 2 pending: ");
            Uart::Raw::PutHex(irq2.Raw32);
            Uart::Raw::Puts("\n");

            // Handle IRQ2
        }

        if (--retryCount == 0)
        {
            Uart::Raw::Puts("Panic: Unhandled SoC interrupt: ");
            Uart::Raw::PutBin(basicPending.Raw32);
            Uart::Raw::Puts("\n");
            Cpu::Halt();
        }
    }

    return {};
}

/*
void SetPeriodicInterrupt(uint64_t us)
{
    Basic_IRQs armTimerBasicIrq{ .ARM_Timer = 1 };
    *(uint32_t volatile*)(Mmio::Base + Enable_Basic_IRQs) = *(uint32_t*)&armTimerBasicIrq;

    // Calculate timer interval in counter ticks
    //uint64_t interval = us * GetPerformanceFrequency() / 1'000'000u;

    *(uint32_t volatile*)(Mmio::Base + Timer_Load) = 0x400;
    time_ctrl_reg_t control
    {
        .unused              = 0,
        .Counter32Bit        = 1,
        .Prescale            = Clkdiv256,
        .unused1             = 0,
        .TimerIrqEnable      = 1,
        .unused2             = 0,
        .TimerEnable         = 1,
        .DbgKeepTimerRunning = 0,
        .CounterFreeRunning  = 0,
        .unused3             = 0,
        .FreeRunPreScaleDiv  = 0x3E, // Default pre-scaler value
        .reserved            = 0
    };
    *(uint32_t volatile*)(Mmio::Base + Timer_Control) = *(uint32_t*)&control;
}

// This should be called from the IRQ handler for the virtual timer
void HandlePeriodicInterrupt()
{
    if (*(uint32_t volatile*)(Mmio::Base + Timer_MaskedIRQ) & 0x1) {
        // Clear the interrupt
        *(uint32_t volatile*)(Mmio::Base + Timer_Clear) = 0x1;
    } else {
        // If the interrupt is not set, we can ignore it
        return;
    }

    // Acknowledge the interrupt by resetting the timer interval
    *(uint32_t volatile*)(Mmio::Base + Timer_Load) = 0x400;
    if (auto lockedStream = Uart::LockedStream(true)) {
        lockedStream.Puts("Periodic interrupt handled\n");
    }
}
*/

}
// namespace Interrupts
