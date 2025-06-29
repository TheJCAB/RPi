// The interrupt controller routes IRQs or FIQs to core 0

struct Basic_IRQs
{
    uint32_t ARM_Timer      : 1; // @0
    uint32_t ARM_Mailbox    : 1; // @1
    uint32_t ARM_Doorbell_0 : 1; // @2
    uint32_t ARM_Doorbell_1 : 1; // @3
    uint32_t GPU_0_Halted   : 1; // @4
    uint32_t GPU_1_Halted   : 1; // @5
    uint32_t Access_Error_1 : 1; // @6
    uint32_t Access_Error_0 : 1; // @7
    uint32_t unused         : 24; // @8-31
};

// IRQs 1
//0       ST_C0
//1       ST_C1
//2       ST_C2
//3       ST_C3
//4       codec0 (vce? h264?)
//5       codec1
//6       codec2
//7       jpeg
//8       isp
//9       usb
//10      v3d
//11      transposer
//12      multicore sync 0
//13      multicore sync 1
//14      multicore sync 2
//15      multicore sync 3
//16      dma0
//17      dma1
//18      dma2
//19      dma3
//20      dma4
//21      dma5
//22      dma6
//23      dma7
//24      dma8
//25      dma9
//26      dma10
//27      dma11/dma12/dma13/dma14
//28      dma-all
//29      aux int
//30      arm
//31      dma-vpu

// IRQs 2
//32      hostport
//33      videoscaler               (HVS)
//34      ccp2tx
//35      sdc
//36      dsi0
//37      axe
//38      cam0
//39      cam1
//40      hdmi0
//41      hdmi1
//42      pixelvalve1 (PV2!!)
//43      i2c_spi_slv_int
//44      dsi1
//45      pwa0    (PV0)
//46      pwa1    (PV1)
//47      cpr
//48      smi
//49      gpio_int[0]
//50      gpio_int[1]
//51      gpio_int[2]
//52      gpio_int[3]
//53      i2c_int
//54      spi_int
//55      i2s_pcm_int
//56      sdio
//57      uart_int (PL011?)
//58      slimbus
//59      vec
//60      cpf
//61      rng
//62      asdio
//63      avspmon

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

}
// namespace Timer
