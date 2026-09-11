#include "SdCard.h"

#include "Cpu.h"
#include "Gpio.h"
#include "Mmio.h"
#include "Uart.h"
#include "Mailbox.h"

#include <BootLib/RegisterProxy.h>

#include <fmt/format.h>

union SdCard::Registers
{
    BootLib::Register<uint32_t, 0x00> ARG2       ;
    BootLib::Register<uint32_t, 0x04> BLKSIZECNT ;
    BootLib::Register<uint32_t, 0x08> ARG1       ;
    BootLib::Register<uint32_t, 0x0C> CMDTM      ;
    BootLib::Register<uint32_t, 0x10> RESP0      ;
    BootLib::Register<uint32_t, 0x14> RESP1      ;
    BootLib::Register<uint32_t, 0x18> RESP2      ;
    BootLib::Register<uint32_t, 0x1C> RESP3      ;
    BootLib::Register<uint32_t, 0x20> DATA       ;
    BootLib::Register<uint32_t, 0x24> STATUS     ;
    BootLib::Register<uint32_t, 0x28> CONTROL0   ;
    BootLib::Register<uint32_t, 0x2C> CONTROL1   ;
    BootLib::Register<uint32_t, 0x30> INTERRUPT  ;
    BootLib::Register<uint32_t, 0x34> INT_MASK   ;
    BootLib::Register<uint32_t, 0x38> INT_EN     ;
    BootLib::Register<uint32_t, 0x3C> CONTROL2   ;
    BootLib::Register<uint32_t, 0xFC> SLOTISR_VER;
};

namespace
{

#define SD_OK                0
#define SD_TIMEOUT          -1
#define SD_ERROR            -2

// command flags
#define CMD_NEED_APP        0x80000000
#define CMD_RSPNS_48        0x00020000
#define CMD_ERRORS_MASK     0xfff9c004
#define CMD_RCA_MASK        0xffff0000

// COMMANDs
#define CMD_GO_IDLE         0x00000000
#define CMD_ALL_SEND_CID    0x02010000
#define CMD_SEND_REL_ADDR   0x03020000
#define CMD_CARD_SELECT     0x07030000
#define CMD_SEND_IF_COND    0x08020000
#define CMD_STOP_TRANS      0x0C030000
#define CMD_READ_SINGLE     0x11220010
#define CMD_READ_MULTI      0x12220032
#define CMD_SET_BLOCKCNT    0x17020000
#define CMD_APP_CMD         0x37000000
#define CMD_SET_BUS_WIDTH   (0x06020000|CMD_NEED_APP)
#define CMD_SEND_OP_COND    (0x29020000|CMD_NEED_APP)
#define CMD_SEND_SCR        (0x33220010|CMD_NEED_APP)

// STATUS register settings
#define SR_READ_AVAILABLE   0x00000800
#define SR_DAT_INHIBIT      0x00000002
#define SR_CMD_INHIBIT      0x00000001
#define SR_APP_CMD          0x00000020

// INTERRUPT register settings
#define INT_DATA_TIMEOUT    0x00100000
#define INT_CMD_TIMEOUT     0x00010000
#define INT_READ_RDY        0x00000020
#define INT_CMD_DONE        0x00000001

#define INT_ERROR_MASK      0x017E8000

// CONTROL register settings
#define C0_SPI_MODE_EN      0x00100000
#define C0_VDD1_BUS_POWER_3_3V 0x00000f00
#define C0_HCTL_HS_EN       0x00000004
#define C0_HCTL_DWITDH      0x00000002

#define C1_SRST_DATA        0x04000000
#define C1_SRST_CMD         0x02000000
#define C1_SRST_HC          0x01000000
#define C1_TOUNIT_DIS       0x000f0000
#define C1_TOUNIT_MAX       0x000e0000
#define C1_CLK_GENSEL       0x00000020
#define C1_CLK_EN           0x00000004
#define C1_CLK_STABLE       0x00000002
#define C1_CLK_INTLEN       0x00000001

// SLOTISR_VER values
#define HOST_SPEC_NUM       0x00ff0000
#define HOST_SPEC_NUM_SHIFT 16
#define HOST_SPEC_V3        2
#define HOST_SPEC_V2        1
#define HOST_SPEC_V1        0

// SCR flags
#define SCR_SD_BUS_WIDTH_4  0x00000400
#define SCR_SUPP_SET_BLKCNT 0x02000000
// added by my driver
#define SCR_SUPP_CCS        0x00000001

#define ACMD41_VOLTAGE      0x00ff8000
#define ACMD41_CMD_COMPLETE 0x80000000
#define ACMD41_CMD_CCS      0x40000000
#define ACMD41_ARG_HC       0x51ff8000

unsigned long sd_scr[2], sd_ocr, sd_rca, sd_err, sd_hv;

/**
 * Wait for data or command ready
 */
int sd_status(auto& registers, unsigned int mask)
{
    int cnt = 500000; while((registers.STATUS & mask) && !(registers.INTERRUPT & INT_ERROR_MASK) && cnt--) Cpu::Delay(1ms);
    return (cnt <= 0 || (registers.INTERRUPT & INT_ERROR_MASK)) ? SD_ERROR : SD_OK;
}

/**
 * Wait for interrupt
 */
int sd_int(auto& registers, unsigned int mask)
{
    unsigned int r, m=mask | INT_ERROR_MASK;
    int cnt = 1000000; while(!(registers.INTERRUPT & m) && cnt--) Cpu::Delay(1ms);
    r=registers.INTERRUPT;
    if(cnt<=0 || (r & INT_CMD_TIMEOUT) || (r & INT_DATA_TIMEOUT) ) { registers.INTERRUPT=r; return SD_TIMEOUT; } else
    if(r & INT_ERROR_MASK) { registers.INTERRUPT=r; return SD_ERROR; }
    registers.INTERRUPT=mask;
    return 0;
}

/**
 * Send a command
 */
int sd_cmd(auto& registers, unsigned int code, unsigned int arg)
{
    int r=0;
    sd_err=SD_OK;
    if(code&CMD_NEED_APP) {
        r=sd_cmd(registers, CMD_APP_CMD|(sd_rca?CMD_RSPNS_48:0),sd_rca);
        if(sd_rca && !r) { fmt::println("ERROR: failed to send SD APP command"); sd_err=SD_ERROR;return 0;}
        code &= ~CMD_NEED_APP;
    }
    if(sd_status(registers, SR_CMD_INHIBIT)) { fmt::println("ERROR: EMMC busy"); sd_err= SD_TIMEOUT;return 0;}
    fmt::println("EMMC: Sending command {:#x} arg {:#x}", code, arg);
    registers.INTERRUPT = registers.INTERRUPT.get();
    registers.ARG1=arg;
    registers.CMDTM=code;
    if(code==CMD_SEND_OP_COND) Cpu::Delay(1000ms); else
    if(code==CMD_SEND_IF_COND || code==CMD_APP_CMD) Cpu::Delay(100ms);
    if((r=sd_int(registers, INT_CMD_DONE))) {fmt::println("ERROR: failed to send EMMC command");sd_err=r;return 0;}
    r=registers.RESP0;
    if(code==CMD_GO_IDLE || code==CMD_APP_CMD) return 0; else
    if(code==(CMD_APP_CMD|CMD_RSPNS_48)) return r&SR_APP_CMD; else
    if(code==CMD_SEND_OP_COND) return r; else
    if(code==CMD_SEND_IF_COND) return r==arg? SD_OK : SD_ERROR; else
    if(code==CMD_ALL_SEND_CID) {r|=registers.RESP3; r|=registers.RESP2; r|=registers.RESP1; return r; } else
    if(code==CMD_SEND_REL_ADDR) {
        sd_err=(((r&0x1fff))|((r&0x2000)<<6)|((r&0x4000)<<8)|((r&0x8000)<<8))&CMD_ERRORS_MASK;
        return r&CMD_RCA_MASK;
    }
    return r&CMD_ERRORS_MASK;
    // make gcc happy
    return 0;
}

/**
 * set SD clock to frequency in Hz
 */
int sd_clk(auto& registers, unsigned int f)
{
    unsigned int d;
    //unsigned int c=41'666'666/f;
    unsigned int coreFrequency = BootLib::Cpu::IsRpi4() ? 500'000'000u : 500'000'000u;
    unsigned int c = coreFrequency / (f * 6u);
    unsigned int x,s=32,h=0;
    int cnt = 100000;
    while((registers.STATUS & (SR_CMD_INHIBIT|SR_DAT_INHIBIT)) && cnt--) Cpu::Delay(1ms);
    if(cnt<=0) {
        fmt::println("ERROR: timeout waiting for inhibit flag");
        return SD_ERROR;
    }

    registers.CONTROL1 &= ~C1_CLK_EN;
    Cpu::Delay(10ms);
    x=c-1; if(!x) s=0; else {
        if(!(x & 0xffff0000u)) { x <<= 16; s -= 16; }
        if(!(x & 0xff000000u)) { x <<= 8;  s -= 8; }
        if(!(x & 0xf0000000u)) { x <<= 4;  s -= 4; }
        if(!(x & 0xc0000000u)) { x <<= 2;  s -= 2; }
        if(!(x & 0x80000000u)) { x <<= 1;  s -= 1; }
        if(s>0) s--;
        if(s>7) s=7;
    }
    if(sd_hv>HOST_SPEC_V2) d=c; else d=(1<<s);
    if(d<=2) {d=2;s=0;}
    fmt::println("sd_clk divisor {:#x}, shift {:#x}", d, s);
    if(sd_hv>HOST_SPEC_V2) h=(d&0x300)>>2;
    d=(((d&0x0ff)<<8)|h);
    registers.CONTROL1=(registers.CONTROL1&0xffff003f)|d; Cpu::Delay(10ms);
    registers.CONTROL1 |= C1_CLK_EN;
    Cpu::Delay(10ms);
    cnt=10000; while(!(registers.CONTROL1 & C1_CLK_STABLE) && cnt--) Cpu::Delay(10ms);
    if(cnt<=0) {
        fmt::println("ERROR: failed to get stable clock");
        return SD_ERROR;
    }
    return SD_OK;
}

}
// unnamed namespace

bool SdCard::Init()
{
    long r;
    int cnt;
    long ccs=0;

    if (BootLib::Cpu::IsRpi4())
    {
        //Mailbox::TagMessage<Mailbox::Tag::SET_GPIO_PIN, 2> setEMMC2Voltage{{ 128 + 4, 0 }};
        //Mailbox::SendTags(setEMMC2Voltage);

        // We're using the legacy SDHCI device.
        // Note that this disables access to WiFi.
        *(uint32_t volatile*)(Mmio::Base + 0x20'00D0u) |= 2u;

        //registers.CONTROL0 |= C0_VDD1_BUS_POWER_3_3V; // Set voltage to 3.3V
    }
    else
    {
        // GPIO_CD
        Gpio::SetFunction(47, Gpio::Function::Alt3);
        Gpio::SetPullUpDown(47, Gpio::PullUpDown::PullUp);
        Gpio::SetHighDetectEnable(47, true);

        // GPIO_CLK, GPIO_CMD
        Gpio::SetFunction(48, Gpio::Function::Alt3);
        Gpio::SetFunction(49, Gpio::Function::Alt3);
        Gpio::SetPullUpDown(48, Gpio::PullUpDown::PullUp);
        Gpio::SetPullUpDown(49, Gpio::PullUpDown::PullUp);

        // GPIO_DAT0, GPIO_DAT1, GPIO_DAT2, GPIO_DAT3
        Gpio::SetFunction(50, Gpio::Function::Alt3);
        Gpio::SetFunction(51, Gpio::Function::Alt3);
        Gpio::SetFunction(52, Gpio::Function::Alt3);
        Gpio::SetFunction(53, Gpio::Function::Alt3);
        Gpio::SetPullUpDown(50, Gpio::PullUpDown::PullUp);
        Gpio::SetPullUpDown(51, Gpio::PullUpDown::PullUp);
        Gpio::SetPullUpDown(52, Gpio::PullUpDown::PullUp);
        Gpio::SetPullUpDown(53, Gpio::PullUpDown::PullUp);
    }

    fmt::println("EMMC: GPIO set up");

    sd_hv = (registers.SLOTISR_VER & HOST_SPEC_NUM) >> HOST_SPEC_NUM_SHIFT;
    fmt::println("EMMC: Spec {}", sd_hv);

    // Reset the card.
    registers.CONTROL0 = 0;
    registers.CONTROL1 |= C1_SRST_HC;

    cnt=10000;
    do
    {
        Cpu::Delay(10ms);
    }
    while((registers.CONTROL1 & C1_SRST_HC) && cnt--);

    if (cnt <= 0) {
        fmt::println("ERROR: failed to reset EMMC");
    }
    fmt::println("EMMC: reset OK");

    registers.CONTROL1 |= C1_CLK_INTLEN | C1_TOUNIT_MAX;
    Cpu::Delay(10ms);

    // Set clock to setup frequency.
    if (sd_clk(registers, 400'000)) return false;

    registers.INT_EN   = 0xffffffff;
    registers.INT_MASK = 0xffffffff;

    sd_scr[0]=0;
    sd_scr[1]=0;
    sd_rca=0;
    sd_err=0;

    sd_cmd(registers, CMD_GO_IDLE,0);
    if (sd_err)
    {
        fmt::println("Error: {:#x}", sd_err);
        return false;
    }

    sd_cmd(registers, CMD_SEND_IF_COND,0x000001AA);
    if (sd_err)
    {
        fmt::println("Error: {:#x}", sd_err);
        return false;
    }

    {
        cnt=6;
        uint64_t result = 0;
        while(!(result & ACMD41_CMD_COMPLETE) && cnt--)
        {
            Cpu::Delay(400us);
            result = sd_cmd(registers, CMD_SEND_OP_COND,ACMD41_ARG_HC);
            fmt::print("EMMC: CMD_SEND_OP_COND returned ");
            if (result & ACMD41_CMD_COMPLETE)
                fmt::print("COMPLETE ");
            if (result & ACMD41_VOLTAGE)
                fmt::print("VOLTAGE ");
            if (result & ACMD41_CMD_CCS)
                fmt::print("CCS ");
            fmt::println("{:#x}", result);
            if (sd_err != SD_TIMEOUT && sd_err != SD_OK )
            {
                fmt::println("ERROR: EMMC ACMD41 returned error {}", sd_err);
                return false;
            }
        }
        fmt::println("EMMC: CMD_SEND_OP_COND completed after {} attempts", 6u - cnt);
        if (!(result & ACMD41_CMD_COMPLETE) || !cnt) return false; //SD_TIMEOUT;
        if (!(result & ACMD41_VOLTAGE)) return false; //SD_ERROR;
        if (result & ACMD41_CMD_CCS) ccs = SCR_SUPP_CCS;
    }

    sd_cmd(registers, CMD_ALL_SEND_CID, 0);

    sd_rca = sd_cmd(registers, CMD_SEND_REL_ADDR, 0);
    fmt::println("EMMC: CMD_SEND_REL_ADDR returned {:#x}", sd_rca);
    if (sd_err) return false;

    if (sd_clk(registers, 25'000'000)) return false;

    sd_cmd(registers, CMD_CARD_SELECT, sd_rca);
    if (sd_err) return false;

    if (sd_status(registers, SR_DAT_INHIBIT)) return false; // SD_TIMEOUT;
    registers.BLKSIZECNT = (1<<16) | 8;
    sd_cmd(registers, CMD_SEND_SCR, 0);
    if (sd_err) return false;
    if (sd_int(registers, INT_READ_RDY)) return false; //SD_TIMEOUT;

    r=0;
    cnt=100000;
    while (r < 2 && cnt)
    {
        if (registers.STATUS & SR_READ_AVAILABLE)
            sd_scr[r++] = registers.DATA;
        else
            Cpu::Delay(1ms);
    }
    if (r != 2) return false; // SD_TIMEOUT;
    if (sd_scr[0] & SCR_SD_BUS_WIDTH_4)
    {
        sd_cmd(registers, CMD_SET_BUS_WIDTH, sd_rca | 2);
        if (sd_err) return false;
        registers.CONTROL0 |= C0_HCTL_DWITDH;
    }
    // add software flag
    fmt::print("EMMC: supports ");
    if (sd_scr[0] & SCR_SUPP_SET_BLKCNT)
        fmt::print("SET_BLKCNT ");
    if (ccs)
        fmt::print("CCS ");
    fmt::println("");
    sd_scr[0] &= ~SCR_SUPP_CCS;
    sd_scr[0] |= ccs;
    return true;
}

/**
 * read a block from sd card and return the number of bytes read
 * returns 0 on error.
 */
bool SdCard::ReadBlock(uint32_t lba, void* buffer, uint32_t count)
{
    int r,c=0,d;
    if(count < 1) count = 1;
    fmt::println("sd_readblock lba {:#x} num {:#x}", lba, count);
    if(sd_status(registers, SR_DAT_INHIBIT)) {sd_err=SD_TIMEOUT; return false;}
    unsigned int *buf=(unsigned int *)buffer;
    if(sd_scr[0] & SCR_SUPP_CCS) {
        if(count > 1 && (sd_scr[0] & SCR_SUPP_SET_BLKCNT)) {
            sd_cmd(registers, CMD_SET_BLOCKCNT,count);
            if(sd_err) return false;
        }
        registers.BLKSIZECNT = (count << 16) | 512;
        sd_cmd(registers, count == 1 ? CMD_READ_SINGLE : CMD_READ_MULTI,lba);
        if(sd_err) return false;
    } else {
        registers.BLKSIZECNT = (1 << 16) | 512;
    }
    while( c < count ) {
        if(!(sd_scr[0] & SCR_SUPP_CCS)) {
            sd_cmd(registers, CMD_READ_SINGLE,(lba+c)*512);
            if(sd_err) return false;
        }
        if((r=sd_int(registers, INT_READ_RDY))){fmt::println("\rERROR: Timeout waiting for ready to read");sd_err=r;return false;}
        for(d=0;d<128;d++) buf[d] = registers.DATA;
        c++; buf+=128;
    }
    if( count > 1 && !(sd_scr[0] & SCR_SUPP_SET_BLKCNT) && (sd_scr[0] & SCR_SUPP_CCS)) sd_cmd(registers, CMD_STOP_TRANS,0);
    return sd_err == SD_OK && c == count;
}
