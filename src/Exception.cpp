#include <stdint.h>
#include <stddef.h>

#include "Uart.h"
#include "Processor.h"

namespace Exception
{

void DataAbortException(uint8_t ec, uint32_t iss, uint32_t iss2, uint32_t far)
{
    Uart::Puts("Data ");
    bool isWrite = ((iss >> 6) & 1) == 1;;
    Uart::Puts(isWrite ? "(write)" : "(read)");
    Uart::Puts(" Abort exception, ");
    Uart::Puts(ec & 1 ? "same" : "lower");
    Uart::Puts(" level\n");
    if (iss & (1 << 10))
    {
        Uart::Puts("Accessing address: ");
        Uart::PutHex(far);
        Uart::Puts("\n");
    }
    else
    {
        Uart::Puts("No address given, FAR = ");
        Uart::PutHex(far);
        Uart::Puts("\n");
    }
    uint8_t const dfsc = iss & 0b11'1111;
    switch (dfsc)
    {
        case 0b00'0100:
        case 0b00'0101:
        case 0b00'0110:
        case 0b00'0111:
            Uart::Puts("Translation fault, level ");
            Uart::PutDec((uint8_t)(dfsc & 3));
            Uart::Puts(".\n");
            Processor::Halt();
        case 0b10'0001:
            Uart::Puts("Alignment fault.\n");
            Processor::Halt();
        default:
            Uart::Puts("Unknown data fault status code: ");
            Uart::PutBin(dfsc);
            Uart::Puts("\n");
            Processor::Halt();
    }
}

void SvcException(uint16_t imm, uint64_t lr)
{
    Uart::Puts("Supervisor Call (SVC) exception\n");
    Uart::Puts("Imm: ");
    Uart::PutDec(imm);
    Uart::Puts("\n");
    Uart::Puts("From address: ");
    Uart::PutHex(lr);
    Uart::Puts("\n");

    // Handle SVC here
}

void PutRawExceptionInfo(uint32_t code, uint64_t esr, uint64_t elr, uint64_t spsr, uint64_t far)
{
    Uart::Puts("Panic Exception Handler\n");
    Uart::Puts("Code: ");
    Uart::PutDec(code);
    Uart::Puts("\nESR : ");
    Uart::PutHex(esr);
    Uart::Puts("\nELR : ");
    Uart::PutHex(elr);
    Uart::Puts("\nSPSR: ");
    Uart::PutHex(spsr);
    Uart::Puts("\nFAR : ");
    Uart::PutHex(far);
    Uart::Puts("\n");

}

void PutRawSynchronousExceptionInfo(uint32_t code, uint8_t ec, uint32_t iss, uint32_t iss2, uint64_t elr, uint64_t spsr, uint64_t far)
{
    Uart::Puts("Synchronous exception\n");
    Uart::Puts("Code: ");
    Uart::PutDec(code);
    Uart::Puts("Exception class: ");
    Uart::PutBin(ec);
    Uart::Puts("\n");
    Uart::Puts("ISS: ");
    Uart::PutBin(iss);
    Uart::Puts("\n");
    Uart::Puts("ISS2: ");
    Uart::PutBin(iss2);
    Uart::Puts("\n");
    Uart::Puts("\nELR : ");
    Uart::PutHex(elr);
    Uart::Puts("\nSPSR: ");
    Uart::PutHex(spsr);
    Uart::Puts("\nFAR : ");
    Uart::PutHex(far);
    Uart::Puts("\n");
}

extern "C" void MainExceptionHandler(uint32_t code)
{
    uint64_t core = 0;
    asm volatile ("mrs %0, mpidr_el1" : "=r"(core));
    Uart::Puts("Main Exception Handler on core");
    Uart::PutDec(core & 3);
    Uart::Puts("\n");
    uint64_t el = 0;
    uint64_t esr = 0;
    uint64_t elr = 0;
    uint64_t spsr = 0;
    uint64_t far = 0;
    asm volatile ("mrs %0, CurrentEL" : "=r"(el));
    if (((el >> 2) & 0b11) == 1)
    {
        Uart::Puts("Handling in EL1\n");
        asm volatile ("mrs %0, esr_el1" : "=r"(esr));
        asm volatile ("mrs %0, elr_el1" : "=r"(elr));
        asm volatile ("mrs %0, spsr_el1" : "=r"(spsr));
        asm volatile ("mrs %0, far_el1" : "=r"(far));
    }
    else if (((el >> 2) & 0b11) == 2)
    {
        Uart::Puts("Handling in EL2\n");
        asm volatile ("mrs %0, esr_el2" : "=r"(esr));
        asm volatile ("mrs %0, elr_el2" : "=r"(elr));
        asm volatile ("mrs %0, spsr_el2" : "=r"(spsr));
        asm volatile ("mrs %0, far_el2" : "=r"(far));
    }
    else
    {
        Uart::Puts("Unknown exception level ");
        Uart::PutDec((el >> 2) & 3);
        Processor::Halt();
    }

    Uart::Puts("Faulting instruction: ");
    Uart::PutHex(elr);
    Uart::Puts("\n");

    switch (code & 3)
    {
        case 0: // Synchronous exception
        {
            uint8_t  const ec   = (esr >> 26) & 0b11'1111;
            uint32_t const iss  = (esr >>  0) & 0x01FF'FFFF;
            uint32_t const iss2 = (esr >> 32) & 0x00FF'FFFF;
            switch ((esr >> 26) & 0b11'1111)
            {
                case 0b00'0000: Uart::Puts("Unknown exception class\n"); break;
                case 0b01'0101: return SvcException((uint16_t)iss, elr);
                case 0b10'0100: [[fallthrough]];
                case 0b10'0101: return DataAbortException(ec, iss, iss2, far);
                default: Uart::Puts("Other synchronous exception class\n"); break;
            }
            PutRawSynchronousExceptionInfo(code, ec, iss, iss2, elr, spsr, far); 
            Uart::Puts("\n");
            break;
        }
        case 1: // IRQ
            Uart::Puts("IRQ exception\n");
            PutRawExceptionInfo(code, esr, elr, spsr, far);
            Uart::Puts("\n");
            break;
        case 2: // FIQ
            Uart::Puts("FIQ exception\n");
            PutRawExceptionInfo(code, esr, elr, spsr, far);
            Uart::Puts("\n");
            break;
        case 3: // SError
            Uart::Puts("SError exception\n");
            PutRawExceptionInfo(code, esr, elr, spsr, far);
            Uart::Puts("\n");
            break;
    }

    Processor::Halt();
}

extern "C" void ExceptionVectors(void);

void Init()
{
    asm volatile ("msr vbar_el1, %0" :: "r"((uint64_t)&ExceptionVectors));
}

void InitEL2()
{
    asm volatile ("msr vbar_el2, %0" :: "r"((uint64_t)&ExceptionVectors));
}

}
// namespace Exception
