#include <stdint.h>
#include <stddef.h>

#include "Cpu.h"
#include "Uart.h"
#include "Processor.h"
#include "ThreadContext.h"

namespace Exception
{

void DataAbortException(Uart::LockedStream& stream, uint8_t ec, uint32_t iss, uint32_t iss2, uint64_t far)
{
    stream.Puts("Data ");
    bool isWrite = ((iss >> 6) & 1) == 1;;
    stream.Puts(isWrite ? "(write)" : "(read)");
    stream.Puts(" Abort exception, ");
    stream.Puts(ec & 1 ? "same" : "lower");
    stream.Puts(" level\n");
    if (iss & (1 << 10))
    {
        stream.Puts("Accessing address: ");
        stream.PutHex(far);
        stream.Puts("\n");
    }
    else
    {
        stream.Puts("No address given, FAR = ");
        stream.PutHex(far);
        stream.Puts("\n");
    }
    uint8_t const dfsc = iss & 0b11'1111;
    switch (dfsc)
    {
        case 0b00'0100:
        case 0b00'0101:
        case 0b00'0110:
        case 0b00'0111:
            stream.Puts("Translation fault, level ");
            stream.PutDec((uint8_t)(dfsc & 3));
            stream.Puts(".\n");
            Processor::Halt();
        case 0b10'0001:
            stream.Puts("Alignment fault.\n");
            Processor::Halt();
        default:
            stream.Puts("Unknown data fault status code: ");
            stream.PutBin(dfsc);
            stream.Puts("\n");
            Processor::Halt();
    }
}

void SvcException(Uart::LockedStream& stream, uint16_t imm, uint64_t lr)
{
    stream.Puts("Supervisor Call (SVC) exception\n");
    stream.Puts("Imm: ");
    stream.PutDec(imm);
    stream.Puts("\n");
    stream.Puts("From address: ");
    stream.PutHex(lr);
    stream.Puts("\n");

    // Handle SVC here
}

void PutRawExceptionInfo(Uart::LockedStream& stream, uint32_t code, uint64_t esr, uint64_t elr, uint64_t spsr, uint64_t far)
{
    stream.Puts("Panic Exception Handler\n");
    stream.Puts("Code: ");
    stream.PutHex(code);
    stream.Puts("\nESR : ");
    stream.PutHex(esr);
    stream.Puts("\nELR : ");
    stream.PutHex(elr);
    stream.Puts("\nSPSR: ");
    stream.PutHex(spsr);
    stream.Puts("\nFAR : ");
    stream.PutHex(far);
    stream.Puts("\n");

}

void PutRawSynchronousExceptionInfo(Uart::LockedStream& stream, uint32_t code, uint8_t ec, uint32_t iss, uint32_t iss2, uint64_t elr, uint64_t spsr, uint64_t far)
{
    stream.Puts("Synchronous exception\n");
    stream.Puts("Code: ");
    stream.PutHex(code);
    stream.Puts("\nException class: ");
    stream.PutBin(ec);
    stream.Puts("\nISS: ");
    stream.PutBin(iss);
    stream.Puts("\nISS2: ");
    stream.PutBin(iss2);
    stream.Puts("\nELR : ");
    stream.PutHex(elr);
    stream.Puts("\nSPSR: ");
    stream.PutHex(spsr);
    stream.Puts("\nFAR : ");
    stream.PutHex(far);
    stream.Puts("\n");
}

extern "C" [[noreturn]] ThreadContext* MainExceptionHandler(ThreadContext* context, uint32_t code)
{
    Uart::LockedStream stream(true);

    uint64_t core = 0;
    asm volatile ("mrs %0, mpidr_el1" : "=r"(core));
    stream.Puts("Main Exception Handler on core");
    stream.PutDec(core & 3);
    stream.Puts("\n");
    uint64_t el = 0;
    uint64_t esr = 0;
    uint64_t elr = 0;
    uint64_t spsr = 0;
    uint64_t far = 0;
    asm volatile ("mrs %0, CurrentEL" : "=r"(el));
    if (((el >> 2) & 0b11) == 1)
    {
        stream.Puts("Handling in EL1\n");
        asm volatile ("mrs %0, esr_el1" : "=r"(esr));
        asm volatile ("mrs %0, elr_el1" : "=r"(elr));
        asm volatile ("mrs %0, spsr_el1" : "=r"(spsr));
        asm volatile ("mrs %0, far_el1" : "=r"(far));
    }
    else if (((el >> 2) & 0b11) == 2)
    {
        stream.Puts("Handling in EL2\n");
        asm volatile ("mrs %0, esr_el2" : "=r"(esr));
        asm volatile ("mrs %0, elr_el2" : "=r"(elr));
        asm volatile ("mrs %0, spsr_el2" : "=r"(spsr));
        asm volatile ("mrs %0, far_el2" : "=r"(far));
    }
    else
    {
        stream.Puts("Unknown exception level ");
        stream.PutDec((el >> 2) & 3);
        Processor::Halt();
    }

    stream.Puts("Faulting instruction: ");
    stream.PutHex(elr);
    stream.Puts("\n");

    switch (code & 3)
    {
        case 0: // Synchronous exception
        {
            uint8_t  const ec   = (esr >> 26) & 0b11'1111;
            uint32_t const iss  = (esr >>  0) & 0x01FF'FFFF;
            uint32_t const iss2 = (esr >> 32) & 0x00FF'FFFF;
            switch (ec)
            {
                case 0b00'0000: stream.Puts("Unknown exception class\n"); break;
                case 0b00'0111: stream.Puts("FP/SIMD disabled exception class\n"); break;
                case 0b01'0101: SvcException(stream, (uint16_t)iss, elr); break;
                case 0b10'0100: [[fallthrough]];
                case 0b10'0101: DataAbortException(stream, ec, iss, iss2, far); break;
                default: stream.Puts("Other synchronous exception class\n"); break;
            }
            PutRawSynchronousExceptionInfo(stream, code, ec, iss, iss2, elr, spsr, far); 
            stream.Puts("\n");
            break;
        }
        case 1: // IRQ
            stream.Puts("IRQ exception\n");
            PutRawExceptionInfo(stream, code, esr, elr, spsr, far);
            stream.Puts("\n");
            break;
        case 2: // FIQ
            stream.Puts("FIQ exception\n");
            PutRawExceptionInfo(stream, code, esr, elr, spsr, far);
            stream.Puts("\n");
            break;
        case 3: // SError
            stream.Puts("SError exception\n");
            PutRawExceptionInfo(stream, code, esr, elr, spsr, far);
            stream.Puts("\n");
            break;
    }

    Processor::Halt();
}

extern "C" void ExceptionVectors(void);

void Init()
{
    if (Cpu::CurrentEL->EL == 1)
    {
        // If we are in EL1, we need to set the exception vector base address register (VBAR_EL1)
        // to point to our exception vectors.
        asm volatile ("msr vbar_el1, %0" :: "r"((uint64_t)&ExceptionVectors));
    }
    else if (Cpu::CurrentEL->EL == 2)
    {
        // If we are in EL2, we need to set the exception vector base address register (VBAR_EL2)
        // to point to our exception vectors.
        asm volatile ("msr vbar_el2, %0" :: "r"((uint64_t)&ExceptionVectors));
    }
    else
    {
        Cpu::Panic("Exception vectors initialized in an unsupported exception level.\n");
    }

    // Enable SError, IRQ and FIQ.
    // Note: SError means synchronous exceptions, all caused by the executing code,
    // but not necessarily means errors. It includes system calls, memory faults, etc...
    Cpu::daifclr = 7;

    Cpu::InstructionSynchronizationBarrier();
}

// Called by the C++ exception handling mechanism to continue unwinding after a cleanup.
// On baremetal systems, stack unwinding is not supported, so just print a message and halt.
extern "C" void _Unwind_Resume(void* exception_object)
{
    Uart::Raw::Puts("_Unwind_Resume called - not implemented\n");
    Processor::Halt();
}

// This line defines a symbol for the C++ exception handling personality function.
// By setting '__gxx_personality_v0' to 0, it disables the default C++ exception handling
// mechanism for this translation unit. This is sometimes used in low-level or embedded
// environments where exception handling is not supported or desired.
extern "C" void* __gxx_personality_v0 = 0;

}
// namespace Exception
