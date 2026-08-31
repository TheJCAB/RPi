#include "Debugger.h"

#include "Scheduler.h"

#include <array>

namespace Debugger
{

void RawPrintCallstack(Uart::LockedStream& stream, ThreadContext* context)
{
    stream.Puts("Call Stack:\n");

    uint64_t fp = context->Fp;

    stream.Puts("0: FP ");
    stream.PutHex(fp);
    stream.Puts("  PC ");
    stream.PutHex(context->Lr);
    stream.Puts("\n");

    stream.Puts("1: FP ");
    stream.PutHex(fp);
    stream.Puts("  PC ");
    stream.PutHex(context->Lr);
    stream.Puts(" (maybe, if leaf)\n");

    for (uint32_t level = 2; fp != 0 && level < 64; ++level)
    {
        auto const frame = reinterpret_cast<uint64_t const*>(fp);

        uint64_t const parentFp = frame[0];
        uint64_t const returnAddress = frame[1];

        stream.PutDec(level);
        stream.Puts(": FP ");
        stream.PutHex(parentFp);
        stream.Puts("  PC ");
        stream.PutHex(returnAddress);
        stream.Puts("\n");

        fp = parentFp;
    }
}

void PrintThreadContext(Uart::LockedStream& stream, ThreadContext* context)
{
    stream.Puts("Thread Context:\n");
    for (uint32_t i = 0; i < 29; ++i)
    {
        stream.PutHex(context->X[i]);
        stream.Puts(" X");
        stream.PutDec(i);
        stream.Puts("\n");
    }
    stream.PutHex(context->Fp);
    stream.Puts(" FP\n");
    stream.PutHex(context->Lr);
    stream.Puts(" LR\n");
    stream.PutHex(context->Sp);
    stream.Puts(" SP\n");
    stream.PutHex(context->Pc);
    stream.Puts(" PC\n");


}

void RawPrintThreadContext(ThreadContext* context)
{
    Uart::Raw::Puts("Thread Context:\n");
    for (uint32_t i = 0; i < 29; ++i)
    {
        Uart::Raw::PutHex(context->X[i]);
        Uart::Raw::Puts(" X");
        Uart::Raw::PutDec(i);
        Uart::Raw::Puts("\n");
    }
    Uart::Raw::PutHex(context->Fp);
    Uart::Raw::Puts(" FP\n");
    Uart::Raw::PutHex(context->Lr);
    Uart::Raw::Puts(" LR\n");
    Uart::Raw::PutHex(context->Sp);
    Uart::Raw::Puts(" SP\n");
    Uart::Raw::PutHex(context->Pc);
    Uart::Raw::Puts(" PC\n");
}


Scheduler::ThreadInfo* DebuggerThread = nullptr;

[[noreturn]] void Run(uintptr_t debuggedThreadContext)
{
    auto const context = reinterpret_cast<ThreadContext*>(debuggedThreadContext);
    Uart::LockedStream stream;
    stream.Puts("Debugger thread started\n");
    stream.Puts("Debugging thread context:\n");
    PrintThreadContext(stream, context);
    RawPrintCallstack(stream, context);

    // Here you can add code to handle debugging tasks, like printing thread contexts.
    while (true)
    {
        // For example, print the current thread context every second.
        //PrintThreadContext(stream, &Scheduler::GetCurrentThreadInfo().Context);
        Cpu::DelayInMilliseconds(1000);
    }
}

//void Debug(ThreadContext* debuggedThread)
//{
//    if (debuggedThread == nullptr)
//    {
//        Uart::Puts("No thread to debug.\n");
//        return;
//    }
//
//    // Create a new thread for debugging
//    DebuggerThread->Context.X[0] = reinterpret_cast<uintptr_t>(debuggedThread);
//    
//    Uart::Puts("Debugger thread created.\n");
//}

Scheduler::ThreadInfo* Init()
{
    auto& debuggerThread = Scheduler::CreateThread(&Run, 0);
    DebuggerThread = &debuggerThread;
    if (DebuggerThread == nullptr)
    {
        Uart::Puts("Failed to create debugger thread.\n");
    }
    else
    {
        Uart::Puts("Debugger thread initialized.\n");
    }

    return &debuggerThread;
}

void RawPrintCallstack()
{
    Uart::LockedStream stream;
    stream.Puts("Call Stack:\n");
    uint64_t fp;
    asm volatile (
        "mov %0, x29\n" // Frame pointer
        : "=r"(fp)
    );
    for (;;)
    {
        auto frame = *(std::array<uint64_t, 2>*)fp;
        fp = frame[0];
        if (fp != 0)
        {
            return;
        }
        stream.Puts("");
        stream.PutHex(frame[0]);
        stream.Puts(" ");
        stream.PutHex(frame[1]);
        stream.Puts("\n");
    }
}

}
// namespace Debugger
