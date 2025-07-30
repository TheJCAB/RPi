#include "Debugger.h"

#include "Scheduler.h"

namespace Debugger
{

void PrintThreadContext(Uart::LockedStream& stream, ThreadContext* context)
{
    stream.Puts("Thread Context:\n");
    for (uint32_t i = 0; i < 31; ++i)
    {
        stream.PutHex(context->X[i]);
        stream.Puts(" X");
        stream.PutDec(i);
        stream.Puts("\n");
    }
    stream.PutHex(context->Sp);
    stream.Puts(" SP\n");
    stream.PutHex(context->Pc);
    stream.Puts(" PC\n");

    stream.PutHex(*reinterpret_cast<uintptr_t*>(context->X[20]));
    stream.Puts(" [X20]\n");
}

void RawPrintThreadContext(ThreadContext* context)
{
    Uart::Raw::Puts("Thread Context:\n");
    for (uint32_t i = 0; i < 31; ++i)
    {
        Uart::Raw::PutHex(context->X[i]);
        Uart::Raw::Puts(" X");
        Uart::Raw::PutDec(i);
        Uart::Raw::Puts("\n");
    }
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
