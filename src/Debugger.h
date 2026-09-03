#pragma once

#include "Uart.h"
#include "ThreadContext.h"
#include "Scheduler.h"

namespace Debugger
{

extern Scheduler::ThreadInfo* DebuggerThread;

void PrintThreadContext(Uart::LockedStream&, ThreadContext*);
void PrintCallstack    (Uart::LockedStream&, ThreadContext*);

void RawPrintThreadContext(ThreadContext* context);

void PrintCurrentCallstack();

//void Debug(ThreadContext* debuggedThread);

Scheduler::ThreadInfo* Init();

}
// namespace Debugger
