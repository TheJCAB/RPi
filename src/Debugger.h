#pragma once

#include "Uart.h"
#include "ThreadContext.h"
#include "Scheduler.h"

namespace Debugger
{

extern Scheduler::ThreadInfo* DebuggerThread;

void PrintThreadContext(Uart::LockedStream& stream, ThreadContext* context);
void RawPrintThreadContext(ThreadContext* context);
void RawPrintCallstack();

//void Debug(ThreadContext* debuggedThread);

Scheduler::ThreadInfo* Init();

}
// namespace Debugger
