#pragma once

#include "ThreadContext.h"
#include "Exception.h"

#include <stdint.h>
#include <stddef.h>

//#include <arm64_neon.h>

#include <span>

namespace Scheduler
{

struct CoreInfo;

struct SparkInfo
{
};

struct TaskInfo
{
    // TLS?
};

struct ThreadInfo
{
    ThreadContext*       ContextWhenSuspended;
    std::span<std::byte> StackBuffer;
    CoreInfo*            Core;
    SparkInfo*           Spark;
    TaskInfo*            Task;
};

// TODO: Scheduler Sparks should be meant to be run in "user mode".
// They should be non-time-critical "fire and forget" snippets to be run as background tasks.
// They should still take priority over threads, just like IRQs take priority over "regular" code.
// We probably should add a priority scheme for Sparks, so that some can be more important than others.
// Threads should be the lowest in the priority totem pole.
// Ideally, user-mode Sparks should be short-lived, and it should be possible for the scheduler
// to kill them or worse if they run too long, based on heuristic (too IRQ time should be deducted).
// Ideally, much "user mode" code will be structured using coroutines that schedule Sparks when suspended.

using SparkFunction = Exception::SparkFunction;
using Spark = Exception::Spark;

using SparkFunctionNoContext = Spark();

inline Spark MakeUserModeSpark(SparkFunction* func, uintptr_t context)
{
    return { .Context = context, .Func = reinterpret_cast<SparkFunction*>(reinterpret_cast<uintptr_t>(func) | 1) };
}

inline Spark MakeUserModeSpark(SparkFunctionNoContext* func)
{
    return { .Func = reinterpret_cast<SparkFunction*>(reinterpret_cast<uintptr_t>(func) | 1) };
}

extern "C"
Spark GetNextScheduledSpark();

void Init();
void AddSpark(Spark const&);
[[noreturn]] void SetSparkContinuation(Spark const&);

inline ThreadInfo& GetCurrentThreadInfo()
{
    ThreadInfo* info;
    asm volatile ("mov %0, x18" : "=r"(info));
    return *info;
}

inline SparkInfo* GetCurrentSparkInfo()
{
    return GetCurrentThreadInfo().Spark;
}

inline TaskInfo* GetCurrentTaskInfo()
{
    return GetCurrentThreadInfo().Task;
}

void YieldToSparks();

void DelayInMilliseconds(uint32_t ms);

using ThreadFunction = void(uintptr_t);

ThreadInfo& CreateThread(ThreadFunction*, uintptr_t context);

}
// namespace Scheduler
