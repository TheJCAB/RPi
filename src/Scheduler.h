#pragma once

#include "ThreadContext.h"

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
    ThreadContext*       Context;
    std::span<std::byte> StackBuffer;
    CoreInfo*            Core;
    SparkInfo*           Spark;
    TaskInfo*            Task;
};

using SparkFunction = void(uintptr_t context);

struct Spark
{
    SparkFunction* Func;
    uintptr_t      Context;
};


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
