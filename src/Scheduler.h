#pragma once

#include <stdint.h>
#include <stddef.h>

#include <span>

namespace Scheduler
{

struct CoreInfo;

struct ThreadInfo
{
    std::span<std::byte> StackBuffer;
};

struct TaskInfo
{
    // TLS?
};

struct RunningSparkInfo
{
    CoreInfo&   Core;
    ThreadInfo& Thread;
    TaskInfo*   Task;
};

using SparkFunction = void(uintptr_t context);



void Init();
void AddSpark(SparkFunction*, uintptr_t context);
[[noreturn]] void SetSparkContinuation(SparkFunction* func, uintptr_t context);

inline RunningSparkInfo& GetCurrentRunningSparkInfo()
{
    RunningSparkInfo* info;
    asm volatile ("mov %0, x18" : "=r"(info));
    return *info;
}

void DelayInMilliseconds(uint32_t ms);

}
// namespace Scheduler
