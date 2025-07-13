#include "Scheduler.h"

#include "Cpu.h"

#include <deque>

namespace Scheduler
{

constexpr size_t MaxCores = 4;

struct PendingSpark
{
    SparkFunction& Func;
    uintptr_t      Context;
    TaskInfo*      Task;
};

struct CoreInfo
{
    size_t CoreId;

    std::deque<ThreadInfo*>  Threads;
    std::deque<PendingSpark> PendingSparksQueue;
};

CoreInfo CoreSchedulingInfos[MaxCores];

inline RunningSparkInfo* SwapRunningSparkInfo(RunningSparkInfo* newInfo)
{
    RunningSparkInfo* oldInfo;
    asm volatile ("mov %0, x18" : "=r"(oldInfo));
    asm volatile ("mov x18, %0" :: "r"(newInfo));
    return oldInfo;
}

void Init()
{
    auto const coreId = Cpu::mpidr_el1->CoreId;
    auto& coreInfo = CoreSchedulingInfos[coreId];
    coreInfo.CoreId = coreId;
    auto const threadInfo = new ThreadInfo
    {
        .StackBuffer{ reinterpret_cast<std::byte*>(0x8'0000 - coreId * 0x1'0000), 0x1'0000 },
    };
    auto const sparkInfo = new RunningSparkInfo
    {
        .Core   = coreInfo,
        .Thread = *threadInfo,
    };
    SwapRunningSparkInfo(sparkInfo);
}

void AddSpark(SparkFunction* func, uintptr_t context)
{
    auto const coreId = Cpu::mpidr_el1->CoreId;
    auto& info = CoreSchedulingInfos[coreId];
    info.PendingSparksQueue.push_back({ *func, context, nullptr });
}

void ScheduleOneSpark()
{
    auto& callingSparkInfo = GetCurrentRunningSparkInfo();
    auto& coreInfo = callingSparkInfo.Core;
    if (coreInfo.PendingSparksQueue.empty())
    {
        return; // No sparks to schedule
    }
    auto const pendingSpark = coreInfo.PendingSparksQueue.front();
    coreInfo.PendingSparksQueue.pop_front();
    RunningSparkInfo oneSparkInfo
    {
        .Core   = coreInfo,
        .Thread = callingSparkInfo.Thread,
        .Task   = pendingSpark.Task,
    };
    SwapRunningSparkInfo(&oneSparkInfo);
    pendingSpark.Func(pendingSpark.Context);
    SwapRunningSparkInfo(&callingSparkInfo);
}

[[noreturn]] void Schedule(CoreInfo& info)
{
    while (true) {}
}

[[noreturn]] void SetSparkContinuation(SparkFunction* func, uintptr_t context)
{
    auto const coreId = Cpu::mpidr_el1->CoreId;
    auto& info = CoreSchedulingInfos[coreId];
    info.PendingSparksQueue.push_back(PendingSpark{ *func, context, GetCurrentRunningSparkInfo().Task });

    Schedule(info);
}

void DelayInMilliseconds(uint32_t ms)
{
    if (ms == 0)
    {
        return; // No delay needed
    }

    auto const targetTime = Cpu::GetPerformanceCounter() + Cpu::GetPerformanceTicksForMs(ms);
    while (Cpu::GetPerformanceDifference(Cpu::GetPerformanceCounter(), targetTime) < 0)
    {
        ScheduleOneSpark(); // Allow other sparks to run while waiting
        // Busy-wait until the specified time has passed
    }
}

}
// namespace Scheduler
