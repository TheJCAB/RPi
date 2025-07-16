#include "Scheduler.h"
#include "Containers.h"

#include "Cpu.h"

#include <deque>

namespace Scheduler
{

constexpr size_t MaxCores = 4;

struct PendingSpark
{
    Spark     Spark;
    TaskInfo* Task;
};

struct CoreInfo
{
    size_t CoreId;

    static constexpr uint32_t MaxThreads = 64;
    static constexpr uint32_t MaxSparks  = 64;
    Containers::CircularFifo<ThreadInfo*, MaxThreads> PendingThreadsFifo;
    Containers::CircularFifo<PendingSpark, MaxSparks> PendingSparksFifo;
};

CoreInfo CoreSchedulingInfos[MaxCores];

inline SparkInfo* SwapCurrentSparkInfo(SparkInfo* newInfo)
{
    return std::exchange(GetCurrentThreadInfo().Spark, newInfo);
}

inline ThreadInfo& SwapCurrentThreadInfo(ThreadInfo* newInfo)
{
    ThreadInfo* oldInfo;
    asm volatile ("mov %0, x18" : "=r"(oldInfo));
    asm volatile ("mov x18, %0" :: "r"(newInfo));
    return *oldInfo;
}

void Init()
{
    auto const coreId = Cpu::mpidr_el1->CoreId;
    auto& coreInfo = CoreSchedulingInfos[coreId];
    coreInfo.CoreId = coreId;
    auto const threadInfo = new ThreadInfo
    {
        .StackBuffer{ reinterpret_cast<std::byte*>(0x8'0000 - coreId * 0x1'0000), 0x1'0000 },
        .Core   = &coreInfo,
    };
    SwapCurrentThreadInfo(threadInfo);
}

void AddSpark(Spark const& spark)
{
    auto const coreId = Cpu::mpidr_el1->CoreId;
    auto& info = CoreSchedulingInfos[coreId];
    info.PendingSparksFifo.Push({ spark, nullptr });
}

bool ScheduleOneSpark()
{
    auto& threadInfo = GetCurrentThreadInfo();
    auto const coreInfo = threadInfo.Core;
    if (coreInfo->PendingSparksFifo.IsEmpty())
    {
        return false; // No sparks to schedule
    }
    auto const pendingSpark = coreInfo->PendingSparksFifo.Pop();
    auto const oldTask = std::exchange(threadInfo.Task, pendingSpark.Task);
    SparkInfo sparkInfo
    {
    };
    auto const oldSparkInfo = std::exchange(threadInfo.Spark, &sparkInfo);
    pendingSpark.Spark.Func(pendingSpark.Spark.Context);
    threadInfo.Spark = oldSparkInfo;
    threadInfo.Task  = oldTask;

    return true;
}

[[noreturn]] void Schedule(CoreInfo& info)
{
    while (true) {}
}

[[noreturn]] void SetSparkContinuation(Spark const& spark)
{
    auto& threadInfo = GetCurrentThreadInfo();
    auto const coreInfo = threadInfo.Core;
    coreInfo->PendingSparksFifo.Push({ spark, threadInfo.Task });

    Schedule(*coreInfo);
}

void Yield()
{
    while (ScheduleOneSpark())
    {
        // Keep scheduling sparks until there are no more pending sparks
    }
    auto& threadInfo = GetCurrentThreadInfo();
    auto const coreInfo = threadInfo.Core;
    if (coreInfo->PendingThreadsFifo.IsEmpty())
    {
        // No threads to schedule, just yield
        return;
    }
}

void DelayInMilliseconds(uint32_t ms)
{
    if (ms == 0)
    {
        return; // No delay needed
    }

    auto const targetTime = Cpu::GetPerformanceCounter() + Cpu::GetPerformanceTicksForMs(ms);
    while (Cpu::GetPerformanceCounter() < targetTime)
    {
        ScheduleOneSpark(); // Allow other sparks to run while waiting
        // Busy-wait until the specified time has passed
    }
}

ThreadInfo& CreateThread(ThreadFunction* func, uintptr_t context)
{
    uint32_t const stackSize = 0x1'0000u; // Allocate 64 KiB stack aligned to 16 bytes
    auto     const stackLow  = new(std::align_val_t{ 16 }) std::byte[0x1'0000];
    auto     const stackHigh = stackLow + stackSize;

    auto const threadContext = reinterpret_cast<ThreadContext*>(stackHigh) - 1;

    Uart::Puts("Creating thread with stack at ");
    Uart::PutHex(reinterpret_cast<uintptr_t>(stackLow));
    Uart::Puts(" and context at ");
    Uart::PutHex(reinterpret_cast<uintptr_t>(threadContext));
    Uart::Puts(" core at: ");
    Uart::PutHex(reinterpret_cast<uintptr_t>(GetCurrentThreadInfo().Core));
    Uart::Puts("\n");

    auto const info = new ThreadInfo
    {
        .Context = threadContext,
        .StackBuffer{ stackLow, stackSize },
        .Core   = GetCurrentThreadInfo().Core,
        .Spark  = nullptr,
        .Task   = nullptr,
    };

    threadContext->X[ 0] = context; // Set the first argument in X0
    threadContext->X[18] = reinterpret_cast<uintptr_t>(info); // Set the thread info pointer in X18
    threadContext->Sp    = reinterpret_cast<uintptr_t>(stackHigh);
    threadContext->Pc    = reinterpret_cast<uintptr_t>(func);
    threadContext->Spsr = { .SP = 1, .EL = 1, .D = 1 };

    auto const coreInfo = info->Core;
    {
        Cpu::WithInterruptsDisabled cs{};
        coreInfo->PendingThreadsFifo.Push(info);
    }

    return *info;
}

}
// namespace Scheduler

static_assert(offsetof(ThreadContext, X   ) == ThreadContext_X   );
static_assert(offsetof(ThreadContext, Sp  ) == ThreadContext_Sp  );
static_assert(offsetof(ThreadContext, Pc  ) == ThreadContext_Pc  );
static_assert(offsetof(ThreadContext, Spsr) == ThreadContext_Spsr);
static_assert(offsetof(ThreadContext, V   ) == ThreadContext_V   );
static_assert(sizeof(ThreadContext::V) == ThreadContext_VSize);
static_assert(sizeof(ThreadContext   ) == ThreadContext_Size);
