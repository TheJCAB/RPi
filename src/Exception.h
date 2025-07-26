#pragma once

#include <stdint.h>
#include <stddef.h>

struct ThreadContext;

namespace Exception
{

// The exception mechanism works like this, for synchronous and IRQ exceptions:
// 1. The exception handler is called with the current thread context.
// 2. The handler does whatever is required by the particular event that caused it to be called.
// 3. The handler may return a Spark, which is a function pointer and a context.
//    It can return an empty Spark (where the function is nullptr).
// 4. If the Spark is not empty, it is invoked to do whatever work is required.
//    The Spark function may return a new Spark, which will be invoked again, and so on.
//
// If/when no Spark is returned, the current thread context will be restored.
// Note that any of the handlers or sparks may modify the current thread context.
// Note also that the Spark function is called as part of the handler, with interrupts still disabled.
//
// Sparks should be straight short functions that do not block or wait for anything,
// so a number of them can be reasonably invoked as part of interrupt handling.

// TODO: Add support for "user-mode" sparks, which would be interruptible by a new IRQ
// and would allow for system calls.
// This will require copying and modifying the ELR register to "return" to the Spark function,
// and terminating the Spark by invoking a SVC so the original context may be restored.

struct Spark;

using SparkFunction = Spark(uintptr_t context);

struct Spark
{
    uintptr_t      Context;
    SparkFunction* Func;

    explicit operator bool() const { return Func != nullptr; }
};

using HandlerFunction = Spark(*)();

extern "C" Spark ThreadContextSwitchSpark(uintptr_t context);

inline Spark MakeSpark(ThreadContext* threadContext)
{
    return Spark {
        .Context = reinterpret_cast<uintptr_t>(threadContext),
        .Func    = &ThreadContextSwitchSpark,
    };
}

void Init();

}
// namespace Exception
