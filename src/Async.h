// This coroutine-based implementation is based on the coroutine.h file of the WIL library:
//
// https://github.com/microsoft/wil/blob/master/include/wil/coroutine.h
//
// This code simplifies from the original by eliminating all the COM and Windows dependencies,
// and support for pre-C++20 implementations.
//
// It also contains two enhancements to the code from that commit:
//  - task<>::empty() (to check whether there's a coroutine in the task),
//  - and the client_await_*() awaiter functions publicly exposed out of task<>,
//    which allow the implementation of custom task [a]waiter functions,
//    both synchronous and asynchronous.

   /*
    * A Async::task<T> is a coroutine with the following characteristics:
    *
    * - T must be a copyable object, movable object, reference, or void.
    * - The coroutine may be awaited at most once. The second await will crash.
    * - The coroutine may be abandoned (allowed to destruct without co_await),
    *   in which case unobserved exceptions are fatal.
    * - The awaiting coroutine may be destroyed while suspended.
    *   (Don't worry if you don't know what this means.)
    * - By default, Async::task resumes on an arbitrary thread.
    *
    * The Async::task is intended to supplement concurrency/multithreading libraries,
    * not to replace them. It provides coroutine implementations for scenarios which
    * such libraries may not support.
    *
    * The implementation is built and optimized on the assumption that the coroutine is
    * awaited only once, and that the coroutine is consumed after completion.
    * To ensure proper usage, the task object is move-only, and
    * co_await takes ownership of the task.
    *
    * Comparison with PPL and C++/WinRT:
    *
    * |                                                     | PPL       | C++/WinRT | wil::*task    |
    * |-----------------------------------------------------|-----------|-----------|---------------|
    * | T can be non-constructible                          | No        | Yes       | Yes           |
    * | T can be void                                       | Yes       | Yes       | Yes           |
    * | T can be reference                                  | No        | No        | Yes           |
    * | T can be move-only                                  | No        | No        | Yes           |
    * | Coroutine can be cancelled                          | Yes       | Yes       | No            |
    * | Awaiting coroutine may be destroyed while suspended | No        | No        | Yes           |
    * | Coroutine can throw arbitrary exceptions            | Yes       | No        | Yes           |
    * | Can co_await more than once                         | Yes       | No        | No            |
    * | Can change coroutine's resumption model             | No        | No        | Yes           |
    * | Can wait synchronously                              | Yes       | Yes       | Yes [2]       |
    * | Implementation is small and efficient               | No        | Yes       | Yes           |
    * | Can abandon coroutine (fail to co_await)            | Yes       | Yes       | Yes           |
    * | Exception in abandoned coroutine                    | Crash     | Ignored   | Crash         |
    * | Coroutine starts automatically                      | Yes       | Yes       | Yes           |
    * | Coroutine starts synchronously                      | No        | Yes       | Yes           |
    * ... And the following which are WinRT/COM related:
    * | T can be WinRT object                               | Yes       | Yes       | Yes           |
    * | T can be non-WinRT object                           | Yes       | No        | Yes           |
    * | co_await resumes in same COM context                | Sometimes | Yes       | You choose [1]|
    * | Can force co_await to resume in same context        | Yes       | N/A       | Yes [1]       |
    * | Can force co_await to resume in any context         | Yes       | No        | Yes           |
    * | Can be consumed by non-C++ languages                | No        | Yes       | No            |
    * | Integrates with C++/WinRT coroutine callouts        | No        | Yes       | No            |
    * 
    * [1] Resumption in the same COM apartment requires that you include COM headers.
    * [2] Synchronous waiting requires that you include <synchapi.h> (usually via <windows.h>).
    *
    * You can store the task in a variable, but since it is a move-only
    * object, you will have to use std::move in order to transfer ownership out of
    * an lvalue.
    *
    */

#pragma once

#include "Cpu.h"
#include "Scheduler.h"
#include "Timer.h"

#include <atomic>
#include <coroutine>
#include <exception>
#include <memory>
#include <utility>

//#include <assert.h>
#define assert(c) do { if (!(c)) { Cpu::Panic("Assertion failed: " #c); } } while (0)

namespace Async
{
    // There are three general categories of T that you can
    // use with a task. We give them these names:
    //
    // T = void ("void category")
    // T = some kind of reference ("reference category")
    // T = non-void non-reference ("object category")
    //
    // Take care that the implementation supports all three categories.
    //
    // There is a sub-category of object category for move-only types.
    // We designed our task to be co_awaitable only once, so that
    // it can contain a move-only type. Any transfer of T as an
    // object category must be done as an rvalue reference.
    template<typename T = void>
    struct task;

    template<typename T>
    struct task_promise;

    // Unions may not contain references, C++/CX types, or void.
    // To work around that, we put everything inside a result_wrapper
    // struct, and put the struct in the union. For void,
    // we create a special empty structure.
    //
    // get_value returns rvalue reference to T for object
    // category, or just T itself for void and reference
    // category.
    //
    // We take advantage of the reference collapsing rules
    // so that T&& = T if T is reference category.

    template<typename T>
    struct result_wrapper
    {
        T value;
        T get_value() { return std::forward<T>(value); }
    };

    template<>
    struct result_wrapper<void>
    {
        void get_value() { }
    };


    // The result_holder is basically a
    // std::variant<std::monotype, T, std::exception_ptr>
    // but with these extra quirks:
    // * The only valid transition is monotype -> something-else.
    //   Consequently, it does not have valueless_by_exception.

    template<typename T>
    struct result_holder
    {
        // The content of the result_holder
        // depends on the result_status:
        //
        // empty: No active member.
        // value: Active member is wrap.
        // error: Active member is error.
        enum class result_status { empty, value, error };

        result_status status{ result_status::empty };
        union variant
        {
            variant() {}
            ~variant() {}
            result_wrapper<T> wrap;
            std::exception_ptr error;
        } result;

        // set_value will be called with
        //
        // * no parameters (void category)
        // * The reference type T (reference category)
        // * Some kind of reference to T (object category)
        //
        // Set the status after constructing the object.
        // That way, if object construction throws an exception,
        // the holder remains empty.
        template<typename...Args>
        void set_value(Args&&... args)
        {
            assert(status == result_status::empty);
            new (std::addressof(result.wrap)) result_wrapper<T>{ std::forward<Args>(args)... };
            status = result_status::value;
        }

        void unhandled_exception() noexcept
        {
            assert(status == result_status::empty);
            new (std::addressof(result.error)) std::exception_ptr(std::current_exception());
            if (!result.error)
            {
                // A null exception would result in bad_exception when rethrown.
                // Best to fail right here so we can diagnose.
                Cpu::Panic("unhandled_exception: Null exception");
            }
            status = result_status::error;
        }

        T get_value()
        {
            if (status == result_status::value)
            {
                return result.wrap.get_value();
            }
            //assert(status == result_status::error);
            if (status != result_status::error)
            {
                // The status is not value or error, but someone called get_value anyway.
                // Best to fail right here so we can diagnose.
                Cpu::Panic("get_value: Unexpected status %u", static_cast<unsigned>(status));
            }
            if (!result.error)
            {
                // A null exception here cannot happen, it would have been spotted in unhandled_exception() above.
                // Best to fail right here so we can diagnose.
                Cpu::Panic("get_value: Null exception");
            }
            std::rethrow_exception(std::exchange(result.error, {}));
        }

        result_holder() = default;
        result_holder(result_holder const&) = delete;
        void operator=(result_holder const&) = delete;

        ~result_holder()
        {
            switch (status)
            {
            case result_status::value:
                result.wrap.~result_wrapper();
                break;
            case result_status::error:
                // Rethrow unobserved exception. Delete this line to
                // discard unobserved exceptions.
                if (result.error) std::rethrow_exception(result.error);
                result.error.~exception_ptr();
                break;
            case result_status::empty:
                break;
            }
        }
    };

    // Most of the work is done in the promise_base,
    // It is a CRTP-like base class for task_promise<void> and
    // task_promise<non-void> because the language forbids
    // a single promise from containing both return_value and
    // return_void methods (even if one of them is deleted by SFINAE).
    template<typename T>
    struct promise_base
    {
        // The coroutine state remains alive as long as the coroutine is
        // still running (hasn't reached final_suspend) or the associated
        // task has not yet abandoned the coroutine (either finished awaiting
        // or destructed without awaiting).
        //
        // This saves an allocation, but does mean that the local
        // frame of the coroutine will remain allocated (with the
        // coroutine's imbound parameters still live) until all
        // references are destroyed. To force the promise_base to be
        // destroyed after co_await, we make the promise_base a
        // move-only object and require co_await to be given an rvalue reference.
        
        // Special values for m_waiting.
        static constexpr uintptr_t Running    = UINTPTR_MAX;
        static constexpr uintptr_t Completing = UINTPTR_MAX - 1;
        static constexpr uintptr_t Completed  = UINTPTR_MAX - 2;
        static constexpr uintptr_t Abandoned  = UINTPTR_MAX - 3;

        // The awaiting coroutine is resumed by calling the
        // m_resumer with the m_waiting. If the resumer is null,
        // then the m_waiting is assumed to be the address of a
        // coroutine_handle<>, which is resumed synchronously.
        // Externalizing the resumer allows unused awaiters to be
        // removed by the linker and removes a hard dependency on COM.
        // Using nullptr to represent the default resumer avoids a
        // CFG check.

        using NotifierType = void(*)(uintptr_t);
        using ResumerType  = void(*)(uintptr_t);

        NotifierType m_notifier = nullptr;
        ResumerType  m_resumer  = nullptr;
        std::atomic<uintptr_t> m_waiting{ Running };
        result_holder<T> m_holder;

        // Make it easier to access our CRTP derived class.
        using Promise = task_promise<T>;
        auto as_promise() noexcept
        {
            return static_cast<Promise*>(this);
        }

        // Make it easier to access the coroutine handle.
        auto as_handle() noexcept
        {
            return std::coroutine_handle<Promise>::from_promise(*as_promise());
        }

        auto get_return_object() noexcept
        {
            // let the compiler construct the task from the promise.
            return as_promise();
        }

        void destroy()
        {
            as_handle().destroy();
        }

        // The client lost interest in the coroutine, either because they are discarding
        // the result without awaiting (risky!), or because they have finished awaiting.
        // Discarding the result without awaiting is risky because any exception in the coroutine
        // will be unobserved and result in a crash. If you want to disallow it, then
        // raise an exception if waiting == running_ptr.
        void abandon()
        {
            auto waiting = Running;
            if (m_waiting.compare_exchange_strong(waiting, Abandoned, std::memory_order_relaxed))
            {
                // It was Running, so it is now truly Abandoned.
                return;
            }
            if (waiting == Completed)
            {
                // It had been completed already, so we destroy the coroutine here. 
                destroy();
                return;
            }
            if (waiting != Completing)
            {
                assert(waiting != Abandoned);
                // It was Running, but has a notifier or resumer, so we try to abandon again.
                if (m_waiting.compare_exchange_strong(waiting, Abandoned, std::memory_order_relaxed))
                {
                    // We managed to steal the resumer argument, so now the coroutine is properly Abandoned.
                    return;
                }
                if (waiting == Completed)
                {
                    // It has just been completed, so we destroy the coroutine here. 
                    destroy();
                    return;
                }
            }
            // At this point, the coroutine is Completing, so we may have to wait.
            assert(waiting == Completing);
            if (m_notifier != nullptr)
            {
                // There was a notifier registered, so we need to be careful and defensive here.
                while (waiting == Completing)
                {
                    // The coroutine is Completing so we need to busy-wait
                    // to protect the awaiter's opaque "m_waiting" object.
                    // If we were to return too early, the currently-ongoing demise of the task object
                    // could easily be seen as permission to free/close/destroy it before the coroutine
                    // has a chance to use it.
                    Cpu::Yield();
                    waiting = m_waiting.load(std::memory_order_relaxed);
                }
            }
        }

        std::suspend_never initial_suspend() noexcept
        {
            return {};
        }

        template<typename...Args>
        void set_value(Args&&... args)
        {
            m_holder.set_value(std::forward<Args>(args)...);
        }

        void unhandled_exception() noexcept
        {
            m_holder.unhandled_exception();
        }

        static void resume_waiting_coroutine(uintptr_t waiting)
        {
            std::coroutine_handle<>::from_address(reinterpret_cast<void*>(waiting)).resume();
        }

        auto final_suspend() noexcept
        {
            struct awaiter : std::suspend_always
            {
                // Note: references to self must be through this-> in order to compile with Clang.
                //       (Clang error: '... ::awaiter::self' is not a member of class 'const awaiter')
                promise_base& self;
                void await_suspend(std::coroutine_handle<>) const noexcept
                {
                    // On asserting "Completing" we need acquire so we'll be able to read
                    // from m_notifier and m_resumer if a thread is awaiting.
                    // We also need release so that the results are published and available
                    // in the case where an awaiter is calling client_await_suspend() right now,
                    // so that they (possibly on another thread) cannot suspend and read the results
                    // without busy-waiting for us to finish.
                    // We need to be careful to watch for the task object abandoning the promise
                    // at all steps, in which case we're responsible to destroy the promise object.
                    auto waiting = this->self.m_waiting.exchange(Completing, std::memory_order_acq_rel);
                    if (waiting == Abandoned)
                    {
                        // The coroutine had been abandoned before it finished its run,
                        // so all we have to do is destroy it here.
                        this->self.destroy();
                        return;
                    }
                    if (waiting == Running)
                    {
                        // The coroutine had no awaiter. Mark the task Completed.
                        waiting = this->self.m_waiting.exchange(Completed, std::memory_order_release);
                        if (waiting == Abandoned)
                        {
                            // The coroutine was been Abandoned just after we asserted "Completing",
                            // so all we have to do is destroy it here.
                            this->self.destroy();
                            return;
                        }
                        // The only remaining possibilities here are "Completing", which means
                        // nobody interfered, or a concurrent attempt to register an awaiter.
                        // Either way we already marked the task complete.
                        // If there was an attempt to register an awaiter, they have seen the "Completing" so they
                        // are guaranteed to abort. They'll just fail to change the state back to "Completing".
                        return;
                    }

                    // An awaiter had successfully called client_await_notify() or client_await_suspend(),
                    // so we need to call the callback.
                    assert(waiting != Completing);
                    assert(waiting != Completed);
                    // Once we assert "Completed", the coroutine frame may be destroyed at any moment
                    // by the task owner (see 'abandon()'), so it'll no longer be safe to use this/self.
                    // 'waiting' also may or may not be safe to use after we assert "Completed",
                    // depending on the caller's intent.
                    // The caller must register itself as a "notifier" if it intends the task owner
                    // to release resources referenced by 'waiting'.
                    auto notifier = this->self.m_notifier;
                    if (notifier != nullptr)
                    {
                        notifier(waiting);
                    }

                    // Attempt to mark the coroutine as completed.
                    // Note that we need to read anything we'll need ahead of time, because
                    // as soon as we assert "Completed" the coroutine frame and anything in it is forfeit.
                    auto resumer = this->self.m_resumer;
                    auto completing = Completing;
                    if (!this->self.m_waiting.compare_exchange_strong(completing, Completed, std::memory_order_release))
                    {
                        // The coroutine was abandoned as we were completing it, so we need to destroy it now.
                        assert(completing == Abandoned);
                        this->self.destroy();
                        return;
                    }

                    if (resumer == &promise_base::resume_waiting_coroutine)
                    {
                        // Common case: the coroutine is co_awaited. Avoid CFG cost by calling the function directly.
                        promise_base::resume_waiting_coroutine(waiting);
                        return;
                    }

                    if (resumer != nullptr)
                    {
                        resumer(waiting);
                    }
                }
            };
            return awaiter{ {}, *this };
        }

        // The remaining methods are used by the awaiters.
        bool client_await_ready() const
        {
            // Need acquire in case the coroutine has already completed,
            // so we can read the results. This matches the release in
            // the final_suspend's await_suspend.
            auto waiting = m_waiting.load(std::memory_order_acquire);
            assert(waiting != Abandoned);
            return waiting == Completing || waiting == Completed;
        }

        // Call the 'notify' callback with the 'waiting' parameter to notify when the value is ready to retrieve.
        // Do this before the coroutine is completely done running,
        // so that the frame is guaranteed to remain alive for the duration.
        bool client_await_notify(uintptr_t waiting, NotifierType notifier)
        {
            // We only support ever setting either a single 'notify' or a single 'resume' callback.
            // Once set, it cannot be overriden.
            assert(m_resumer == nullptr && m_notifier == nullptr);
            m_notifier = notifier;
            return client_await_suspend(waiting);
        }

        // Call the 'resume' callback with the 'waiting' parameter to notify when the value is ready to retrieve.
        // Do this after the coroutine is completely done running and the frame is free to be destroyed.
        bool client_await_suspend(uintptr_t waiting, ResumerType resumer)
        {
            // We only support ever setting either a single 'notify' or a single 'resume' callback.
            // Once set, it cannot be overriden.
            assert(m_resumer == nullptr && m_notifier == nullptr);
            m_resumer = resumer;
            return client_await_suspend(waiting);
        }

        bool client_await_suspend(std::coroutine_handle<> waiting)
        {
            return client_await_suspend(reinterpret_cast<uintptr_t>(waiting.address()), &resume_waiting_coroutine);
        }

        bool client_await_suspend(uintptr_t waiting)
        {
            // "waiting" needs to be a pointer to an object. We reserve the first 16
            // pseudo-pointers as sentinels.
            assert(reinterpret_cast<uintptr_t>(waiting) > 16);

            // Acquire to ensure that we can read the results of the return value, if the coroutine is completed.
            // Release to ensure that our resumption state is published, if the coroutine is not completed.
            auto previous = m_waiting.exchange(waiting, std::memory_order_acq_rel);

            // Suspend if the coroutine is still running.
            // Otherwise, the coroutine is completed: Nobody will resume us, so we will have to resume ourselves.
            assert((previous == Running) || (previous == Completing) || (previous == Completed));
            if (previous == Running)
            {
                // The coroutine was still running so the resumer will be invoked when the coroutine completes.
                return true;
            }
            else if (previous == Completing)
            {
                // The coroutine is already in the process of completing, and we were just too late.
                // The result is ready now, but the resumer that we just published won't be invoked.
                // That's actually not a problem: we'll just return false so the result can be retrieved immediately.
                // But first we attempt to restore the state to "completing", so that client_await_ready() can
                // still be used for polling/querying.
                m_waiting.compare_exchange_strong(waiting, Completing, std::memory_order_relaxed);
                // Either the waiting state is now "completing" and final_suspend() will eventually turn it
                // to "completed" as normal, or final_suspend() already had turned it to "completed".
                // Either way, the result is ready and the coroutine completed "enough" to retrieve it.
                return false;
            }
            else
            {
                // final_suspend() already had turned the waiting state to "completed", so they are done.
                // We restore the waiting state back to "completed", to allow client_await_ready() to be used
                // for polling/querying and destruction of the coroutine can proceed when the awaiter is done.
                m_waiting.store(Completed, std::memory_order_relaxed);
                return false;
            }
        }

        T client_await_resume()
        {
            return m_holder.get_value();
        }
    };

    template<typename T>
    struct task_promise : promise_base<T>
    {
        template<typename U>
        void return_value(U&& value)
        {
            this->set_value(std::forward<U>(value));
        }

        void return_value(T const& value)
            requires(!std::is_reference_v<T>)
        {
            this->set_value(value);
        }
    };

    template<>
    struct task_promise<void> : promise_base<void>
    {
        void return_void()
        {
            this->set_value();
        }
    };

    template<typename T>
    struct promise_deleter
    {
        void operator()(promise_base<T>* promise) const noexcept
        {
            promise->abandon();
        }
    };

    template<typename T>
    using promise_ptr = std::unique_ptr<promise_base<T>, promise_deleter<T>>;

    template<typename T>
    struct [[nodiscard]] task
    {
        using promise_type = task_promise<T>;

        using NotifierType = promise_type::NotifierType;
        using ResumerType  = promise_type::ResumerType ;

        // Compiler error message metaprogramming: Tell people that they
        // need to use std::move() if they try to co_await an lvalue.
        struct cannot_await_lvalue_use_std_move { void await_ready() {} };
        cannot_await_lvalue_use_std_move operator co_await() & = delete;

        [[nodiscard]] auto operator co_await() && noexcept
        {
            struct awaiter
            {
                promise_ptr<T> promise;

                bool await_ready  ()                               { return promise->client_await_ready(); }
                bool await_suspend(std::coroutine_handle<> handle) { return promise->client_await_suspend(handle); }
                T    await_resume ()                               { return promise->client_await_resume(); }
            };
            return awaiter{ std::move(promise) };
        }

        bool empty() const { return !promise; }

        // The remaining methods are used by extension awaiters.
        bool client_await_ready  () const                               { return promise->client_await_ready(); }
        bool client_await_notify (void* waiting, NotifierType notifier) { return promise->client_await_notify (waiting, notifier); }
        bool client_await_suspend(void* waiting, ResumerType  resumer ) { return promise->client_await_suspend(waiting, resumer ); }
        T    client_await_resume () &&                                  { return std::exchange(promise, {})->client_await_resume(); }

        // Constructing from task_promise<T>* cannot be explicit because get_return_object relies on implicit conversion.
        task(task_promise<T>* initial = nullptr) noexcept : promise(initial) {}

    private:
        promise_ptr<T> promise;
    };

    struct [[nodiscard]] DelayAwaitable
    {
        Cpu::PerformanceTime TargetTime;

        bool await_ready() const
        {
            return TargetTime <= Cpu::GetPerformanceCounter();
        }

        auto await_suspend(std::coroutine_handle<> handle) const
        {
            if (TargetTime <= Cpu::GetPerformanceCounter())
            {
                // Already ready, no need to suspend.
                return false;
            }
            Timer::ScheduleSparkAtTime(TargetTime, Scheduler::MakeUserModeSpark(
                [](uintptr_t handleAddress) -> Scheduler::Spark
                {
                    // Resume the coroutine when the delay is over.
                    std::coroutine_handle<>::from_address(reinterpret_cast<void*>(handleAddress)).resume();
                    return {};
                },
                reinterpret_cast<uintptr_t>(handle.address())
            ));
            return true;
        }

        void await_resume() const {}
    };

    inline DelayAwaitable Delay(std::chrono::microseconds delay)
    {
        return DelayAwaitable{ Cpu::GetPerformanceCounter() + Cpu::ToTicks(delay) };
    }

    template < typename T >
    inline T WaitOnTask(Async::task<T>&& task)
    {
        while (!task.client_await_ready())
        {
            Cpu::Delay(100us);
        }

        //usbInitTask.client_await_notify(nullptr, [](void*){});
        return std::move(task).client_await_resume();
    }

}
// namespace Async
