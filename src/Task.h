#pragma once

#include <coroutine>

// As we ramp up multithreading and interrupts, using coroutines
// can result in much cleaner code for anything that normally would wait.
// TODO: Do this properly, of course.
struct Task
{
    struct promise_type
    {
        std::suspend_always initial_suspend();
        std::suspend_always final_suspend() noexcept;
        void return_value(int value) noexcept {}
        Task get_return_object() noexcept { return Task{}; }
        void unhandled_exception() noexcept {}

        void* operator new(size_t size) noexcept
        {
            return nullptr;
        }

        int blah;
    };
    int blah;
};

Task Coro()
{
    co_return 0;
}
