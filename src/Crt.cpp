
//#include "Cpu.h"
//#include "Uart.h"
#include "Heap.h"
//#include "Debugger.h"

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <wchar.h>

#include <new>
#include <memory>
#include <atomic>
#include <string>

namespace Cpu { [[noreturn]] void Panic(char const* fmt, ...); }

extern "C"
{

void* memset(void* const dst, int val, size_t size)
{
    uint8_t* p = static_cast<uint8_t*>(dst);
    size_t firstBytes = (uintptr_t)p & 7;
    firstBytes = firstBytes > size ? size : firstBytes;
    size -= firstBytes;
    for (; firstBytes > 0; --firstBytes)
    {
        *p++ = static_cast<uint8_t>(val);
    }
    size_t wholeQwords = size / 8;
    if (wholeQwords > 0)
    {
        size -= wholeQwords * 8;
        uint64_t const val64 = static_cast<uint8_t>(val) * 0x0101'0101'0101'0101u;
        uint64_t* q = reinterpret_cast<uint64_t*>(p);
        for (; wholeQwords > 0; --wholeQwords)
        {
            *q++ = val64;
        }
        p = reinterpret_cast<uint8_t*>(q);
    }
    for (; size > 0; --size)
    {
        *p++ = static_cast<uint8_t>(val);
    }
    return dst;
}

void* memcpy(void* restrict s1, void const* restrict s2, size_t n)
{
    uint8_t* d = static_cast<uint8_t*>(s1);
    uint8_t const* s = static_cast<uint8_t const*>(s2);
    size_t firstBytes = (uintptr_t)d & 7;
    firstBytes = firstBytes > n ? n : firstBytes;
    n -= firstBytes;
    for (; firstBytes > 0; --firstBytes)
    {
        *d++ = *s++;
    }
    size_t wholeQwords = n / 8;
    if (wholeQwords > 0)
    {
        n -= wholeQwords * 8;
        uint64_t* qd = reinterpret_cast<uint64_t*>(d);
        uint64_t const* qs = reinterpret_cast<uint64_t const*>(s);
        for (; wholeQwords > 0; --wholeQwords)
        {
            *qd++ = *qs++;
        }
        d = reinterpret_cast<uint8_t*>(qd);
        s = reinterpret_cast<uint8_t const*>(qs);
    }
    for (; n > 0; --n)
    {
        *d++ = *s++;
    }
    return s1;
}

void* memmove(void* dest, void const* src, size_t n)
{
    uint8_t* d = static_cast<uint8_t*>(dest);
    uint8_t const* s = static_cast<uint8_t const*>(src);
    if (d < s || d >= s + n)
    {
        // No overlap or forward copy
        return memcpy(dest, src, n);
    }
    else
    {
        // Backward copy
        d += n;
        s += n;
        while (n-- > 0)
        {
            *(--d) = *(--s);
        }
        return dest;
    }
}

int memcmp(void const* restrict s1, void const* restrict s2, size_t n)
{
    uint8_t const* d = static_cast<uint8_t const*>(s1);
    uint8_t const* s = static_cast<uint8_t const*>(s2);
    size_t firstBytes = (uintptr_t)d & 7;
    firstBytes = firstBytes > n ? n : firstBytes;
    n -= firstBytes;
    for (; firstBytes > 0; --firstBytes)
    {
        if (*d++ != *s++) return static_cast<uint8_t>(*(d - 1)) < static_cast<uint8_t>(*(s - 1)) ? -1 : 1;
    }
    size_t wholeQwords = n / 8;
    if (wholeQwords > 0)
    {
        n -= wholeQwords * 8;
        uint64_t const* qd = reinterpret_cast<uint64_t const*>(d);
        uint64_t const* qs = reinterpret_cast<uint64_t const*>(s);
        for (; wholeQwords > 0; --wholeQwords)
        {
            if (*qd++ != *qs++) return static_cast<uint8_t>(*(qd - 1)) < static_cast<uint8_t>(*(qs - 1)) ? -1 : 1;
        }
        d = reinterpret_cast<uint8_t const*>(qd);
        s = reinterpret_cast<uint8_t const*>(qs);
    }
    for (; n > 0; --n)
    {
        if (*d++ != *s++) return static_cast<uint8_t>(*(d - 1)) < static_cast<uint8_t>(*(s - 1)) ? -1 : 1;
    }
    return 0;
}

void* memchr(void* s, int c, size_t n)
{
    uint8_t* p = static_cast<uint8_t*>(s);
    for (size_t i = 0; i < n; ++i)
    {
        if (p[i] == static_cast<uint8_t>(c))
        {
            return static_cast<void*>(&p[i]);
        }
    }
    return nullptr;
}

int wctob(wint_t c)
{
    if (c > 255)
    {
        return EOF;
    }
    return static_cast<char>(c);
}

size_t strlen(char const* s)
{
    char const* p = s;
    while (*p != '\0')
    {
        ++p;
    }
    return static_cast<size_t>(p - s);
}

int strcmp(char const* s1, char const* s2)
{
    while (*s1 && *s2 && *s1 == *s2)
    {
        ++s1;
        ++s2;
    }
    return static_cast<uint8_t>(*s1) - static_cast<uint8_t>(*s2);
}

int strncmp(char const* s1, char const* s2, size_t n)
{
    if (n == 0)
    {
        return 0;
    }
    
    while (n > 0 && *s1 && *s2 && *s1 == *s2)
    {
        ++s1;
        ++s2;
        --n;
    }
    
    if (n == 0)
    {
        return 0;
    }
    
    return static_cast<uint8_t>(*s1) - static_cast<uint8_t>(*s2);
}

void* __dso_handle = nullptr;

int __cxa_atexit(void (*func)(void*), void* arg, void* dso_handle)
{
    // Simple implementation that ignores atexit functions
    // In a full implementation, these would be stored and called at program exit
    (void)func;
    (void)arg;
    (void)dso_handle;
    return 0; // Success
}

}
// extern "C"

uintptr_t HeapStartAddress = 0x1000'0000u;
uintptr_t HeapSizeInBytes  = 0x1000'0000u;

constexpr std::align_val_t MinHeapAlign{ 16 };

static std::atomic<uintptr_t> CurrentHeapPos = 0;

Heap::Heap* g_globalHeap = nullptr;

void InitGlobalHeap()
{
    g_globalHeap = Heap::CreateHeap();
    if (!g_globalHeap)
    {
        Cpu::Panic("Failed to create global heap");
    }
}

void *HeapAlloc(size_t size, std::align_val_t alignVal)
{
    auto const alignment = static_cast<size_t>(alignVal);
    if (alignment == 0 || (alignment & (alignment - 1)) != 0)
    {
        return nullptr; // Invalid alignment
    }

    
    auto const result = Heap::AllocateAligned(g_globalHeap, size, alignment);
//    Uart::Puts("HeapAlloc: Allocated ");
//    Uart::PutDec(size);
//    Uart::Puts(" bytes at ");
//    Uart::PutHex(result);
//    Uart::Puts("\n");
//    Debugger::RawPrintCallstack();

    return result;

/*
    auto currentHeapPos = CurrentHeapPos.load(std::memory_order_relaxed);

    for (;;)
    {
        auto const pos = (currentHeapPos + static_cast<size_t>(alignment) - 1) & ~(static_cast<size_t>(alignment) - 1);
        if (pos + size > HeapSizeInBytes)
        {
            return nullptr; // Out of memory
        }

        void* result = reinterpret_cast<void*>(HeapStartAddress + pos);
        if (CurrentHeapPos.compare_exchange_strong(currentHeapPos, pos + size, std::memory_order_acquire))
        {
            return result;
        }
    }
*/
}

void HeapFree(void* ptr, size_t size) noexcept
{
    if (ptr == nullptr)
    {
        return; // No action for null pointer
    }

    if (size == 0)
    {
        return Heap::Deallocate(g_globalHeap, ptr);
    }

//    Uart::Puts("HeapAlloc: Freeing from ");
//    Uart::PutHex(ptr);
//    Uart::Puts("\n");

    // Simple heap implementation - no actual deallocation unless it's from the top.

    Heap::Deallocate(g_globalHeap, ptr);

//    auto currentHeapPos = CurrentHeapPos.load(std::memory_order_relaxed);
//
//    while (reinterpret_cast<uintptr_t>(ptr) == HeapStartAddress + currentHeapPos - size)
//    {
//        // Deallocate only if it matches the last allocation.
//        // Note that we're only deallocating the aligned portion.
//        // Any alignment padding we incurred during allocation is lost.
//        if (CurrentHeapPos.compare_exchange_strong(currentHeapPos, currentHeapPos - size, std::memory_order_release))
//        {
//            // Successfully deallocated
//            return;
//        }
//    }
}

extern "C" void *aligned_alloc(size_t alignment, size_t size) { return HeapAlloc(size, std::align_val_t{ alignment }); }

extern "C" void* malloc(size_t size) { return HeapAlloc(size, MinHeapAlign); }

extern "C" void free(void* ptr) noexcept { HeapFree(ptr, 0); }

void* operator new  (size_t size, std::align_val_t align) { return HeapAlloc(size, align); }
void* operator new[](size_t size, std::align_val_t align) { return HeapAlloc(size, align); }

void* operator new  (size_t size) { return HeapAlloc(size, MinHeapAlign); }
void* operator new[](size_t size) { return HeapAlloc(size, MinHeapAlign); }

void operator delete  (void* ptr, size_t size, std::align_val_t) noexcept { HeapFree(ptr, size); }
void operator delete[](void* ptr, size_t size, std::align_val_t) noexcept { HeapFree(ptr, size); }

void operator delete  (void* ptr, size_t size) noexcept { HeapFree(ptr, size); }
void operator delete[](void* ptr, size_t size) noexcept { HeapFree(ptr, size); }

void operator delete  (void* ptr, std::align_val_t) noexcept { HeapFree(ptr, 0); }
void operator delete[](void* ptr, std::align_val_t) noexcept { HeapFree(ptr, 0); }

void operator delete  (void* ptr) noexcept { HeapFree(ptr, 0); }
void operator delete[](void* ptr) noexcept { HeapFree(ptr, 0); }

extern "C" void abort()
{
    Cpu::Panic("abort called");
}

extern "C" void __cxa_pure_virtual()
{
    Cpu::Panic("Pure virtual function called");
}



namespace std
{

void terminate()
{
    Cpu::Panic("std::terminate called");
}


ABI::__shared_count     ::~__shared_count     () = default;
ABI::__shared_weak_count::~__shared_weak_count() = default;

void ABI::__shared_weak_count::__release_weak() noexcept
{
    // TODO: Do this. For now, we won't be using weak pointers.
}

void const* ABI::__shared_weak_count::__get_deleter(std::type_info const&) const noexcept
{
    // TODO: Implement this
    return nullptr;
}

template class basic_string<char, std::char_traits<char>, std::allocator<char>>;

}
// namespace std
