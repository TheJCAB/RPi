
#include "Cpu.h"
#include "Uart.h"

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <wchar.h>

#include <new>


extern "C"
{

void* memset(void* const dst, int val, size_t size)
{
    unsigned char* p = static_cast<unsigned char*>(dst);
    size_t firstBytes = (uintptr_t)p & 7;
    firstBytes = firstBytes > size ? size : firstBytes;
    size -= firstBytes;
    for (; firstBytes > 0; --firstBytes)
    {
        *p++ = static_cast<unsigned char>(val);
    }
    size_t wholeQwords = size / 8;
    if (wholeQwords > 0)
    {
        size -= wholeQwords * 8;
        uint64_t const val64 = static_cast<unsigned char>(val) * 0x0101'0101'0101'0101u;
        uint64_t* q = reinterpret_cast<uint64_t*>(p);
        for (; wholeQwords > 0; --wholeQwords)
        {
            *q++ = val64;
        }
        p = reinterpret_cast<unsigned char*>(q);
    }
    for (; size > 0; --size)
    {
        *p++ = static_cast<unsigned char>(val);
    }
    return dst;
}

void* memcpy(void* restrict s1, const void* restrict s2, size_t n)
{
    unsigned char* d = static_cast<unsigned char*>(s1);
    const unsigned char* s = static_cast<const unsigned char*>(s2);
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
        const uint64_t* qs = reinterpret_cast<const uint64_t*>(s);
        for (; wholeQwords > 0; --wholeQwords)
        {
            *qd++ = *qs++;
        }
        d = reinterpret_cast<unsigned char*>(qd);
        s = reinterpret_cast<const unsigned char*>(qs);
    }
    for (; n > 0; --n)
    {
        *d++ = *s++;
    }
    return s1;
}

int wctob(wint_t c)
{
    if (c > 255)
    {
        return EOF;
    }
    return static_cast<char>(c);
}

size_t strlen(const char* s)
{
    const char* p = s;
    while (*p != '\0')
    {
        ++p;
    }
    return static_cast<size_t>(p - s);
}

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

static uintptr_t currentHeapPos = 0;

void* operator new(size_t size)
{
    size = (size + 15) & ~size_t{ 15 }; // Align to 16 bytes
    if (currentHeapPos + size > HeapSizeInBytes)
    {
        Cpu::Panic("Out of memory in operator new");
    }
    void* result = reinterpret_cast<void*>(HeapStartAddress + currentHeapPos);
    currentHeapPos += size;
    return result;
}

void* operator new[](size_t size)
{
    size = (size + 15) & ~size_t{ 15 }; // Align to 16 bytes
    if (currentHeapPos + size > HeapSizeInBytes)
    {
        Cpu::Panic("Out of memory in operator new[]");
    }
    void* result = reinterpret_cast<void*>(HeapStartAddress + currentHeapPos);
    currentHeapPos += size;
    return result;
}

void operator delete(void* ptr, size_t size) noexcept
{
    if (ptr == nullptr)
    {
        return; // No action for null pointer
    }
    size = (size + 15) & ~size_t{ 15 }; // Align to 16 bytes
    if (reinterpret_cast<uintptr_t>(ptr) == HeapStartAddress + currentHeapPos - size)
    {
        currentHeapPos -= size; // Deallocate only if it matches the last allocation
    }

    // Simple heap implementation - no actual deallocation unless it's from the top.
}

void operator delete[](void* ptr) noexcept
{
    if (ptr == nullptr)
    {
        // No action for null pointer
        return;
    }

    // Simple heap implementation - no actual deallocation
}

extern "C" void* malloc(size_t size)
{
    size = (size + 15) & ~size_t{ 15 }; // Align to 16 bytes
    if (currentHeapPos + size > HeapSizeInBytes)
    {
        Cpu::Panic("Out of memory in malloc");
    }
    void* result = reinterpret_cast<void*>(HeapStartAddress + currentHeapPos);
    currentHeapPos += size;
    return result;
}

extern "C" void free(void* ptr) noexcept
{
    if (ptr == nullptr)
    {
        // No action for null pointer
        return;
    }

    // Simple heap implementation - no actual deallocation
}

extern "C" void abort()
{
    Uart::Raw::Puts("abort called\n");
    Cpu::Halt();
}

namespace std
{

void terminate()
{
    Uart::Raw::Puts("std::terminate called\n");
    Cpu::Halt();
}

}
// namespace std
