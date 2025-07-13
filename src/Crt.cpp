
#include "Cpu.h"
#include "Uart.h"

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <wchar.h>

#include <new>
#include <memory>


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

void* memcpy(void* restrict s1, const void* restrict s2, size_t n)
{
    uint8_t* d = static_cast<uint8_t*>(s1);
    const uint8_t* s = static_cast<const uint8_t*>(s2);
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
        d = reinterpret_cast<uint8_t*>(qd);
        s = reinterpret_cast<const uint8_t*>(qs);
    }
    for (; n > 0; --n)
    {
        *d++ = *s++;
    }
    return s1;
}

void* memmove(void* dest, const void* src, size_t n)
{
    uint8_t* d = static_cast<uint8_t*>(dest);
    const uint8_t* s = static_cast<const uint8_t*>(src);
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

int strcmp(const char* s1, const char* s2)
{
    while (*s1 && *s2 && *s1 == *s2)
    {
        ++s1;
        ++s2;
    }
    return static_cast<uint8_t>(*s1) - static_cast<uint8_t>(*s2);
}

int strncmp(const char* s1, const char* s2, size_t n)
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

void* operator new(size_t size, std::align_val_t align)
{
    auto const pos = (currentHeapPos + static_cast<size_t>(align) - 1) & ~(static_cast<size_t>(align) - 1);
    if (pos + size > HeapSizeInBytes)
    {
        Cpu::Panic("Out of memory in operator new");
    }
    void* result = reinterpret_cast<void*>(HeapStartAddress + pos);
    currentHeapPos = pos + size;
    return result;
}

void* operator new[](size_t size, std::align_val_t align) { return operator new(size, align); }

void* operator new(size_t size) { return operator new(size, std::align_val_t{ 16 }); }
void* operator new[](size_t size) { return operator new(size, std::align_val_t{ 16 }); }

void operator delete(void* ptr, size_t size, std::align_val_t) noexcept
{
    if (ptr == nullptr)
    {
        return; // No action for null pointer
    }
    if (reinterpret_cast<uintptr_t>(ptr) == HeapStartAddress + currentHeapPos - size)
    {
        currentHeapPos -= size; // Deallocate only if it matches the last allocation
    }

    // Simple heap implementation - no actual deallocation unless it's from the top.
}

void operator delete[](void* ptr, size_t size, std::align_val_t align) noexcept { return operator delete(ptr, size, align); }

void operator delete(void* ptr, size_t size) noexcept { return operator delete(ptr, size, std::align_val_t{ 16 }); }
void operator delete[](void* ptr, size_t size) noexcept { return operator delete(ptr, size, std::align_val_t{ 16 }); }

void operator delete(void* ptr, std::align_val_t) noexcept
{
    if (ptr == nullptr)
    {
        // No action for null pointer
        return;
    }

    // Simple heap implementation - no actual deallocation
}

void operator delete[](void* ptr, std::align_val_t align) noexcept { return operator delete(ptr, align); }

void operator delete(void* ptr) noexcept { return operator delete(ptr, std::align_val_t{ 16 }); }
void operator delete[](void* ptr) noexcept { return operator delete(ptr, std::align_val_t{ 16 }); }

extern "C" void *aligned_alloc(size_t alignment, size_t size)
{
    if (alignment == 0 || (alignment & (alignment - 1)) != 0)
    {
        return nullptr; // Invalid alignment
    }

    auto const pos = (currentHeapPos + static_cast<size_t>(alignment) - 1) & ~(static_cast<size_t>(alignment) - 1);
    if (pos + size > HeapSizeInBytes)
    {
        return nullptr; // Out of memory
    }

    void* result = reinterpret_cast<void*>(HeapStartAddress + pos);
    currentHeapPos = pos + size;
    return result;
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

extern "C" void __cxa_pure_virtual()
{
    Uart::Raw::Puts("Pure virtual function called\n");
    Cpu::Halt();
}



namespace std
{

void terminate()
{
    Uart::Raw::Puts("std::terminate called\n");
    Cpu::Halt();
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

}
// namespace std
