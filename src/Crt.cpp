
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <wchar.h>


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

}
// extern "C"
