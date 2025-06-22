
#include <stdint.h>
#include <stddef.h>
#include <string.h>


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

}
// extern "C"
