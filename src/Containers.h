
#include "Cpu.h"

#include <stdint.h>

namespace Containers
{

constexpr inline char CircularFifoName[] = "CircularFifo";

template < typename T, uint32_t capacity, char const* name = CircularFifoName >
struct CircularFifo
{
    T Data[capacity];
    uint32_t Head = 0;
    uint32_t Tail = 0;

    void Push(T const& item)
    {
        Data[Head] = item;
        Head = (Head + 1) % capacity;
        if (Head == Tail)
        {
            Cpu::Panic("%s overflow", name);
        }
    }

    T Pop()
    {
        if (Tail == Head)
        {
            Cpu::Panic("%s is empty", name);
        }
        auto item = Data[Tail];
        Tail = (Tail + 1) % capacity;
        return item;
    }

    bool IsEmpty() const
    {
        return Tail == Head;
    }
};

}
// namespace Containers
