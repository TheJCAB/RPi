
#include "Cpu.h"

#include <stdint.h>

#include <bit>

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

// Returns the lowest bit position that starts a sequence of N consecutive zeros for 0 < N <= 64.
// `data` is considered infinitely zero-extended, so finding some position is guaranteed.
inline uint32_t FindConsecutiveZeros(uint64_t data, uint32_t N)
{
    if (N == 0) return 0;
    if (N > 64) return 64;

    auto const highestPos = 64 - N;

    // Create mask for N consecutive bits
    uint64_t const mask = UINT64_MAX >> highestPos;

    // Search in the data
    uint32_t pos = std::countr_one(data);
    for (;;)
    {
        auto const datamask = data >> pos;
        if ((datamask & mask) == 0)
        {
            return pos;
        }

        // Not enough zeros, so skip however many zeros may be then skip the ones after.
        // One of these is guaranteed to 
        pos += std::countr_zero(data >> pos);
        pos += std::countr_one (data >> pos);
        if (pos >= 64)
        {
            // If we reached the end of the data, we can stop.
            return 64;
        }
    }
}

// Returns the lowest bit position in data0 that starts a sequence of N consecutive zeros for 0 < N <= 64.
// `data0` is considered extended with `data1`.
// If no such position is found, 64 is returned.
inline uint32_t FindConsecutiveZeros(uint64_t data0, uint64_t data1, uint32_t N)
{
    if (N == 0) return 0;
    if (N > 64) return 64;

    auto pos = FindConsecutiveZeros(data0, N);
    if (pos + N <= 64)
    {
        // Found in the first word
        return pos;
    }

    if (pos < 64)
    {
        // We found some zeros in the first word, but not enough.
        // Check if the next word has enough zeros.
        if ((data1 << (128 - (pos + N))) == 0)
        {
            return pos;
        }
    }

    return 64;
}

constexpr inline char PoolAllocatorName[] = "PoolAllocator";

template < uint32_t PoolSize, char const* name = PoolAllocatorName >
struct PoolAllocator
{
    static_assert(PoolSize <= 64, "Pool size too small for current implementation");

    static constexpr uint32_t WordCount = (PoolSize + 63) / 64;

    uint64_t Bitmap[WordCount]{};
    uint64_t Starts[WordCount]{};

    uint32_t Allocate(uint32_t count)
    {
        if (count == 0 || count > 64)
        {
            Cpu::Panic("%s invalid allocation size", name);
        }

        for (uint32_t wordIndex = 0; wordIndex < WordCount; ++wordIndex)
        {
            auto const word = Bitmap[wordIndex];
            if (word == UINT64_MAX)
            {
                continue;
            }
            auto bitIndex = FindConsecutiveZeros(word, count);
            if (bitIndex + count <= 64)
            {
                // Found.
                auto const result = bitIndex + wordIndex * 64;
                if (result + count > PoolSize)
                {
                    // Not enough space in the pool.
                    return UINT32_MAX;
                }
                Bitmap[wordIndex] |= (UINT64_MAX >> (64 - count)) << bitIndex;
                Starts[wordIndex] |= (1ull << bitIndex);
                return result;
            }

            if (bitIndex >= 64)
            {
                // Can't start in this word, so continue to the next word.
                continue;
            }

            if (wordIndex + 1 >= WordCount)
            {
                // No next word to check, so we can't allocate.
                return UINT32_MAX;
            }

            // We found some zeros in the word, but not enough.
            // Check if the next word has enough zeros.
            auto const nextWord = Bitmap[wordIndex + 1];
            if ((nextWord << (128 - (bitIndex + count))) == 0)
            {
                // Found.
                auto const result = bitIndex + wordIndex * 64;
                if (result + count > PoolSize)
                {
                    // Not enough space in the pool.
                    return UINT32_MAX;
                }
                Bitmap[wordIndex] |= UINT64_MAX << bitIndex;
                Starts[wordIndex] |= (1ull << bitIndex);
                Bitmap[wordIndex + 1] |= UINT64_MAX >> (128 - (bitIndex + count));
                return result;
            }
        }
        // Nothing found, so we can't allocate.
        return UINT32_MAX;
    }
    
    void Deallocate(uint32_t start)
    {
        if (start >= PoolSize)
        {
            Cpu::Panic("%s invalid deallocation start", name);
        }

        uint32_t wordIndex = start / 64;
        uint32_t bitIndex = start % 64;

        if (!(Starts[wordIndex] & (1ull << bitIndex)))
        {
            Cpu::Panic("%s trying to deallocate a block that is not allocated", name);
        }

        uint32_t count = 1;
        if (bitIndex + 1 < 64)
        {
            count += std::countr_one((Bitmap[wordIndex] ^ Starts[wordIndex]) >> (bitIndex + 1));
        }

        // Clear the allocated bits in the bitmap
        Bitmap[wordIndex] &= ~((UINT64_MAX >> (64 - count)) << bitIndex);
        Starts[wordIndex] &= ~(1ull << bitIndex);

        if (bitIndex + count == 64 && wordIndex + 1 < WordCount)
        {
            // The block may span multiple words, so we need to count the next word as well.
            auto const countHi = std::countr_one(Bitmap[wordIndex + 1] ^ Starts[wordIndex + 1]);
            if (countHi > 0)
            {
                // Clear the next word as well
                Bitmap[wordIndex + 1] &= ~(UINT64_MAX >> (64 - countHi));
                Starts[wordIndex + 1] &= ~(1ull << 0); // Clear the first bit of the next word
                count += countHi;
            }
        }
        if (count > 64)
        {
            Cpu::Panic("%s found an apparent block that is bigger than the maximum allowed size", name);
        }
    }
    
//    uint32_t GetFreeCount() const
//    {
//    }

private:
};

}
// namespace Containers
