
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

// Returns the lowest bit position that starts a sequence of N consecutive zeros.
// `data` is considered infinitely zero-extended, so finding some position <= 64is guaranteed.
// If N is zero, 0 is returned.
inline uint32_t FindConsecutiveZeros(uint64_t data, uint32_t N)
{
    if (N == 0) return 0;

    // We can't work with N > 64. And we don't care because once we find 64,
    // we've found any greater number by way of the infinite zero extension.
    if (N > 64) N = 64;

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

constexpr inline char PoolAllocatorName[] = "PoolAllocator";

template < uint32_t PoolSize, char const* name = PoolAllocatorName >
    requires (PoolSize > 0)
class PoolAllocator
{
    static_assert(PoolSize > 0, "Pool size too small for current implementation");

    static constexpr uint32_t WordCount = (PoolSize + 63) / 64;

    uint64_t Bitmap[WordCount]{};
    uint64_t Starts[WordCount]{};

public:
    // If we have a partial word at the end, we need to "allocate" it to simplify the allocation logic.
    PoolAllocator()
    {
        if constexpr (PoolSize % 64 != 0)
        {
            Bitmap[PoolSize / 64] = UINT64_MAX << (PoolSize % 64);
            Starts[PoolSize / 64] =       1ull << (PoolSize % 64);
        }
    }

    uint32_t Allocate(uint32_t count)
    {
        if (count == 0)
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
            auto const result = bitIndex + wordIndex * 64;
            if (bitIndex + count <= 64)
            {
                // Found entirely within one word.
                Bitmap[wordIndex] |= (UINT64_MAX >> (64 - count)) << bitIndex;
                Starts[wordIndex] |= (1ull << bitIndex);
                return result;
            }

            if (bitIndex == 64)
            {
                // Can't start in this word, so continue to the next word.
                continue;
            }

            // We found some zeros at the end of the word, but not enough.
            // This is promising, but we need to keep looking.
            uint32_t countRemaining = count - (64 - bitIndex);
            uint32_t nextWordIndex = wordIndex + 1;

            // Find as many completely free words as we need.
            while (countRemaining >= 64)
            {
                if (nextWordIndex < WordCount)
                {
                    // No words left to check, so we can't allocate.
                    // Even if we retried now, we wouldn't be able to fit it.
                    return UINT32_MAX;
                }
                if (Bitmap[nextWordIndex] != 0)
                {
                    // Found a non-zero word, so we can't allocate here after all.
                    break;
                }
                countRemaining -= 64;
                ++nextWordIndex;
            }
            if (countRemaining >= 64)
            {
                // Didn't reach the end of the allocation, so we failed.
                // Note: We keep any skipped words, as we couldn't possibly allocate anywhere in there.
                // nextWordIndex is the word that we couldn't skip, so we want to use it in the next iteration.
                wordIndex = nextWordIndex - 1;
                continue;
            }
            if (countRemaining == 0)
            {
                // Found it!
                Bitmap[wordIndex] |= INT64_MAX << bitIndex;
                Starts[wordIndex] |= (1ull << bitIndex);
                for (wordIndex += 1; wordIndex < nextWordIndex; ++wordIndex)
                {
                    Bitmap[wordIndex] = UINT64_MAX;
                }
                return result;
            }

            // We have a remainder of zeros to find at the end of this next word.
            auto const nextWord = Bitmap[nextWordIndex];
            if ((nextWord << (64 - countRemaining)) == 0)
            {
                // Found.
                Bitmap[wordIndex] |= UINT64_MAX << bitIndex;
                Starts[wordIndex] |= (1ull << bitIndex);
                for (wordIndex += 1; wordIndex < nextWordIndex; ++wordIndex)
                {
                    Bitmap[wordIndex] = UINT64_MAX;
                }
                Bitmap[wordIndex] |= UINT64_MAX >> (64 - countRemaining);
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

        uint32_t count = GetBlockSize(start);

        uint32_t wordIndex = start / 64;
        uint32_t bitIndex = start % 64;

        bool const isSingleWord = bitIndex + count <= 64;
        if (isSingleWord)
        {
            // Clear the allocated bits in the bitmap
            uint32_t mask = ~((UINT64_MAX >> (64 - count)) << bitIndex);
            Bitmap[wordIndex] &= mask;
            Starts[wordIndex] &= mask;
            return;
        }

        // Clear the allocated bits in the bitmap
        {
            uint32_t mask = ~(UINT64_MAX << bitIndex);
            Bitmap[wordIndex] &= mask;
            Starts[wordIndex] &= mask;
        }

        count -= 64 - bitIndex;
        while (count >= 64 && ++wordIndex < WordCount)
        {
            // Clear the next word as well
            Bitmap[wordIndex] = 0;
            Starts[wordIndex] = 0;
            count -= 64;
        }

        if (count == 0)
        {
            // We cleared the entire word, so we can stop here.
            return;
        }

        // We have a remainder of zeros to clear at the beginning of this next word.
        if (wordIndex >= WordCount)
        {
            Cpu::Panic("%s some math is wrong, we're trying to deallocate beyond the end of the pool", name);
        }

        // Clear the remaining bits in the next word
        {
            uint64_t mask = UINT64_MAX >> (64 - count);
            Bitmap[wordIndex] &= mask;
            Starts[wordIndex] &= mask;
        }
    }

    uint32_t GetBlockSize(uint32_t blockStart) const
    {
        if (blockStart >= PoolSize)
        {
            Cpu::Panic("%s invalid block start", name);
        }

        uint32_t wordIndex = blockStart / 64;
        uint32_t bitIndex = blockStart % 64;

        if (!(Starts[wordIndex] & (1ull << bitIndex)))
        {
            Cpu::Panic("%s trying to get size of a block that is not allocated", name);
        }

        uint32_t count = 1;
        if (bitIndex + 1 < 64)
        {
            count += std::countr_one((Bitmap[wordIndex] ^ Starts[wordIndex]) >> (bitIndex + 1));
        }
        auto position = bitIndex + count;
        while (position == 64 && ++wordIndex < WordCount)
        {
            position = std::countr_one(Bitmap[wordIndex] ^ Starts[wordIndex]);
            count += position;
        }
        return count;
    }
};

}
// namespace Containers
