
#pragma once

#include "Cpu.h"

#include <stdint.h>

#include <atomic>
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

// Returns the lowest bit position that starts a sequence of N consecutive zeros.
// The position must be aligned to `alignment` (i.e., position % alignment == 0).
// `data` is considered infinitely zero-extended, so finding some position <= 64 is guaranteed.
// If N is zero, 0 is returned.
// If alignment is zero, this behaves the same as FindConsecutiveZeros.
// If alignment is >= 64 and the consecutive zeros are not found right at the beginning, 64 is returned.
inline uint32_t FindConsecutiveZerosAligned(uint64_t data, uint32_t N, uint32_t alignment)
{
    if (N == 0) return 0;
    if (alignment <= 1) return FindConsecutiveZeros(data, N);

    if (alignment & (alignment - 1))
    {
        Cpu::Panic("Alignment must be a power of two");
    }

    // We can't work with N > 64. And we don't care because once we find 64,
    // we've found any greater number by way of the infinite zero extension.
    if (N > 64) N = 64;

    auto const highestPos = 64 - N;

    // Create mask for N consecutive bits
    uint64_t const mask = UINT64_MAX >> highestPos;

    uint32_t pos = 0;
    // Search in the data
    for (;;)
    {
        pos += std::countr_one(data >> pos);
        pos = (pos + alignment - 1) & ~(alignment - 1); // Align to the next alignment boundary
        if (pos >= 64)
        {
            // If we reached the end of the data, we can stop.
            return 64;
        }

        auto const datamask = data >> pos;
        if ((datamask & mask) == 0)
        {
            return pos;
        }

        // Not enough zeros, so skip however many zeros may be then skip the ones after.
        // One of these is guaranteed to 
        pos += std::countr_zero(data >> pos);
    }
}

constexpr inline char PoolAllocatorName[] = "PoolAllocator";

template < uint32_t PoolSize, char const* name = PoolAllocatorName >
    requires (PoolSize > 0)
class PoolAllocator
{
    static_assert(PoolSize > 0, "Pool size too small for current implementation");

    static constexpr uint32_t WordCount = (PoolSize + 63) / 64;

    struct alignas(16) Word {
        uint64_t bitmap = 0;
        uint64_t starts = 0;
    };

    Word Words[WordCount]{};

    // x64 should support 16-byte atomics via cmpxchg16b instruction
    // Note: Currently using mutex-based atomics due to MSVC STL ABI limitations
    // TODO: Enable once we configure proper lock-free 16-byte atomic support
    static_assert(std::atomic_ref<Word>::is_always_lock_free, "Atomic Word must be lock-free");

public:
    // If we have a partial word at the end, we need to "allocate" it to simplify the allocation logic.
    PoolAllocator()
    {
        if constexpr (PoolSize % 64 != 0)
        {
            Words[PoolSize / 64] = Word{
                .bitmap = UINT64_MAX << (PoolSize % 64),
                .starts = 1ull << (PoolSize % 64)
            };
        }
    }

    uint32_t Allocate(uint32_t count, uint32_t alignment = 1)
    {
        uint32_t hint = 0;
        for (;;)
        {
            auto const [result, done] = AllocateInternal(hint, count, alignment);
            if (done)
            {
                return result;
            }
            // If we failed to allocate due to contention, we need to retry, providing the given result hint.
            // This is better than just a goto, right?
            hint = result;
        }
    }

    std::pair<uint32_t, bool> AllocateInternal(uint32_t hint, uint32_t count, uint32_t alignment)
    {
        if (count == 0)
        {
            Cpu::Panic("%s invalid allocation size", name);
        }
        if (alignment == 0)
        {
            Cpu::Panic("%s invalid alignment", name);
        }

        if (alignment > 1 && (alignment & (alignment - 1)) != 0)
        {
            Cpu::Panic("%s alignment must be a power of two", name);
        }

        if (hint & (alignment - 1))
        {
            Cpu::Panic("%s hint must be aligned to the alignment", name);
        }

        uint32_t const wordAlignment = (alignment + 63) / 64;
        uint32_t const wordAlignmentMask = UINT32_MAX * wordAlignment;

        for (uint32_t wordIndex = hint / 64; wordIndex < WordCount; wordIndex = (wordIndex + wordAlignment) & wordAlignmentMask)
        {
            uint32_t const startWordIndex = wordIndex;
            Word currentWord = Words[wordIndex];
            auto const word = currentWord.bitmap;
            if (word == UINT64_MAX)
            {
                continue;
            }
            
            auto bitIndex = FindConsecutiveZerosAligned(word, count, alignment);
            if (bitIndex == 64)
            {
                // Can't start in this word, so continue to the next word.
                continue;
            }

            auto const result = bitIndex + wordIndex * 64;
            if (bitIndex + count <= 64)
            {
                // Found entirely within one word.
                Word const newWord{
                    .bitmap = currentWord.bitmap | (UINT64_MAX >> (64 - count)) << bitIndex,
                    .starts = currentWord.starts | (1ull << bitIndex),
                };
                if (std::atomic_ref{ Words[wordIndex] }.compare_exchange_strong(currentWord, newWord, std::memory_order_acquire))
                {
                    return { result, true };
                }
                else
                {
                    return { result, false };
                }
            }

            // We found some zeros at the end of the word, but not enough.
            // This is promising, but we need to keep looking.
            uint32_t countRemaining = count - (64 - bitIndex);
            uint32_t nextWordIndex = wordIndex + 1;

            // Find as many completely free words as we need.
            while (countRemaining >= 64)
            {
                if (nextWordIndex >= WordCount)
                {
                    // No words left to check, so we can't allocate.
                    // Even if we retried now, we wouldn't be able to fit it.
                    return { UINT32_MAX, true };
                }
                Word nextWord = Words[nextWordIndex];
                if (nextWord.bitmap != 0)
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
            if (countRemaining > 0)
            {
                if (nextWordIndex >= WordCount)
                {
                    // No words left to check, so we can't allocate.
                    return { UINT32_MAX, true };
                }

                // We have a remainder of zeros to find at the end of this next word.
                Word nextWordData = Words[nextWordIndex];
                if ((nextWordData.bitmap << (64 - countRemaining)) != 0)
                {
                    // Not enough zeros at the end of this word, so we can't allocate here.
                    // Note: We keep any skipped words, as we couldn't possibly allocate anywhere in there.
                    wordIndex = nextWordIndex - 1;
                    continue;
                }
            }

            // Ok, we found the allocation, now we need to (attempt to) commit it, lock-free style.
            // If we find a contention, we'll need to retry.

            Word const newWord{
                .bitmap = currentWord.bitmap | (UINT64_MAX << bitIndex),
                .starts = currentWord.starts | (1ull << bitIndex),
            };
            //printf("Allocating %s: wordIndex: %u, bitmap: 0x%016llx, starts: 0x%016llx, mask: 0x%016llx (start)\n", name, wordIndex, currentWord.bitmap, currentWord.starts, (UINT64_MAX << bitIndex));
            if (!std::atomic_ref{ Words[wordIndex] }.compare_exchange_strong(currentWord, newWord, std::memory_order_acquire))
            {
                // If we failed to allocate right on the first word, so no need to do any more cleanup.
                // We'll just need to retry.
                return { result, false };
            }

            // Now we've changed something, so we'll need to clean up if we encounter contention.
            bool cleanupNeeded = false;
            for (wordIndex += 1; wordIndex < nextWordIndex; ++wordIndex)
            {
                Word empty{ .bitmap = 0, .starts = 0 };
                Word const fullWord{.bitmap = UINT64_MAX, .starts = 0};
                if (!std::atomic_ref{ Words[wordIndex] }.compare_exchange_strong(empty, fullWord, std::memory_order_acquire))
                {
                    cleanupNeeded = true;
                    break;
                }
            }

            if (!cleanupNeeded && countRemaining == 0)
            {
                // All done, successfully allocated.
                return { result, true };
            }

            if (!cleanupNeeded)
            {
                // We have a remainder of zeros to find at the end of this next word.
                Word finalWord = Words[wordIndex];
                uint64_t const mask = UINT64_MAX >> (64 - countRemaining);
                // The bits that we are about to allocate must remain clear.
                // If they are really not, then we are contended and will have to retry.
                finalWord.bitmap &= ~mask;
                finalWord.starts &= ~mask;
                Word const newWord{
                    .bitmap = finalWord.bitmap | mask,
                    .starts = finalWord.starts,
                };
                //printf("Allocating %s: wordIndex: %u, bitmap: 0x%016llx, starts: 0x%016llx, mask: 0x%016llx (end)\n", name, wordIndex, finalWord.bitmap, finalWord.starts, mask);
                if (std::atomic_ref{ Words[wordIndex] }.compare_exchange_strong(finalWord, newWord, std::memory_order_acquire))
                {
                    // All done, successfully allocated.
                    return { result, true };
                }
                // Ok, cleanup is going to be needed. We don't need to clean up the last word because we couldn't change it.
            }

            // Unfortunately, we encountered contention, so we need to clean up.
            // Backwards, which is the only way to do it safely lock-free,
            // protected by the start of block marker which is at the start.
            // Note that wordIndex is the one that we failed to change, so we need to start by decrementing it.
            for (--wordIndex; wordIndex != startWordIndex; --wordIndex)
            {
                // Any words that we did commit whole we can 
                Word empty{ .bitmap = 0, .starts = 0 };
                std::atomic_ref{ Words[wordIndex] }.store(empty, std::memory_order_release);
            }

            currentWord = newWord;
            for (;;)
            {
                Word undoneWord{
                    .bitmap = currentWord.bitmap & ~(INT64_MAX << bitIndex),
                    .starts = currentWord.starts & ~(1ull << bitIndex),
                };
                if (std::atomic_ref{ Words[wordIndex] }.compare_exchange_strong(currentWord, undoneWord, std::memory_order_release))
                {
                    // Cleanup is now successful. Go retry.
                    return { result, false };
                }
                // Retry.
            }
        }
        // Nothing found, so we can't allocate.
        return { UINT32_MAX, true };
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
            uint64_t const mask = ~((UINT64_MAX >> (64 - count)) << bitIndex);
            Word currentWord = Words[wordIndex];
            for (;;)
            {
                //printf("Deallocating %s: wordIndex: %u, bitIndex: %u, bitmap: 0x%016llx, starts: 0x%016llx, mask: 0x%016llx\n", name, wordIndex, bitIndex, currentWord.bitmap, currentWord.starts, mask);
                if ((currentWord.bitmap & ~mask) != ~mask || (currentWord.starts & ~mask) != (1ull << bitIndex))
                {
                    Cpu::Panic("%s trying to deallocate a block that is not allocated", name);
                }
                Word const newWord{
                    .bitmap = currentWord.bitmap & mask,
                    .starts = currentWord.starts & mask,
                };
                if (std::atomic_ref{ Words[wordIndex] }.compare_exchange_strong(currentWord, newWord, std::memory_order_release))
                {
                    // Successfully cleared the bits.
                    return;
                }
            }
        }

        uint32_t startWordIndex = wordIndex;
        uint32_t const end = start + count;

        // We need to clear the words backwards, which is the only way to do it safely.
        // Any observer will just see an allocation shrinking, so they can't trample it.
        wordIndex = (end - 1) / 64;
        if (wordIndex >= WordCount)
        {
            Cpu::Panic("%s some math is wrong, we're trying to deallocate beyond the end of the pool", name);
        }

        // Clear the bits in the last word, if it's not full.
        if (end % 64 != 0)
        {
            uint64_t mask = UINT64_MAX << (end % 64);
            Word currentWord = Words[wordIndex];
            for (;;)
            {
                //printf("Deallocating %s: wordIndex: %u, bitmap: 0x%016llx, starts: 0x%016llx, mask: 0x%016llx\n", name, wordIndex, currentWord.bitmap, currentWord.starts, mask);
                if ((currentWord.bitmap & ~mask) != ~mask || (currentWord.starts & ~mask) != 0)
                {
                    Cpu::Panic("%s trying to deallocate a block that is not allocated", name);
                }
                Word const newWord{
                    .bitmap = currentWord.bitmap & mask,
                    .starts = currentWord.starts & mask,
                };
                if (std::atomic_ref{ Words[wordIndex] }.compare_exchange_strong(currentWord, newWord, std::memory_order_release))
                {
                    break;
                }
            }
            --wordIndex;
        }

        for (; wordIndex != startWordIndex; --wordIndex)
        {
            // Clear the entire word, as we know it is completely free.
            Word empty{ .bitmap = 0, .starts = 0 };
            std::atomic_ref{ Words[wordIndex] }.store(empty, std::memory_order_release);
        }

        // Clear the bits in the first word
        {
            uint64_t mask = ~(UINT64_MAX << bitIndex);
            Word currentWord = Words[wordIndex];
            for (;;)
            {
                //printf("Deallocating %s: wordIndex: %u, bitmap: 0x%016llx, starts: 0x%016llx, mask: 0x%016llx\n", name, wordIndex, currentWord.bitmap, currentWord.starts, mask);
                if ((currentWord.bitmap & ~mask) != ~mask || (currentWord.starts & ~mask) != (1ull << bitIndex))
                {
                    Cpu::Panic("%s trying to deallocate a block that is not allocated", name);
                }
                Word const newWord{
                    .bitmap = currentWord.bitmap & mask,
                    .starts = currentWord.starts & mask,
                };
                if (std::atomic_ref{ Words[wordIndex] }.compare_exchange_strong(currentWord, newWord, std::memory_order_release))
                {
                    // Successfully cleared the bits.
                    break;
                }
            }
        }
    }

    struct BlockInformation
    {
        uint32_t Start;
        uint32_t Size;
        bool     IsAllocated;
    };
    BlockInformation GetContainingBlockInformation(uint32_t address) const
    {
        if (address >= PoolSize)
        {
            Cpu::Panic("%s invalid block start", name);
        }

        BlockInformation result;

        for (;;)
        {
            uint32_t wordIndex = address / 64;
            uint32_t bitIndex  = address % 64;

            Word currentWord = std::atomic_ref{ const_cast<Word&>(Words[wordIndex]) }.load(std::memory_order_relaxed);
            if (!(currentWord.starts & (1ull << bitIndex)))
            {
                result.IsAllocated = false;

                // Need to find the start.
                auto startIndex = address;
                if (bitIndex > 0)
                {
                    startIndex -= std::countl_one(~currentWord.bitmap << (64 - bitIndex));
                }
                auto startWordIndex = wordIndex;
                while (startWordIndex > 0 && startIndex % 64 == 0)
                {
                    --startWordIndex;
                    Word startWord = std::atomic_ref{ const_cast<Word&>(Words[startWordIndex]) }.load(std::memory_order_relaxed);
                    startWordIndex -= std::countl_zero(startWord.bitmap);
                }

                // And now find the end.
                auto const endIndex = address + std::countr_one(~currentWord.bitmap >> bitIndex);
                auto endWordIndex = wordIndex;
                while (endWordIndex < WordCount && endIndex % 64 == 0)
                {
                    ++endWordIndex;
                    Word endWord = std::atomic_ref{ const_cast<Word&>(Words[endWordIndex]) }.load(std::memory_order_relaxed);
                    endIndex += std::countr_zero(endWord.bitmap);
                }

                endIndex = std::min(endIndex, PoolSize);

                result.Start = startIndex;
                result.Size  = endIndex - startIndex;
            }
            else
            {
                result.IsAllocated = true;
            }
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

        Word currentWord = std::atomic_ref{ const_cast<Word&>(Words[wordIndex]) }.load(std::memory_order_relaxed);
        if (!(currentWord.starts & (1ull << bitIndex)))
        {
            //printf("wordIndex: %u, bitIndex: %u, Starts[wordIndex]: 0x%016llx\n", wordIndex, bitIndex, currentWord.starts);
            Cpu::Panic("%s trying to get size of a block 0x%X that is not allocated", name, blockStart);
        }

        //printf("GetBlockSize %s: wordIndex: %u, bitIndex: %u, bitmap: 0x%016llx, starts: 0x%016llx\n", name, wordIndex, bitIndex, currentWord.bitmap, currentWord.starts);

        uint32_t count = 1;
        if (bitIndex + 1 < 64)
        {
            count += std::countr_one((currentWord.bitmap ^ currentWord.starts) >> (bitIndex + 1));
        }
        auto position = bitIndex + count;
        while (position == 64 && ++wordIndex < WordCount)
        {
            Word nextWord = std::atomic_ref{ const_cast<Word&>(Words[wordIndex]) }.load(std::memory_order_relaxed);
            //printf("GetBlockSize %s: wordIndex: %u, bitmap: 0x%016llx, starts: 0x%016llx\n", name, wordIndex, nextWord.bitmap, nextWord.starts);

            position = std::countr_one(nextWord.bitmap ^ nextWord.starts);
            count += position;
        }
        return count;
    }
};

}
// namespace Containers
