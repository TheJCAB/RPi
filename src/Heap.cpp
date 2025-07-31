#include "Heap.h"

#include "Mmu.h"
#include "Containers.h"
#include "Cpu.h"
#include "emb-stdio.h"

#include <atomic>

namespace Heap
{

// Configuration constants
constexpr uint32_t PageSize              = 4096;  // Standard page size
constexpr uint32_t PagesPerBlock         = 16;    // 64KB blocks
constexpr uint32_t BlockSize             = PageSize * PagesPerBlock;
constexpr uint32_t AllocationGranularity = 16;    // 16-byte granularity
constexpr uint32_t AllocationsPerBlock   = BlockSize / AllocationGranularity;  // 4096 allocations per block
constexpr uint32_t MaxBlocks             = 256;   // Maximum number of blocks per heap

// Pool allocator for managing allocations within each block
constexpr inline char HeapPoolName[] = "HeapPool";
using BlockAllocator = Containers::PoolAllocator<AllocationsPerBlock, HeapPoolName>;

struct Block
{
    void*           Memory    = nullptr; // Pointer to the allocated pages
    BlockAllocator  Allocator{};
};

void* AllocateAndCommitBlockIfNeeded(Block& block)
{
    auto memory = std::atomic_ref{ block.Memory }.load(std::memory_order_relaxed);
    if (memory != nullptr)
    {
        return memory;
    }

    // Allocate memory for the block if not already allocated
    auto const newMemory = Mmu::AllocateAndCommitPages(PagesPerBlock);
    if (!newMemory)
    {
        return nullptr; // Allocation failed
    }
    void* expected = nullptr;
    if (std::atomic_ref{ block.Memory }.compare_exchange_strong(memory, newMemory, std::memory_order_release))
    {
        return newMemory;
    }

    // Another thread allocated the memory first
    // TODO: Implement this or else we're leaking:
    // Mmu::FreePages(newMemory, PagesPerBlock);
    return memory;
}

void* AllocateInBlock(Block& block, size_t sizeInUnits, size_t alignmentInUnits)
{
    auto memory = AllocateAndCommitBlockIfNeeded(block);
    if (memory == nullptr)
    {
        return nullptr; // Allocation failed
    }

    // Allocate from the pool allocator
    auto const allocationIndex = block.Allocator.Allocate(sizeInUnits, alignmentInUnits);
    if (allocationIndex == UINT32_MAX)
    {
        return nullptr; // No space available
    }

    // Calculate the actual memory address
    return static_cast<std::byte*>(block.Memory) + (allocationIndex * AllocationGranularity);
}

struct Heap
{
    Block                     Blocks[MaxBlocks];
    std::atomic<uint32_t>     BlockCount;
    std::atomic<uint32_t>     AllocationHint;  // Hint for next allocation attempt
};

Heap* CreateHeap()
{
    // Allocate memory for the heap structure itself
    void* heapMemory = Mmu::AllocateAndCommitPages((sizeof(Heap) + PageSize - 1) / PageSize);
    if (!heapMemory)
    {
        return nullptr;
    }
    
    Heap* heap = new(heapMemory) Heap{};
    heap->BlockCount    .store(0, std::memory_order_relaxed);
    heap->AllocationHint.store(0, std::memory_order_relaxed);
    
    return heap;
}

void DestroyHeap(Heap* heap)
{
    if (!heap)
    {
        return;
    }
    
    // TODO: Free all allocated blocks
    // For now, we'll leave them allocated as we don't have page deallocation
    
    heap->~Heap();
    // TODO: Free the heap structure page when we have page deallocation
}

void* Allocate(Heap* heap, size_t size)
{
    return AllocateAligned(heap, size, AllocationGranularity);
}

void* AllocateAligned(Heap* heap, size_t size, size_t alignment)
{
    if (!heap || size == 0 || alignment == 0)
    {
        Cpu::Panic("Heap::AllocateAligned: invalid parameters");
    }
    
    // Calculate number of allocation units needed
    uint32_t const unitsNeeded = (size + AllocationGranularity - 1) / AllocationGranularity;
    uint32_t const alignmentUnits = (alignment + AllocationGranularity - 1) / AllocationGranularity;
    
    if (unitsNeeded > AllocationsPerBlock)
    {
        // Allocation too large for a single block
        Cpu::Panic("Heap::AllocateAligned: size %zu exceeds maximum allocation size of %zu", size, AllocationsPerBlock * AllocationGranularity);
    }
    
    // Try to allocate starting from the hint
    uint32_t blockCount = heap->BlockCount    .load(std::memory_order_relaxed);
    uint32_t blockIndex = heap->AllocationHint.load(std::memory_order_relaxed) % blockCount;
    
    while (blockIndex < MaxBlocks)
    {
        for (uint32_t attempt = 0; attempt < heap->BlockCount; ++attempt)
        {
            Block& block = heap->Blocks[blockIndex];
            
            ++blockIndex;
            if (blockIndex >= blockCount)
            {
                blockIndex = 0; // Wrap around
            }
        
            // Try to allocate from this block
            auto const result = AllocateInBlock(block, unitsNeeded, alignmentUnits);
            if (result != nullptr)
            {
                // Update hint for next allocation
                heap->AllocationHint.store(blockIndex, std::memory_order_relaxed);
                
                // Calculate the actual memory address
                return result;
            }
        }

        if (blockCount >= MaxBlocks)
        {
            Cpu::Panic("Heap::AllocateAligned: no space available");
        }
        // Note: We don't care who adds to the block count. It's all good.
        blockIndex = blockCount;
        if (blockCount < MaxBlocks)
        {
            (void)heap->BlockCount.compare_exchange_strong(blockCount, blockCount + 1, std::memory_order_relaxed);
        }
    }
    
    Cpu::Panic("Heap::AllocateAligned: no space available after trying all blocks");
}

void Deallocate(Heap* heap, void* ptr)
{
    if (!heap || !ptr)
    {
        return;
    }
    
    // Find which block contains this pointer
    uint32_t blockCount = heap->BlockCount.load(std::memory_order_relaxed);
    for (uint32_t blockIndex = 0; blockIndex < blockCount; ++blockIndex)
    {
        Block& block = heap->Blocks[blockIndex];
        
        if (block.Memory == nullptr)
        {
            continue;
        }
        
        uintptr_t const blockStart = reinterpret_cast<uintptr_t>(block.Memory);
        uintptr_t const blockEnd   = blockStart + BlockSize;
        uintptr_t const ptrAddr    = reinterpret_cast<uintptr_t>(ptr);
        
        if (blockStart <= ptrAddr && ptrAddr < blockEnd)
        {
            // Calculate allocation index within this block
            uint32_t const allocationIndex = (ptrAddr - blockStart) / AllocationGranularity;
            
            // Deallocate from the pool allocator
            block.Allocator.Deallocate(allocationIndex);
            return;
        }
    }
    
    // Pointer not found in any block - this is an error
    Cpu::Panic("Heap::Deallocate: invalid pointer %p", ptr);
}

size_t GetTotalSize(Heap* heap)
{
    if (!heap)
    {
        return 0;
    }
    
    return heap->BlockCount * BlockSize;
}

size_t GetFreeSize(Heap* heap)
{
    if (!heap)
    {
        return 0;
    }
    
    size_t freeSize = 0;
    
    for (uint32_t blockIndex = 0; blockIndex < heap->BlockCount; ++blockIndex)
    {
        Block& block = heap->Blocks[blockIndex];
        
        if (block.Memory != nullptr)
        {
            // Count free allocations in this block
            // TODO: Add a method to PoolAllocator to count free allocations
            // For now, assume each committed block contributes its full size
            freeSize += BlockSize;
        }
    }
    
    return freeSize;
}

size_t GetUsedSize(Heap* heap)
{
    return GetTotalSize(heap) - GetFreeSize(heap);
}

size_t GetLargestFreeBlock(Heap* heap)
{
    if (!heap)
    {
        return 0;
    }
    
    // TODO: Implement by checking each block's allocator for largest contiguous free space
    // For now, return the allocation granularity as a safe estimate
    return AllocationGranularity;
}

bool ValidateHeap(Heap* heap)
{
    if (!heap)
    {
        return false;
    }
    
    // Basic validation
    if (heap->BlockCount > MaxBlocks)
    {
        return false;
    }
    
    for (uint32_t blockIndex = 0; blockIndex < heap->BlockCount; ++blockIndex)
    {
        Block& block = heap->Blocks[blockIndex];
        
        if (block.Memory != nullptr)
        {
            // TODO: Validate the block's allocator
            //if (!ValidatePoolAllocator(&block.Allocator))
            //{
            //    return false;
            //}
        }
    }
    
    return true;
}

void DumpHeapState(Heap* heap)
{
    if (!heap)
    {
        printf("Heap: null\n");
        return;
    }
    
    printf("Heap at %p:\n", heap);
    printf("  Block count: %u\n", heap->BlockCount.load(std::memory_order_relaxed));
    printf("  Total size: %zu bytes\n", GetTotalSize(heap));
    printf("  Used size: %zu bytes\n", GetUsedSize(heap));
    printf("  Free size: %zu bytes\n", GetFreeSize(heap));
    
    for (uint32_t blockIndex = 0; blockIndex < heap->BlockCount; ++blockIndex)
    {
        Block& block = heap->Blocks[blockIndex];
        
        if (block.Memory)
        {
            printf("  Block %u: %p\n", 
                blockIndex, 
                block.Memory
            );
        }
    }
}

}
// namespace Heap