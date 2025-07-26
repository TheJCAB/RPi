#pragma once

#include <stdint.h>
#include <stddef.h>

namespace Heap
{

// Forward declaration for opaque heap handle
struct Heap;

// Heap creation and destruction
Heap* CreateHeap();
void  DestroyHeap(Heap* heap);

// Memory allocation functions
void* Allocate(Heap* heap, size_t size);
void* AllocateAligned(Heap* heap, size_t size, size_t alignment);
void  Deallocate(Heap* heap, void* ptr);
void  DeallocateAligned(Heap* heap, void* ptr, size_t alignment);

// Heap information and debugging
size_t GetTotalSize(Heap* heap);
size_t GetFreeSize(Heap* heap);
size_t GetUsedSize(Heap* heap);
size_t GetLargestFreeBlock(Heap* heap);

// Heap validation and debugging
bool   ValidateHeap(Heap* heap);
void   DumpHeapState(Heap* heap);

}
// namespace Heap
