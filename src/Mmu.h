#pragma once

#include <stdint.h>
#include <stddef.h>

namespace Mmu
{

constexpr uint32_t PageSize = 4096;

void InitPageTables();

void EnableCachesAndMMU();
void DumpMMUState();

[[nodiscard]] void* AllocatePages         (uint32_t pageCount);
[[nodiscard]] void* AllocateAndCommitPages(uint32_t pageCount);

void CommitPages(void const* address, uint32_t pageCount);

template < typename T >
inline T* AllocatePages(uint32_t pageCount)
{
    return static_cast<T*>(AllocatePages(pageCount));
}

template < typename T >
inline T* AllocateAndCommitPages(uint32_t pageCount)
{
    return static_cast<T*>(AllocateAndCommitPages(pageCount));
}

void* AllocateGpuMemory(uint32_t pageCount);

template < typename T >
inline T* AllocateGpuMemory(uint32_t pageCount)
{
    return static_cast<T*>(AllocateGpuMemory(pageCount));
}

}
// namespace Mmu
