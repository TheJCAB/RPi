#pragma once

#include <stdint.h>
#include <stddef.h>

namespace Mmu
{

void Init();

void EnableCachesAndMMU();

void* AllocatePages(uint32_t num_pages);

template < typename T >
inline T* AllocatePages(uint32_t num_pages)
{
    return static_cast<T*>(AllocatePages(num_pages));
}

}
// namespace Mmu
