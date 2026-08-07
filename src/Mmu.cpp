#include "Mmu.h"
#include "Cpu.h"
#include "Mmio.h"
#include "Uart.h"
#include "Containers.h"

#include "emb-stdio.h"

#include <stdint.h>
#include <stddef.h>

extern uintptr_t GpuMemBase;

namespace Mmu
{

struct alignas(8) InvalidDescriptor
{
    uint64_t valid     :  1; // [0] 0b0 = invalid
    uint64_t reserved0 : 63; // [63:1]

    constexpr InvalidDescriptor()
        : valid(0), reserved0(0) {}
};

struct alignas(8) TableDescriptor
{
    uint64_t valid       :  1; // [0] 0b1 = valid
    uint64_t type        :  1; // [1] 0b1 = table
    uint64_t reserved0   : 10; // [11:2]
    uint64_t output_addr : 36; // [47:12] Output address
    uint64_t reserved1   :  3; // [50:48]
    uint64_t ignored1    :  8; // [58:51]
    uint64_t pxnt        :  1; // [59]
    uint64_t xnt         :  1; // [60]
    uint64_t apt         :  2; // [62:61]
    uint64_t nst         :  1; // [63]

    constexpr TableDescriptor(uint64_t addr = 0)
        : valid(1), type(1), reserved0(0), output_addr(addr >> 12), reserved1(0),
          ignored1(0), pxnt(0), xnt(0), apt(0), nst(0) {}
};

struct alignas(8) L1BlockDescriptor
{
    uint64_t valid       :  1; // [0] 0b1 = valid
    uint64_t type        :  1; // [1] 0b0 = block
    uint64_t attr_index  :  3; // [4:2] AttrIndx[2:0]
    uint64_t ns          :  1; // [5]   NS (non-secure)
    uint64_t ap          :  2; // [7:6] AP[1:0] (access permissions)
    uint64_t sh          :  2; // [9:8] SH[1:0] (shareability)
    uint64_t af          :  1; // [10]  AF (access flag)
    uint64_t ng          :  1; // [11]  nG (not global)
    uint64_t reserved0   : 18; // [29:12] Reserved
    uint64_t output_addr : 18; // [47:30] Output address (block base >> 30)
    uint64_t reserved1   :  4; // [51:48] Reserved
    uint64_t contiguous  :  1; // [52]   Contiguous
    uint64_t pxn         :  1; // [53]   PXN
    uint64_t uxn         :  1; // [54]   UXN
    uint64_t reserved2   :  9; // [63:55] Reserved

    constexpr L1BlockDescriptor(uint64_t addr = 0)
        : valid(1), type(0), attr_index(0), ns(0), ap(0), sh(3), af(1), ng(0),
          reserved0(0), output_addr(addr >> 30), reserved1(0),
          contiguous(0), pxn(0), uxn(0), reserved2(0) {}
};

struct alignas(8) L2BlockDescriptor
{
    uint64_t valid       :  1; // [0] 0b1 = valid
    uint64_t type        :  1; // [1] 0b0 = block
    uint64_t attr_index  :  3; // [4:2] AttrIndx[2:0]
    uint64_t ns          :  1; // [5]   NS (non-secure)
    uint64_t ap          :  2; // [7:6] AP[1:0] (access permissions)
    uint64_t sh          :  2; // [9:8] SH[1:0] (shareability)
    uint64_t af          :  1; // [10]  AF (access flag)
    uint64_t ng          :  1; // [11]  nG (not global)
    uint64_t reserved0   :  9; // [20:12] Reserved
    uint64_t output_addr : 27; // [47:21] Output address (block base >> 21)
    uint64_t reserved1   :  4; // [51:48] Reserved
    uint64_t contiguous  :  1; // [52]   Contiguous
    uint64_t pxn         :  1; // [53]   PXN
    uint64_t uxn         :  1; // [54]   UXN
    uint64_t reserved2   :  9; // [63:55] Reserved

    constexpr L2BlockDescriptor(uint64_t addr = 0)
        : valid(1), type(0), attr_index(0), ns(0), ap(0), sh(3), af(1), ng(0),
          reserved0(0), output_addr(addr >> 21), reserved1(0),
          contiguous(0), pxn(0), uxn(0), reserved2(0) {}
};

struct alignas(8) L3PageDescriptor
{
    uint64_t valid       :  1; // [0] 0b1 = valid
    uint64_t type        :  1; // [1] 0b1 = page
    uint64_t attr_index  :  3; // [4:2] AttrIndx[2:0]
    uint64_t ns          :  1; // [5]   NS (non-secure)
    uint64_t ap          :  2; // [7:6] AP[1:0] (access permissions)
    uint64_t sh          :  2; // [9:8] SH[1:0] (shareability)
    uint64_t af          :  1; // [10]  AF (access flag)
    uint64_t ng          :  1; // [11]  nG (not global)
    uint64_t output_addr : 36; // [47:12] Output address (page base >> 12)
    uint64_t reserved0   :  4; // [51:48] Reserved
    uint64_t contiguous  :  1; // [52]   Contiguous
    uint64_t pxn         :  1; // [53]   PXN
    uint64_t uxn         :  1; // [54]   UXN
    uint64_t reserved2   :  9; // [63:55] Reserved

    constexpr L3PageDescriptor(uint64_t addr = 0)
        : valid(1), type(1), attr_index(0), ns(0), ap(0), sh(3), af(1), ng(0),
          output_addr(addr >> 12), reserved0(0),
          contiguous(0), pxn(0), uxn(0), reserved2(0) {}
};

static_assert(sizeof(InvalidDescriptor) == 8, "InvalidDescriptor must be 8 bytes");
static_assert(sizeof(TableDescriptor) == 8, "TableDescriptor must be 8 bytes");
static_assert(sizeof(L1BlockDescriptor) == 8, "L1BlockDescriptor must be 8 bytes");
static_assert(sizeof(L2BlockDescriptor) == 8, "L2BlockDescriptor must be 8 bytes");

union L1Entry
{
    InvalidDescriptor invalid;
    TableDescriptor   table;
    L1BlockDescriptor block;

    constexpr L1Entry()                    : invalid()  {}
    constexpr L1Entry(InvalidDescriptor i) : invalid(i) {}
    constexpr L1Entry(TableDescriptor   t) : table  (t) {}
    constexpr L1Entry(L1BlockDescriptor b) : block  (b) {}

    constexpr bool IsValid() const { return invalid.valid != 0; }
    constexpr bool IsTable() const { return table.valid != 0 && table.type == 1; }
    constexpr bool IsBlock() const { return block.valid != 0 && block.type == 0; }
};

union L2Entry
{
    InvalidDescriptor invalid;
    TableDescriptor   table;
    L2BlockDescriptor block;

    constexpr L2Entry()                    : invalid()  {}
    constexpr L2Entry(InvalidDescriptor i) : invalid(i) {}
    constexpr L2Entry(TableDescriptor   t) : table  (t) {}
    constexpr L2Entry(L2BlockDescriptor b) : block  (b) {}

    constexpr bool IsValid() const { return invalid.valid != 0; }
    constexpr bool IsTable() const { return table.valid != 0 && table.type == 1; }
    constexpr bool IsBlock() const { return block.valid != 0 && block.type == 0; }
};

union L3Entry
{
    InvalidDescriptor invalid;
    L3PageDescriptor  page;

    constexpr L3Entry()                    : invalid()  {}
    constexpr L3Entry(InvalidDescriptor i) : invalid(i) {}
    constexpr L3Entry(L3PageDescriptor  p) : page   (p) {}

    constexpr bool IsValid() const { return invalid.valid != 0; }
    constexpr bool IsPage () const { return page.valid != 0 && page.type == 1; }
};

// Normal memory (cacheable) for RAM
constexpr L1BlockDescriptor L1NormalMem(uint64_t addr)
{
    return L1BlockDescriptor(addr);
}

// GPU memory (transient, write-through)
constexpr L1BlockDescriptor L1GpuMem(uint64_t addr)
{
    L1BlockDescriptor desc(addr);
    desc.attr_index = 2; // AttrIndx[2:0]=2 (transient memory, see MAIR)
    desc.sh = 2;         // Outer-shareable
    desc.pxn = 1;        // Privileged execute-never
    desc.uxn = 1;        // Unprivileged execute-never
    return desc;
}

// Device memory (non-cacheable, strongly ordered)
constexpr L1BlockDescriptor L1DeviceMem(uint64_t addr)
{
    L1BlockDescriptor desc(addr);
    desc.attr_index = 1; // AttrIndx[2:0]=1 (device memory, see MAIR)
    desc.sh = 2;         // Outer-shareable
    desc.pxn = 1;        // Privileged execute-never
    desc.uxn = 1;        // Unprivileged execute-never
    return desc;
}

// Normal memory (cacheable) for RAM
constexpr L2BlockDescriptor L2NormalMem(uint64_t addr)
{
    return L2BlockDescriptor(addr);
}

// Device memory (non-cacheable, strongly ordered)
constexpr L2BlockDescriptor L2DeviceMem(uint64_t addr)
{
    L2BlockDescriptor desc(addr);
    desc.attr_index = 1; // AttrIndx[2:0]=1 (device memory, see MAIR)
    desc.sh = 2;         // Outer-shareable
    desc.pxn = 1;        // Privileged execute-never
    desc.uxn = 1;        // Unprivileged execute-never
    return desc;
}

// Normal memory (cacheable) for RAM
constexpr L3PageDescriptor L3NormalMem(uint64_t addr)
{
    return L3PageDescriptor(addr);
}

// MAIR: Attr0=0xFF (normal), Attr1=0x04 (device-nGnRE)
constexpr uint64_t MAIR_ATTR = 0xFF | (0x04 << 8) | (0x33 << 16);

/*
TCR_EL1 (Translation Control Register, EL1) format (AArch64):

Bits  | Name      | Description
------|-----------|---------------------------------------------------------------
  5:0    T0SZ     | Size offset for TTBR0_EL1 region (VA size = 64 - T0SZ bits)
  6      RES0     | Reserved
  7      EPD0     | Disable translation table walks using TTBR0_EL1 (0=enable)
  9:8    IRGN0    | Inner cacheability for TTBR0_EL1 (0b00=NC, 0b01=WBWA, 0b10=WT, 0b11=WB)
 11:10   ORGN0    | Outer cacheability for TTBR0_EL1 (same encoding as IRGN0)
 13:12   SH0      | Shareability for TTBR0_EL1 (0b00=Non-shareable, 0b10=Inner, 0b11=Outer)
 15:14   TG0      | Granule size for TTBR0_EL1 (0b00=4KB, 0b01=64KB, 0b10=16KB)
 21:16   T1SZ     | Size offset for TTBR1_EL1 region (VA size = 64 - T1SZ bits)
 22      RES0     | Reserved
 23      EPD1     | Disable translation table walks using TTBR1_EL1 (0=enable)
 25:24   IRGN1    | Inner cacheability for TTBR1_EL1 (0b00=NC, 0b01=WBWA, 0b10=WT, 0b11=WB)
 27:26   ORGN1    | Outer cacheability for TTBR1_EL1 (same encoding as IRGN0)
 29:28   SH1      | Shareability for TTBR1_EL1 (0b00=Non-shareable, 0b10=Inner, 0b11=Outer)
 31:30   TG1      | Granule size for TTBR1_EL1 (0b00=4KB, 0b01=64KB, 0b10=16KB)
 ...     ...      | (See ARM ARM DDI0487 for further fields and details)

Common encodings:
- IRGNx/ORGNx: 0b00 = Non-cacheable, 0b01 = Write-Back Write-Allocate, 0b10 = Write-Through, 0b11 = Write-Back no Write-Allocate
- SHx: 0b00 = Non-shareable, 0b10 = Inner Shareable, 0b11 = Outer Shareable
- TGx: For TG0: 0b00=4KB, 0b01=64KB, 0b10=16KB; For TG1: 0b11=4KB, 0b01=64KB, 0b10=16KB
- IPS: 0b000=32b, 0b001=36b, 0b010=40b, 0b011=42b, 0b100=44b, 0b101=48b physical address

See ARM ARM DDI0487 for full details.

ARM64 (AArch64) Cacheability Settings (used in IRGNx/ORGNx fields of TCR_EL1):

Encoding | Name/Policy                        | Description
---------|------------------------------------|---------------------------------------------------------------
  0b00   | Non-cacheable                      | Memory accesses bypass all caches. No allocation or caching at this level.
  0b01   | Write-Back, Write-Allocate (WBWA)  | Writes are cached and only written to memory on eviction (write-back).
                                              | On a write miss, a cache line is allocated (write-allocate).
                                              | This is the most common and efficient setting for normal RAM.
  0b10   | Write-Through, no Write-Allocate   | Writes update both the cache and main memory simultaneously (write-through).
                                              | On a write miss, no cache line is allocated (no write-allocate).
                                              | Useful for memory regions where coherence with external agents is needed.
  0b11   | Write-Back, no Write-Allocate      | Writes are cached and written to memory on eviction (write-back).
                                              | On a write miss, no cache line is allocated (no write-allocate).
                                              | Useful for special cases where you want caching but not allocation on write.

- These encodings are used in the IRGNx (Inner) and ORGNx (Outer) fields of TCR_EL1 to control cacheability for inner (L1) and outer (L2/L3/system) caches.
- "Write-Back, Write-Allocate" (0b01) is the default for normal memory.
- Device or strongly-ordered memory should use "Non-cacheable" (0b00).
- The choice affects memory performance, coherency, and visibility to DMA or other agents.

References:
- ARM ARM DDI0487: TCR_EL1, memory attributes, and cacheability policies.

*/

// L1 page table: map RAM, video, and peripherals
// Each entry maps 1G of virtual memory, for a total of 512G per table.
struct alignas(0x1000) L1PageTable
{
    L1Entry entries[512]{}; // 512 entries, each 8 bytes
};

// L2 page table: map RAM, video, and peripherals
// Each entry maps 2M of virtual memory, for a total of 1G per table.
struct alignas(0x1000) L2PageTable
{
    L2Entry entries[512]{}; // 512 entries, each 8 bytes
};

// L3 page table: map RAM, video, and peripherals
// Each entry maps 4K of virtual memory, for a total of 2M per table.
struct alignas(0x1000) L3PageTable
{
    L3Entry entries[512]{}; // 512 entries, each 8 bytes
};

alignas(0x1000) static constinit L2PageTable l2_page_table = []() constexpr
{
    L2PageTable table = {};
    // Map 1 GB of RAM (0x00000000 - 0x3FFFFFFF) as normal memory
    for (size_t i = 0; i < 511; ++i)
    {
        table.entries[i] = L2NormalMem(i * 0x20'0000ULL); // Each entry maps 2 MB
    }
    table.entries[511] = L2DeviceMem(0x3F00'0000ULL);
    return table;
}();

alignas(0x1000) static constinit L1PageTable Rpi3_l1_page_table
{{
    L1NormalMem(0),             // 0x0000'0000 - 0x3FFF'FFFF: 1 GB RAM (normal memory)
    L1DeviceMem(0),             // 0x4000'0000 - 0x7FFF'FFFF: 1 GB RAM, including the MMIO (device)
    L1DeviceMem(0x4000'0000),   // 0x8000'0000 - 0xBFFF'FFFF: (unused)
    L1GpuMem(0),                // 0xC000'0000 - 0xFFFF'FFFF: 1 GB RAM (transient, WT memory for GPU (and devices) data)
    {},                             // 0x1'0000'0000 - 0x1'3FFF'FFFF: (unused)
    {},                             // 0x1'4000'0000 - 0x1'7FFF'FFFF: (unused)
    {},                             // 0x1'8000'0000 - 0x1'BFFF'FFFF: (unused)
    {},                             // 0x1'C000'0000 - 0x1'FFFF'FFFF: (unused)
    {},                             // 0x2'0000'0000 - 0x2'3FFF'FFFF: (unused)
    {},                             // 0x2'4000'0000 - 0x2'7FFF'FFFF: (unused)
    {},                             // 0x2'8000'0000 - 0x2'BFFF'FFFF: (unused)
    {},                             // 0x2'C000'0000 - 0x2'FFFF'FFFF: (unused)
    {},                             // 0x3'0000'0000 - 0x3'3FFF'FFFF: (unused)
    {},                             // 0x3'4000'0000 - 0x3'7FFF'FFFF: (unused)
    {},                             // 0x3'8000'0000 - 0x3'BFFF'FFFF: (unused)
    {},                             // 0x3'C000'0000 - 0x3'FFFF'FFFF: (unused)
    {},                             // 0x4'0000'0000 - 0x4'3FFF'FFFF: (unused)
    {},                             // 0x4'4000'0000 - 0x4'7FFF'FFFF: (unused)
    {},                             // 0x4'8000'0000 - 0x4'BFFF'FFFF: (unused)
    {},                             // 0x4'C000'0000 - 0x4'FFFF'FFFF: (unused)
    {},                             // 0x5'0000'0000 - 0x5'3FFF'FFFF: (unused)
    {},                             // 0x5'4000'0000 - 0x5'7FFF'FFFF: (unused)
    {},                             // 0x5'8000'0000 - 0x5'BFFF'FFFF: (unused)
    {},                             // 0x5'C000'0000 - 0x5'FFFF'FFFF: (unused)
    {},                             // 0x6'0000'0000 - 0x6'3FFF'FFFF: (unused)
    {},                             // 0x6'4000'0000 - 0x6'7FFF'FFFF: (unused)
    {},                             // 0x6'8000'0000 - 0x6'BFFF'FFFF: (unused)
    {},                             // 0x6'C000'0000 - 0x6'FFFF'FFFF: (unused)
    L1GpuMem(0x0'0000'0000ull),     // 0x7'0000'0000 - 0x7'3FFF'FFFF: // Map all of physical RAM, up to 8 GB, as GPU memory (fast but aggressive on writes)
    L1GpuMem(0x0'4000'0000ull),     // 0x7'4000'0000 - 0x7'7FFF'FFFF: 
    L1GpuMem(0x0'8000'0000ull),     // 0x7'8000'0000 - 0x7'BFFF'FFFF: 
    L1GpuMem(0x0'C000'0000ull),     // 0x7'C000'0000 - 0x7'FFFF'FFFF: 
    L1GpuMem(0x1'0000'0000ull),     // 0x8'0000'0000 - 0x8'3FFF'FFFF: 
    L1GpuMem(0x1'4000'0000ull),     // 0x8'4000'0000 - 0x8'7FFF'FFFF: 
    L1GpuMem(0x1'8000'0000ull),     // 0x8'8000'0000 - 0x8'BFFF'FFFF: 
    L1GpuMem(0x1'C000'0000ull),     // 0x8'C000'0000 - 0x8'FFFF'FFFF: 
    // Remaining entries are invalid
}};

alignas(0x1000) static constinit L1PageTable Rpi4_l1_page_table
{{
    L1NormalMem(0),                 //   0x0000'0000 -   0x3FFF'FFFF: 1 GB RAM (normal memory)
    L1DeviceMem(0x0'4000'0000ull),  //   0x4000'0000 -   0x7FFF'FFFF: 1 GB of MMIO (device)
    L1DeviceMem(0x0'C000'0000ull),  //   0x8000'0000 -   0xBFFF'FFFF: MMIO
    L1GpuMem(0),                    //   0xC000'0000 -   0xFFFF'FFFF: 1 GB RAM (transient, WT memory for GPU (and devices) data)
    {},                             // 0x1'0000'0000 - 0x1'3FFF'FFFF: (unused)
    {},                             // 0x1'4000'0000 - 0x1'7FFF'FFFF: (unused)
    {},                             // 0x1'8000'0000 - 0x1'BFFF'FFFF: (unused)
    {},                             // 0x1'C000'0000 - 0x1'FFFF'FFFF: (unused)
    {},                             // 0x2'0000'0000 - 0x2'3FFF'FFFF: (unused)
    {},                             // 0x2'4000'0000 - 0x2'7FFF'FFFF: (unused)
    {},                             // 0x2'8000'0000 - 0x2'BFFF'FFFF: (unused)
    {},                             // 0x2'C000'0000 - 0x2'FFFF'FFFF: (unused)
    {},                             // 0x3'0000'0000 - 0x3'3FFF'FFFF: (unused)
    {},                             // 0x3'4000'0000 - 0x3'7FFF'FFFF: (unused)
    {},                             // 0x3'8000'0000 - 0x3'BFFF'FFFF: (unused)
    {},                             // 0x3'C000'0000 - 0x3'FFFF'FFFF: (unused)
    {},                             // 0x4'0000'0000 - 0x4'3FFF'FFFF: (unused)
    L1DeviceMem(0x4'4000'0000ull),  // 0x4'4000'0000 - 0x4'7FFF'FFFF: (unused)
    {},                             // 0x4'8000'0000 - 0x4'BFFF'FFFF: (unused)
    L1DeviceMem(0x4'C000'0000ull),  // 0x4'C000'0000 - 0x4'FFFF'FFFF: MMIO
    {},                             // 0x5'0000'0000 - 0x5'3FFF'FFFF: (unused)
    {},                             // 0x5'4000'0000 - 0x5'7FFF'FFFF: (unused)
    {},                             // 0x5'8000'0000 - 0x5'BFFF'FFFF: (unused)
    {},                             // 0x5'C000'0000 - 0x5'FFFF'FFFF: (unused)
    L1DeviceMem(0x6'0000'0000ull),  // 0x6'0000'0000 - 0x6'3FFF'FFFF: PCIe
    {},                             // 0x6'4000'0000 - 0x6'7FFF'FFFF: (unused)
    {},                             // 0x6'8000'0000 - 0x6'BFFF'FFFF: (unused)
    {},                             // 0x6'C000'0000 - 0x6'FFFF'FFFF: (unused)
    L1GpuMem(0x0'0000'0000ull),     // 0x7'0000'0000 - 0x7'3FFF'FFFF: // Map all of physical RAM, up to 8 GB, as GPU memory (fast but aggressive on writes)
    L1GpuMem(0x0'4000'0000ull),     // 0x7'4000'0000 - 0x7'7FFF'FFFF: 
    L1GpuMem(0x0'8000'0000ull),     // 0x7'8000'0000 - 0x7'BFFF'FFFF: 
    L1GpuMem(0x0'C000'0000ull),     // 0x7'C000'0000 - 0x7'FFFF'FFFF: 
    L1GpuMem(0x1'0000'0000ull),     // 0x8'0000'0000 - 0x8'3FFF'FFFF: 
    L1GpuMem(0x1'4000'0000ull),     // 0x8'4000'0000 - 0x8'7FFF'FFFF: 
    L1GpuMem(0x1'8000'0000ull),     // 0x8'8000'0000 - 0x8'BFFF'FFFF: 
    L1GpuMem(0x1'C000'0000ull),     // 0x8'C000'0000 - 0x8'FFFF'FFFF: 
    // Remaining entries are invalid
}};

constexpr uintptr_t PhysicalMemoryApertureBase = 0x7'0000'0000;

L1PageTable* l1_page_table = reinterpret_cast<L1PageTable*>(reinterpret_cast<uintptr_t>(Cpu::IsRpi4() ? &Rpi4_l1_page_table : &Rpi3_l1_page_table) + PhysicalMemoryApertureBase);

constexpr char PhysicalMemory[] = "Physical Memory";
constexpr char VirtualMemory [] = "Virtual Memory";

// Manages 256 MB of physical 4 KB memory pages.
Containers::PoolAllocator<256 * 1024 / 4, PhysicalMemory> PhysicalMemoryAllocator;
constexpr uintptr_t PhysicalMemoryAllocatorOffset = 0x2000'0000; // 256 MB of physical memory, starting at 0x200'0000
constexpr uintptr_t PhysicalMemoryAllocatorPageOffset = PhysicalMemoryAllocatorOffset >> 12;

// Manages 1 GB of virtual 4 KB memory pages.
Containers::PoolAllocator<1024 * 1024 / 4, VirtualMemory> VirtualMemoryAllocator;
constexpr uintptr_t VirtualMemoryAllocatorOffset = 0x1'0000'0000; // 1 GB of virtual memory, starting at 0x1'0000'0000
constexpr uintptr_t VirtualMemoryAllocatorPageOffset = VirtualMemoryAllocatorOffset >> 12;

void MapOnePage(L3PageTable& table, uintptr_t virtualPageIndex, L3Entry entry)
{
//    printf("L3 table %zX entry index is %zX\n", &table, virtualPageIndex);
    auto& tableEntry = table.entries[virtualPageIndex];
    tableEntry = entry;
    // Clean the cache line containing the modified page table entry
    asm volatile ("dc civac, %0" : : "r"(&tableEntry) : "memory");
//    printf("L3 table entry at %zX: %llX\n", &tableEntry, reinterpret_cast<uint64_t&>(tableEntry));
}

void MapOnePage(L2PageTable& table, uintptr_t virtualPageIndex, L3Entry entry)
{
//    printf("L2 table %zX entry index is %zX\n", &table, virtualPageIndex >> 9);
    auto& tableEntry = table.entries[virtualPageIndex >> 9];
    L3PageTable* nextTable = nullptr;
    if (!tableEntry.IsValid())
    {
        auto const tablePhysicalAddress = (PhysicalMemoryAllocator.Allocate(1) + PhysicalMemoryAllocatorPageOffset) << 12;
        printf("Allocated physical address for L3 page table at %zX\n", tablePhysicalAddress);
        tableEntry = TableDescriptor(tablePhysicalAddress);
        // Clean the cache line containing the modified page table entry
        asm volatile ("dc civac, %0" : : "r"(&tableEntry) : "memory");
        nextTable = new(reinterpret_cast<void*>(PhysicalMemoryApertureBase + tablePhysicalAddress)) L3PageTable;
    }
    else if (tableEntry.IsTable())
    {
        nextTable = reinterpret_cast<L3PageTable*>(PhysicalMemoryApertureBase + (tableEntry.table.output_addr << 12));
    }
//    printf("L2 table entry at %zX: %llX\n", &tableEntry, reinterpret_cast<uint64_t&>(tableEntry));
    MapOnePage(*nextTable, virtualPageIndex & ~(UINTPTR_MAX << 9), entry);
}

void MapOnePage(L1PageTable& table, uintptr_t virtualPageIndex, L3Entry entry)
{
//    printf("L1 table %zX entry index is %zX\n", &table, virtualPageIndex >> 18);
    auto& tableEntry = table.entries[virtualPageIndex >> 18];
    L2PageTable* nextTable = nullptr;
    if (!tableEntry.IsValid())
    {
        auto const tablePhysicalAddress = (PhysicalMemoryAllocator.Allocate(1) + PhysicalMemoryAllocatorPageOffset) << 12;
        printf("Allocated physical address for L2 page table at %zX\n", tablePhysicalAddress);
        tableEntry = TableDescriptor(tablePhysicalAddress);
        // Clean the cache line containing the modified page table entry
        asm volatile ("dc civac, %0" : : "r"(&tableEntry) : "memory");
        nextTable = new(reinterpret_cast<void*>(PhysicalMemoryApertureBase + tablePhysicalAddress)) L2PageTable;
    }
    else if (tableEntry.IsTable())
    {
        nextTable = reinterpret_cast<L2PageTable*>(PhysicalMemoryApertureBase + (tableEntry.table.output_addr << 12));
    }
//    printf("L1 table entry at %zX: %llX\n", &tableEntry, reinterpret_cast<uint64_t&>(tableEntry));
    MapOnePage(*nextTable, virtualPageIndex & ~(UINTPTR_MAX << 18), entry);
}

void MapOnePage(uintptr_t virtualPageIndex, L3Entry entry)
{
//    printf("Mapping virtual page %zX to physical page %zX\n", virtualPageIndex, entry.page.output_addr << 12);
    MapOnePage(*l1_page_table, virtualPageIndex, entry);
}

void CommitPages(uint32_t basePage, uint32_t pageCount)
{
    for (uint32_t i = 0; i < pageCount; ++i)
    {
        // Get a physical page and map it to the virtual page
        auto const physicalPageAddress = (PhysicalMemoryAllocator.Allocate(1) + PhysicalMemoryAllocatorPageOffset) << 12;
        //printf("Mapping virtual page %zX to physical page %zX\n", basePage + i, physicalPageAddress);

        //printf("L1 table entry was %llX\n", reinterpret_cast<uint64_t&>(l1_page_table->entries[basePage >> 18]));
        //l1_page_table->entries[basePage >> 18] = L1NormalMem(physicalPageAddress);
        //printf("L1 table entry is %llX\n", reinterpret_cast<uint64_t&>(l1_page_table->entries[basePage >> 18]));

        MapOnePage(basePage + i, L3NormalMem(physicalPageAddress));
    }
    // Flush the TLB for the newly mapped virtual pages
    //for (uint32_t i = 0; i < pageCount; ++i)
    //{
    //    asm volatile ("tlbi vae1, %0" : : "r"((basePage + i) << 12) : "memory");
    //}
    Cpu::InstructionSynchronizationBarrier();
    Cpu::InnerDataSynchronizationBarrier();
}

void* AllocatePages(uint32_t pageCount)
{
    // Return the virtual address of the first page
    return reinterpret_cast<void*>(VirtualMemoryAllocator.Allocate(pageCount) * 0x1000 + VirtualMemoryAllocatorOffset);
}

void* AllocateAndCommitPages(uint32_t pageCount)
{
    auto const result = AllocatePages(pageCount);
    CommitPages(result, pageCount);
    return result;
}

void CommitPages(void const* address, uint32_t pageCount)
{
    auto const basePage = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address) >> 12);
    CommitPages(basePage, pageCount);
}

void* AllocateGpuMemory(uint32_t pageCount)
{
    auto const gpuPhysicalAddress = (PhysicalMemoryAllocator.Allocate(pageCount) + PhysicalMemoryAllocatorPageOffset) << 12;
    //printf("Allocated GPU memory at physical address %zX\n", gpuPhysicalAddress);
    return reinterpret_cast<void*>(gpuPhysicalAddress + GpuMemBase);
}

//uint64_t GetMairEl1()
//{
//    uint64_t mair;
//    asm volatile ("mrs %0, mair_el1" : "=r"(mair));
//    return mair;
//}
//
//uint64_t const MairEl1 = GetMairEl1();

static void DumpMMUState()
{
    Uart::Puts("MMU State:\n");
    Uart::Puts("  TTBR0_EL1: ");
    uint64_t ttbr0;
    asm volatile ("mrs %0, ttbr0_el1" : "=r"(ttbr0));
    Uart::PutHex(ttbr0);
    Uart::Puts("\n");

    Uart::Puts("  TCR_EL1: ");
    uint64_t tcr;
    asm volatile ("mrs %0, tcr_el1" : "=r"(tcr));
    Uart::PutHex(tcr);
    Uart::Puts("  ");
    Uart::PutBin(tcr);
    Uart::Puts("\n");

    Uart::Puts("  MAIR_EL1: ");
    uint64_t mair;
    asm volatile ("mrs %0, mair_el1" : "=r"(mair));
    Uart::PutHex(mair);
    Uart::Puts("\n");

    uint64_t sctlr;
    asm volatile ("mrs %0, sctlr_el1" : "=r"(sctlr));
    Uart::Puts("  SCTLR_EL1: ");
    Uart::PutHex(sctlr);
    Uart::Puts("  ");
    Uart::PutBin(sctlr);
    Uart::Puts("\n");
}

static void InitPageTables()
{
    // Set MAIR_EL1: Attr0 = 0xFF (normal memory, inner/outer write-back, write-allocate)
    asm volatile ("msr mair_el1, %0" : : "r"(MAIR_ATTR));

    // Set TCR_EL1: 4KB granule, 39-bit VA, 1GB region, inner/outer WB WA cacheable, shareable
    uint64_t tcr = (25ULL << 0) | // T0SZ = 64-25 = 39 (39-bit address space (512 GB) allows level 1 tables to be complete at 1 GB covered per entry)
                   (0ULL << 7) | // EPD0 = 0b0 (enable the bottom page tables)
                   (0b01ULL << 8) | // IRGN0 = 0b01 (inner WB WA)
                   (0b01ULL << 10) | // ORGN0 = 0b01 (outer WB WA)
                   (0b11ULL << 12) | // SH0 = 0b11 (inner-shareable)
                   (0b00ULL << 14) | // TG0 = 0b00 (4KB granule)
                   (0ULL << 16) |     // Reserved (more stuff)
                   (1ULL << (16+7)) | // EPD1 = 0b1 (disable the top page tables)
                   (1ULL << 32); // IPS = 64GB
    //uint64_t tcr = (28ULL << 0) | // T0SZ = 64-28 = 36 (36-bit address space)
    //               (0ULL << 7) | // EPD0 = 0b0 (enable the bottom page tables)
    //               (0b01ULL << 8) | // IRGN0 = 0b01 (inner WB WA)
    //               (0b01ULL << 10) | // ORGN0 = 0b01 (outer WB WA)
    //               (0b11ULL << 12) | // SH0 = 0b11 (inner-shareable)
    //               (0b00ULL << 14) | // TG0 = 0b00 (4KB granule)
    //               (0ULL << 16) |     // Reserved (more stuff)
    //               (1ULL << (16+7)) | // EPD1 = 0b1 (disable the top page tables)
    //               (1ULL << 32); // IPS = 64GB (36 bits) of physical address space
    asm volatile ("msr tcr_el1, %0" : : "r"(tcr));

    if (Cpu::IsRpi4())
    {
        if (Mmio::Rpi4Base == Mmio::Rpi4BaseLo)
        {
            Mmio::Base    = 0xBE00'0000u; // Update MMIO base to the new aperture.
            Mmio::QA7Base = 0xBF80'0000u; // Update ARM cores' MMIO base to the new aperture.
        }
        GpuMemBase    = 0xC000'0000u; // Update the GPU memory base to the new aperture.


        // Set TTBR0_EL1 to point to our L1 table
        asm volatile ("msr ttbr0_el1, %0" : : "r"((uint64_t)&Rpi4_l1_page_table + 1)); // +1 == CnP
    }
    else
    {
        Mmio::Base    = 0x7F00'0000u; // Update MMIO base to the new aperture.
        Mmio::QA7Base = 0x8000'0000u; // Update ARM cores' MMIO base to the new aperture.
        GpuMemBase    = 0xC000'0000u; // Update the GPU memory base to the new aperture.

        // Set TTBR0_EL1 to point to our L1 table
        asm volatile ("msr ttbr0_el1, %0" : : "r"((uint64_t)&Rpi3_l1_page_table + 1)); // +1 == CnP
    }

    // ISB to synchronize context
    asm volatile ("isb");
}

void EnableCachesAndMMU()
{
    uint64_t sctlr;
    asm volatile ("mrs %0, sctlr_el1" : "=r"(sctlr));
    // Set I (bit 12, instruction cache), C (bit 2, data cache), M (bit 0, MMU)
    sctlr |= (1 << 12) | (1 << 2) | (1 << 0);
    asm volatile ("msr sctlr_el1, %0" : : "r"(sctlr));
    // Flush TLBs (optional, but recommended)
    //asm volatile ("tlbi alle1"); This one can only be done at EL2 or EL3
    asm volatile ("dsb ish; isb" ::: "memory");
    // L2 cache is enabled automatically with L1 on Cortex-A53 (Pi 3B)
}

void Init()
{
    //DumpMMUState();

    // Initialize page tables and MMU
    InitPageTables();

    // Enable caches and MMU
    EnableCachesAndMMU();

    // Now the MMU is enabled and caches are active
    DumpMMUState();
}

}
// namespace Mmu
