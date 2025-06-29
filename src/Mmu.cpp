#include "Mmu.h"

#include <stdint.h>
#include <stddef.h>

namespace Mmu
{

struct alignas(8) InvalidDescriptor
{
    uint64_t type      :  2; // [1:0] 0b00 = table
    uint64_t reserved0 : 62; // [63:2]

    constexpr InvalidDescriptor()
        : type(0), reserved0(0) {}
};

struct alignas(8) TableDescriptor
{
    uint64_t type        :  2; // [1:0] 0b11 = table
    uint64_t reserved0   : 10; // [11:2]
    uint64_t output_addr : 36; // [47:12] Output address
    uint64_t reserved1   :  3; // [50:48]
    uint64_t ignored1    :  8; // [58:51]
    uint64_t pxnt        :  1; // [59]
    uint64_t xnt         :  1; // [60]
    uint64_t apt         :  2; // [62:61]
    uint64_t nst         :  1; // [63]

    constexpr TableDescriptor(uint64_t addr = 0)
        : type(3), reserved0(0), output_addr(addr >> 30), reserved1(0),
          ignored1(0), pxnt(0), xnt(0), apt(0), nst(0) {}
};

struct alignas(8) L1BlockDescriptor
{
    uint64_t type        :  2; // [1:0] 0b01 = block
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
        : type(1), attr_index(0), ns(0), ap(0), sh(3), af(1), ng(0),
          reserved0(0), output_addr(addr >> 30), reserved1(0),
          contiguous(0), pxn(0), uxn(0), reserved2(0) {}
};

struct alignas(8) L2BlockDescriptor
{
    uint64_t type        :  2; // [1:0] 0b01 = block
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
        : type(1), attr_index(0), ns(0), ap(0), sh(3), af(1), ng(0),
          reserved0(0), output_addr(addr >> 21), reserved1(0),
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

    constexpr bool IsValid() const { return invalid.type != 0; }
    constexpr bool IsTable() const { return table.type == 3; }
    constexpr bool IsBlock() const { return block.type == 1; }
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

    constexpr bool IsValid() const { return invalid.type != 0; }
    constexpr bool IsTable() const { return table.type == 3; }
    constexpr bool IsBlock() const { return block.type == 1; }
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
struct alignas(0x1000) L1PageTable
{
    L1Entry entries[512]; // 512 entries, each 8 bytes
};

// L2 page table: map RAM, video, and peripherals
struct alignas(0x1000) L2PageTable
{
    L2Entry entries[512]; // 512 entries, each 8 bytes
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

alignas(0x1000) static constinit L1PageTable l1_page_table
{{
    L1NormalMem(0),        // 0x00000000 - 0x3FFFFFFF: 1 GB RAM (normal memory)
    L1DeviceMem(0),        // 0x40000000 - 0x7FFFFFFF: 1 GB RAM, including the MMIO (device)
    L1DeviceMem(0x40000000),   // 0x80000000 - 0xBFFFFFFF: (unused)
    L1GpuMem(0),           // 0xC0000000 - 0xFFFFFFFF: 1 GB RAM (transient, WT memory for GPU (and devices) data)
    // Remaining entries are invalid
}};

//uint64_t GetMairEl1()
//{
//    uint64_t mair;
//    asm volatile ("mrs %0, mair_el1" : "=r"(mair));
//    return mair;
//}
//
//uint64_t const MairEl1 = GetMairEl1();

static void InitPageTablesAndMMU()
{
    //l1_page_table.entries[0] = TableDescriptor((uint64_t)&l2_page_table);
    // Set MAIR_EL1: Attr0 = 0xFF (normal memory, inner/outer write-back, write-allocate)
    asm volatile ("msr mair_el1, %0" : : "r"(MAIR_ATTR));

    // Set TCR_EL1: 4KB granule, 48-bit VA, 1GB region, inner/outer WB WA cacheable, shareable
    uint64_t tcr = (25ULL << 0) | // T0SZ = 64-25 = 32 (39-bit address space)
                   (0ULL << 7) | // EPD0 = 0b0 (enable the bottom page tables)
                   (0b01ULL << 8) | // IRGN0 = 0b01 (inner WB WA)
                   (0b01ULL << 10) | // ORGN0 = 0b01 (outer WB WA)
                   (0b11ULL << 12) | // SH0 = 0b11 (inner-shareable)
                   (0b00ULL << 14) | // TG0 = 0b00 (4KB granule)
                   (0ULL << 16) |     // Reserved (more stuff)
                   (1ULL << (16+7)) | // EPD1 = 0b1 (disable the top page tables)
                   (2ULL << 32); // IPS = 1TB
    asm volatile ("msr tcr_el1, %0" : : "r"(tcr));

    // Set TTBR0_EL1 to point to our L1 table
    asm volatile ("msr ttbr0_el1, %0" : : "r"((uint64_t)&l1_page_table + 1)); // +1 == CnP

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
    // Initialize page tables and MMU
    InitPageTablesAndMMU();

    // Enable caches and MMU
    EnableCachesAndMMU();

    // Now the MMU is enabled and caches are active
}

}
// namespace Mmu
