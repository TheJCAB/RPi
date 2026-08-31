#include "Uart.h"

#include "emb-stdio.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

uint16_t FromBE(uint16_t x)
{
    return ((x & 0xFF00) >> 8) |
           ((x & 0x00FF) << 8);
}

uint32_t FromBE(uint32_t x)
{
    return ((x & 0xFF000000) >> 24) |
           ((x & 0x00FF0000) >> 8)  |
           ((x & 0x0000FF00) << 8)  |
           ((x & 0x000000FF) << 24);
}

uint64_t FromBE(uint64_t x)
{
    return (static_cast<uint64_t>(FromBE(static_cast<uint32_t>(x))) << 32)
         | FromBE(static_cast<uint32_t>(x >> 32));
}


template < std::unsigned_integral T >
struct BE
{
    T value;

    operator std::remove_cv_t<T>() const { return FromBE(value); }
};

// DTB constants
constexpr uint32_t FDT_MAGIC = 0xd00d'feed;
constexpr uint32_t FDT_BEGIN_NODE = 0x1;
constexpr uint32_t FDT_END_NODE = 0x2;
constexpr uint32_t FDT_PROP = 0x3;
constexpr uint32_t FDT_NOP = 0x4;
constexpr uint32_t FDT_END = 0x9;

struct fdt_header {
    BE<uint32_t> magic;
    BE<uint32_t> totalsize;
    BE<uint32_t> off_dt_struct;
    BE<uint32_t> off_dt_strings;
    BE<uint32_t> off_mem_rsvmap;
    BE<uint32_t> version;
    BE<uint32_t> last_comp_version;
    BE<uint32_t> boot_cpuid_phys;
    BE<uint32_t> size_dt_strings;
    BE<uint32_t> size_dt_struct;
};

void parse_dtb(void* dtb)
{
    if (dtb == nullptr)
    {
        Uart::Puts("No DeviceTree found\n");
        return;
    }
    auto* hdr = static_cast<fdt_header*>(dtb);
    if (hdr->magic != FDT_MAGIC) {
        Uart::Puts("Invalid DTB magic\n");
        return;
    }

    printf("DTB magic found\n");
    printf("DTB total size: %u bytes\n", static_cast<uint32_t>(hdr->totalsize));
    printf("DTB structure offset: %u bytes\n", static_cast<uint32_t>(hdr->off_dt_struct));
    printf("DTB strings offset: %u bytes\n", static_cast<uint32_t>(hdr->off_dt_strings));
    printf("DTB version: %u\n", static_cast<uint32_t>(hdr->version));
    printf("DTB last compatible version: %u\n", static_cast<uint32_t>(hdr->last_comp_version));
    printf("DTB boot CPU ID: %u\n", static_cast<uint32_t>(hdr->boot_cpuid_phys));
    printf("DTB size of strings: %u bytes\n", static_cast<uint32_t>(hdr->size_dt_strings));
    printf("DTB size of structure: %u bytes\n", static_cast<uint32_t>(hdr->size_dt_struct));
    printf("DTB memory reservation map offset: %u bytes\n", static_cast<uint32_t>(hdr->off_mem_rsvmap));

    if (hdr->off_mem_rsvmap != 0)
    {
        BE<uint64_t>* mem_rsvmap = reinterpret_cast<BE<uint64_t>*>(static_cast<char*>(dtb) + hdr->off_mem_rsvmap);
        while (mem_rsvmap[0] != 0 && mem_rsvmap[1] != 0) {
            printf("Memory reservation: base=0x%016llx, size=0x%016llx\n", static_cast<uint64_t>(mem_rsvmap[0]), static_cast<uint64_t>(mem_rsvmap[1]));
            mem_rsvmap += 2;
        }
    }

    const char* strings = static_cast<char*>(dtb) + hdr->off_dt_strings;
    BE<uint32_t> const* struct_block = reinterpret_cast<BE<uint32_t> const*>(
        static_cast<char*>(dtb) + hdr->off_dt_struct);

    bool in_root_node = false;
    bool in_memory_node = false;

    while (true) {
        uint32_t token = *struct_block++;
        if (token == FDT_BEGIN_NODE) {
            const char* name = reinterpret_cast<const char*>(struct_block);
            //printf("Begin node name: '%s'\n", name);
            size_t len = strlen(name);
            in_root_node = len == 0;
            in_memory_node = (strncmp(name, "memory", 6) == 0);
            struct_block += (len + 4) / 4;
        } else if (token == FDT_END_NODE) {
            in_root_node = false;
            in_memory_node = false;
        } else if (token == FDT_PROP) {
            uint32_t len = *struct_block++;
            uint32_t nameoff = *struct_block++;
            const char* prop_name = strings + nameoff;
            auto* value = struct_block;
            //printf(" Property: %s (%u bytes)\n", prop_name, len);

            if (in_root_node || in_memory_node) {
                if (strcmp(prop_name, "#address-cells") == 0) {
                    Uart::Puts("Address cells found: ");
                    Uart::PutDec(static_cast<uint32_t>(*value));
                    Uart::Puts("\n");
                } else if (strcmp(prop_name, "#size-cells") == 0) {
                    Uart::Puts("Size cells found: ");
                    Uart::PutDec(static_cast<uint32_t>(*value));
                    Uart::Puts("\n");
                } else if (strcmp(prop_name, "memreserve") == 0) {
                    Uart::Puts("Memory reservation found: ");
                    Uart::PutHex(static_cast<uint32_t>(value[0]));
                    Uart::Puts(" ");
                    Uart::PutHex(static_cast<uint32_t>(value[1]));
                    Uart::Puts("\n");
                }
            }

            if (in_memory_node && strcmp(prop_name, "reg") == 0) {
                Uart::Puts("Memory reg found. Length: ");
                Uart::PutDec(len);
                Uart::Puts("\n");
                while (len >= 12) {
                    uint64_t base = (static_cast<uint64_t>(value[0]) << 32) |
                                    value[1];
                    uint64_t size = value[2];

                    Uart::Puts("Memory base: ");
                    Uart::PutHex(base);
                    Uart::Puts("\nMemory size: ");
                    Uart::PutHex(size);
                    Uart::Puts("\n");

                    value += 3;
                    len -= 12;
                }
                return;
            }

            struct_block += (len + 3) / 4;
        } else if (token == FDT_END) {
            break;
        } else if (token == FDT_NOP) {
            continue;
        } else {
            Uart::Puts("Unknown token\n");
            break;
        }
    }
}
