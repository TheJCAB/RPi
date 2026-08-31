
#include <BootLib/DeviceTree.h>

#include <BootLib/Uart.h>

#include <concepts>

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace BootLib::DeviceTree
{

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
constexpr uint32_t FDT_MAGIC      = 0xd00d'feed;
constexpr uint32_t FDT_BEGIN_NODE = 0x1;
constexpr uint32_t FDT_END_NODE   = 0x2;
constexpr uint32_t FDT_PROP       = 0x3;
constexpr uint32_t FDT_NOP        = 0x4;
constexpr uint32_t FDT_END        = 0x9;

struct Header
{
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

void ParseDeviceTree(void* dtb, Uart::PL011Uart* log)
{
    if (dtb == nullptr)
    {
        Uart::Puts(log, "No DeviceTree found\n");
        return;
    }
    auto const hdr = static_cast<Header*>(dtb);
    if (hdr->magic != FDT_MAGIC)
    {
        Uart::Puts(log, "Invalid DTB magic\n");
        return;
    }

    Uart::Puts(log, "DTB magic found\n");
    Uart::Puts(log, "DTB total size:                    "); Uart::PutDec(log, static_cast<uint32_t>(hdr->totalsize        )); Uart::Puts(log, " bytes\n");
    Uart::Puts(log, "DTB structure offset:              "); Uart::PutDec(log, static_cast<uint32_t>(hdr->off_dt_struct    )); Uart::Puts(log, " bytes\n");
    Uart::Puts(log, "DTB strings offset:                "); Uart::PutDec(log, static_cast<uint32_t>(hdr->off_dt_strings   )); Uart::Puts(log, " bytes\n");
    Uart::Puts(log, "DTB version:                       "); Uart::PutDec(log, static_cast<uint32_t>(hdr->version          )); Uart::Puts(log, "\n");
    Uart::Puts(log, "DTB last compatible version:       "); Uart::PutDec(log, static_cast<uint32_t>(hdr->last_comp_version)); Uart::Puts(log, "\n");
    Uart::Puts(log, "DTB boot CPU ID:                   "); Uart::PutDec(log, static_cast<uint32_t>(hdr->boot_cpuid_phys  )); Uart::Puts(log, "\n");
    Uart::Puts(log, "DTB size of strings:               "); Uart::PutDec(log, static_cast<uint32_t>(hdr->size_dt_strings  )); Uart::Puts(log, " bytes\n");
    Uart::Puts(log, "DTB size of structure:             "); Uart::PutDec(log, static_cast<uint32_t>(hdr->size_dt_struct   )); Uart::Puts(log, " bytes\n");
    Uart::Puts(log, "DTB memory reservation map offset: "); Uart::PutDec(log, static_cast<uint32_t>(hdr->off_mem_rsvmap   )); Uart::Puts(log, " bytes\n");

    if (hdr->off_mem_rsvmap != 0)
    {
        // TODO: Do something with these.
        BE<uint64_t>* mem_rsvmap = reinterpret_cast<BE<uint64_t>*>(static_cast<char*>(dtb) + hdr->off_mem_rsvmap);
        while (mem_rsvmap[0] != 0 && mem_rsvmap[1] != 0) {
            Uart::Puts(log, "Memory reservation: base=");
            Uart::PutHex(log, static_cast<uint64_t>(mem_rsvmap[0]));
            Uart::Puts(log, ", size=");
            Uart::PutHex(log, static_cast<uint64_t>(mem_rsvmap[1]));
            Uart::Puts(log, "\n");
            mem_rsvmap += 2;
        }
    }

    auto const strings = static_cast<char const*>(dtb) + hdr->off_dt_strings;
    BE<uint32_t volatile> const* struct_block = reinterpret_cast<BE<uint32_t volatile> const*>(
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
                    Uart::Puts(log, "Address cells found: ");
                    Uart::PutDec(log, static_cast<uint32_t>(*value));
                    Uart::Puts(log, "\n");
                } else if (strcmp(prop_name, "#size-cells") == 0) {
                    Uart::Puts(log, "Size cells found: ");
                    Uart::PutDec(log, static_cast<uint32_t>(*value));
                    Uart::Puts(log, "\n");
                } else if (strcmp(prop_name, "memreserve") == 0) {
                    Uart::Puts(log, "Memory reservation found: ");
                    Uart::PutHex(log, static_cast<uint32_t>(value[0]));
                    Uart::Puts(log, " ");
                    Uart::PutHex(log, static_cast<uint32_t>(value[1]));
                    Uart::Puts(log, "\n");
                }
            }

            if (in_memory_node && strcmp(prop_name, "reg") == 0) {
                Uart::Puts(log, "Memory reg found. Length: ");
                Uart::PutDec(log, len);
                Uart::Puts(log, "\n");
                while (len >= 12) {
                    uint64_t base = (static_cast<uint64_t>(value[0]) << 32) |
                                    value[1];
                    uint64_t size = value[2];

                    Uart::Puts(log, "Memory base: ");
                    Uart::PutHex(log, base);
                    Uart::Puts(log, "\nMemory size: ");
                    Uart::PutHex(log, size);
                    Uart::Puts(log, "\n");

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
            Uart::Puts(log, "Unknown token\n");
            break;
        }
    }
}

}
// namespace BootLib::DeviceTree
