
#include <BootLib/DeviceTree.h>

#include <BootLib/Uart.h>

#include <concepts>
#include <span>
#include <string_view>

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#pragma GCC optimize("no-tree-vectorize,no-tree-slp-vectorize")

namespace BootLib::DeviceTree
{

MemoryRange memoryRanges[16]{};
size_t      memoryRangeCount = 0;

Cpu      cpus[16]{};
size_t   cpuCount = 0;

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

struct ParseState
{
    Uart::PL011Uart*    log;
    char const*         strings;
    BE<uint32_t> const* struct_block;
    uint32_t            addressCells[4]{ 1 };
    uint32_t            sizeCells   [4]{ 1 };
    uint32_t            currentCells = 0;

    void PushCells(uint32_t address, uint32_t size)
    {
        if (currentCells < 3)
        {
            ++currentCells;
            addressCells[currentCells] = address;
            sizeCells   [currentCells] = size;
        }
    }

    void PopCells()
    {
        if (currentCells > 0)
        {
            --currentCells;
        }
    }

    void SetCurrentAddressCells(uint32_t value) { addressCells[currentCells] = value; }
    void SetCurrentSizeCells   (uint32_t value) { sizeCells   [currentCells] = value; }

    uint32_t GetCurrentAddressCells() const { return addressCells[currentCells]; }
    uint32_t GetCurrentSizeCells   () const { return sizeCells   [currentCells]; }
};

template < typename F > concept NodeFunction     = std::predicate<F, ParseState&, std::string_view /* name */, std::string_view /* address */>;
template < typename F > concept PropertyFunction = std::predicate<F, ParseState&, std::string_view /* name */, std::span<BE<uint32_t> const>>;

bool ParseNode(ParseState& state, NodeFunction auto&& node, PropertyFunction auto&& property)
{
    for (;;)
    {
        uint32_t const token = *state.struct_block++;
        switch (token)
        {
        case FDT_BEGIN_NODE: {
            std::string_view name{ reinterpret_cast<char const*>(state.struct_block) };
            state.struct_block += (name.size() + 4) / 4;
            auto const atSeparator = name.find_first_of('@');
            std::string_view address{};
            if (atSeparator != name.npos)
            {
                address = name.substr(atSeparator + 1);
                name    = name.substr(0, atSeparator);
            }
            //printf("Begin node name: '%s'\n", name.data());
            state.PushCells(state.GetCurrentAddressCells(), state.GetCurrentSizeCells());
            if (!node(state, name, address))
            {
                return false;
            }
            state.PopCells();
            break;
        }
        case FDT_PROP: {
            uint32_t         const len = *state.struct_block++;
            std::string_view const name = state.strings + *state.struct_block++;
            std::span value{ state.struct_block, (len + 3) / 4 };
            state.struct_block += value.size();
            if (name == "#address-cells")
            {
                state.SetCurrentAddressCells(value[0]);
            }
            else if (name == "#size-cells")
            {
                state.SetCurrentSizeCells(value[0]);
            }
            else if (!property(state, name, value))
            {
                return false;
            }
            break;
        }
        case FDT_END_NODE: return true;
        case FDT_END:      return false;
        case FDT_NOP:      break;
        default:
            Uart::Puts(state.log, "Unknown token\n");
            Uart::PutDec(state.log, token);
            return false;
        }
    }
}

bool PrintProperty(ParseState& state, std::string_view name, std::span<BE<uint32_t> const> value)
{
    Uart::Puts(state.log, "  Property: ");
    Uart::Puts(state.log, name.data());
    Uart::Puts(state.log, "\n");
    return true;
}

bool SkipNode(ParseState& state)
{
    return ParseNode(state,
        [](ParseState& state, std::string_view name, std::string_view address)
        {
            return SkipNode(state);
        },
        [](ParseState& state, std::string_view name, std::span<BE<uint32_t> const> value)
        {
            return true;
        }
    );
}

bool ParseMemoryNode(ParseState& state)
{
    std::span<BE<uint32_t> const> reg;
    if (!ParseNode(state,
            [](ParseState& state, std::string_view name, std::string_view address)
            {
                return SkipNode(state);
            },
            [&](ParseState& state, std::string_view name, std::span<BE<uint32_t> const> value)
            {
                if (name == "reg")
                {
                    // Handle memory region
                    Uart::Puts(state.log, "  Memory region found\n");
                    reg = value;
                }
                return true;
            }
        ))
    {
        return false;
    }
    if (!reg.empty())
    {
        for (size_t i = 0; i < reg.size(); i += state.GetCurrentAddressCells() + state.GetCurrentSizeCells())
        {
            uintptr_t base = 0;
            uintptr_t size = 0;
            for (size_t j = 0; j < state.GetCurrentAddressCells(); ++j, ++i)
            {
                base = (base << 32) + reg[i];
            }
            for (size_t j = 0; j < state.GetCurrentSizeCells(); ++j, ++i)
            {
                size = (size << 32) + reg[i];
            }
            if (size > 0)
            {
                memoryRanges[memoryRangeCount++] = { base, size };
                Uart::Puts(state.log, "  Memory range added\n");
                Uart::Puts(state.log, "    Base: "); Uart::PutHex(state.log, base); Uart::Puts(state.log, " Size: "); Uart::PutHex(state.log, size); Uart::Puts(state.log, "\n");
            }
        }
        
    }
    return true;
}

bool ParseRootNode(ParseState& state)
{
    return ParseNode(state,
        [](ParseState& state, std::string_view name, std::string_view address)
        {
            if (name == "memory")
            {
                // Handle memory node
                Uart::Puts(state.log, "Memory node found\n");
                return ParseMemoryNode(state);
            }
            else
            {
                Uart::Puts(state.log, "Unknown node: ");
                Uart::Puts(state.log, name.data());
                Uart::Puts(state.log, "\n");
                return SkipNode(state);
            }
        },
        [](ParseState& state, std::string_view name, std::span<BE<uint32_t> const> value)
        {
            Uart::Puts(state.log, "  Property: ");
            Uart::Puts(state.log, name.data());
            Uart::Puts(state.log, "\n");
            return true;
        }
    );
}

void ParseDeviceTree(uintptr_t dtb, Uart::PL011Uart* log)
{
    if (dtb == 0)
    {
        Uart::Puts(log, "No DeviceTree found\n");
        return;
    }
    auto const hdr = reinterpret_cast<Header const*>(dtb);
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
        BE<uint64_t>* mem_rsvmap = reinterpret_cast<BE<uint64_t>*>(dtb + hdr->off_mem_rsvmap);
        while (mem_rsvmap[0] != 0 && mem_rsvmap[1] != 0) {
            Uart::Puts(log, "Memory reservation: base=");
            Uart::PutHex(log, static_cast<uint64_t>(mem_rsvmap[0]));
            Uart::Puts(log, ", size=");
            Uart::PutHex(log, static_cast<uint64_t>(mem_rsvmap[1]));
            Uart::Puts(log, "\n");
            mem_rsvmap += 2;
        }
    }

    ParseState state
    {
        .log          = log,
        .strings      = reinterpret_cast<char const*>(dtb + hdr->off_dt_strings),
        .struct_block = reinterpret_cast<BE<uint32_t> const*>(dtb + hdr->off_dt_struct),
    };

    // Find the root node.
    for (;;)
    {
        uint32_t token = *state.struct_block++;
        if (token == FDT_BEGIN_NODE) {
            std::string_view name{ reinterpret_cast<char const*>(state.struct_block) };
            if (name == "")
            {
                state.struct_block++;
                ParseRootNode(state);
            }
            else
            {
                Uart::Puts(log, "Root node expected, but found: ");
                Uart::Puts(log, name.data());
                Uart::Puts(log, "\n");
                return;
            }
        } else if (token == FDT_END) {
            break;
        } else if (token == FDT_NOP) {
            continue;
        } else {
            Uart::Puts(log, "Unknown expected token: ");
            Uart::PutHex(log, token);
            Uart::Puts(log, "\n");
            break;
        }

//        } else if (token == FDT_END_NODE) {
//            in_root_node = false;
//            in_memory_node = false;
//        } else if (token == FDT_PROP) {
//            uint32_t len = *state.struct_block++;
//            uint32_t nameoff = *state.struct_block++;
//            char const* prop_name = strings + nameoff;
//            auto* value = state.struct_block;
//            //printf(" Property: %s (%u bytes)\n", prop_name, len);
//
//            if (in_root_node || in_memory_node) {
//                if (strcmp(prop_name, "#address-cells") == 0) {
//                    Uart::Puts(log, "Address cells found: ");
//                    Uart::PutDec(log, static_cast<uint32_t>(*value));
//                    Uart::Puts(log, "\n");
//                } else if (strcmp(prop_name, "#size-cells") == 0) {
//                    Uart::Puts(log, "Size cells found: ");
//                    Uart::PutDec(log, static_cast<uint32_t>(*value));
//                    Uart::Puts(log, "\n");
//                } else if (strcmp(prop_name, "memreserve") == 0) {
//                    Uart::Puts(log, "Memory reservation found: ");
//                    Uart::PutHex(log, static_cast<uint32_t>(value[0]));
//                    Uart::Puts(log, " ");
//                    Uart::PutHex(log, static_cast<uint32_t>(value[1]));
//                    Uart::Puts(log, "\n");
//                }
//            }
//
//            if (in_memory_node && strcmp(prop_name, "reg") == 0) {
//                Uart::Puts(log, "Memory reg found. Length: ");
//                Uart::PutDec(log, len);
//                Uart::Puts(log, "\n");
//                while (len >= 12) {
//                    uint64_t base = (static_cast<uint64_t>(value[0]) << 32) |
//                                    value[1];
//                    uint64_t size = value[2];
//
//                    Uart::Puts(log, "Memory base: ");
//                    Uart::PutHex(log, base);
//                    Uart::Puts(log, "\nMemory size: ");
//                    Uart::PutHex(log, size);
//                    Uart::Puts(log, "\n");
//
//                    value += 3;
//                    len -= 12;
//                }
//                return;
//            }
//
//            state.struct_block += (len + 3) / 4;
    }
}

}
// namespace BootLib::DeviceTree
