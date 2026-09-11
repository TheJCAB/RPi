
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

    std::remove_cv_t<T> get() const { return FromBE(value); }
    operator std::remove_cv_t<T>() const { return FromBE(value); }

    friend void PutHex(BootLib::Stream::Out& out, BE<T> value) { using namespace Stream; PutHex(out, value.get()); }
    friend void PutBin(BootLib::Stream::Out& out, BE<T> value) { using namespace Stream; PutBin(out, value.get()); }
    friend void PutDec(BootLib::Stream::Out& out, BE<T> value) { using namespace Stream; PutDec(out, value.get()); }
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
    Stream::Out         log;
    char const*         strings;
    BE<uint32_t> const* struct_block;
    uint32_t            addressCells[8]{ 1 };
    uint32_t            sizeCells   [8]{ 1 };
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

    uint32_t GetParentAddressCells() const { return currentCells > 0 ? addressCells[currentCells - 1] : addressCells[currentCells]; }
    uint32_t GetParentSizeCells   () const { return currentCells > 0 ? sizeCells   [currentCells - 1] : sizeCells   [currentCells]; }
};

template < typename F > concept NodeFunction     = std::predicate<F, ParseState&, std::string_view /* name */, std::string_view /* address */>;
template < typename F > concept PropertyFunction = std::predicate<F, ParseState&, std::string_view /* name */, std::span<BE<uint32_t> const>>;

bool ParseNode(ParseState& state, NodeFunction auto&& node, PropertyFunction auto&& property)
{
    std::span<BE<uint32_t> const> reg;
    std::span<BE<uint32_t> const> ranges;
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
            if (name == "reg")
            {
                reg = value;
            }
            if (name == "ranges")
            {
                ranges = value;
            }

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
        case FDT_END_NODE: {
            if (!reg.empty())
            {
                Puts(state.log, "  reg:");
                for (auto value : reg)
                {
                    Puts(state.log, " ");
                    PutHex(state.log, value);
                }
                Puts(state.log, "\n");
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
                    Puts(state.log, "  'reg' -- Base: "); PutHex(state.log, base); Puts(state.log, " Size: "); PutHex(state.log, size); Puts(state.log, "\n");
                }
            }

            if (!ranges.empty())
            {
                Puts(state.log, "  ranges:");
                for (auto value : ranges)
                {
                    Puts(state.log, " ");
                    PutHex(state.log, value);
                }
                Puts(state.log, "\n");
                for (size_t i = 0; i < ranges.size(); i += state.GetCurrentAddressCells() + state.GetParentAddressCells() + state.GetCurrentSizeCells())
                {
                    uintptr_t base = 0;
                    uintptr_t parent = 0;
                    uintptr_t size = 0;
                    for (size_t j = 0; j < state.GetCurrentAddressCells(); ++j, ++i)
                    {
                        base = (base << 32) + ranges[i];
                    }
                    for (size_t j = 0; j < state.GetParentAddressCells(); ++j, ++i)
                    {
                        parent = (parent << 32) + ranges[i];
                    }
                    for (size_t j = 0; j < state.GetCurrentSizeCells(); ++j, ++i)
                    {
                        size = (size << 32) + ranges[i];
                    }
                    Puts(state.log, "  'ranges' -- Base: "); PutHex(state.log, base); Puts(state.log, " Parent: "); PutHex(state.log, parent); Puts(state.log, " Size: "); PutHex(state.log, size); Puts(state.log, "\n");
                }
            }

            return true;
        }
        case FDT_END:      return false;
        case FDT_NOP:      break;
        default:
            Puts(state.log, "Unknown token\n");
            PutDec(state.log, token);
            return false;
        }
    }
}

bool PrintProperty(ParseState& state, std::string_view name, std::span<BE<uint32_t> const> value)
{
    Puts(state.log, "  Property: ");
    Puts(state.log, name);
    Puts(state.log, "\n");
    return true;
}

bool SkipNode(ParseState& state)
{
    return ParseNode(state,
        [](ParseState& state, std::string_view name, std::string_view address)
        {
            Puts(state.log, "Unknown skipped node: ");
            Puts(state.log, name);
            if (!address.empty())
            {
                Puts(state.log, "  Address: ");
                Puts(state.log, address);
            }
            Puts(state.log, "\n");
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
                Puts(state.log, "Unknown memory node: ");
                Puts(state.log, name);
                if (!address.empty())
                {
                    Puts(state.log, "  Address: ");
                    Puts(state.log, address);
                }
                Puts(state.log, "\n");
                return SkipNode(state);
            },
            [&](ParseState& state, std::string_view name, std::span<BE<uint32_t> const> value)
            {
                if (name == "reg")
                {
                    // Handle memory region
                    Puts(state.log, "  Memory region found\n");
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
                Puts(state.log, "  : ");
                Puts(state.log, "    Base: "); PutHex(state.log, base); Puts(state.log, " Size: "); PutHex(state.log, size); Puts(state.log, "\n");
            }
        }
        
    }
    return true;
}

bool ParsePsciNode(ParseState& state)
{
    std::span<BE<uint32_t> const> method;
    if (!ParseNode(state,
            [](ParseState& state, std::string_view name, std::string_view address)
            {
                Puts(state.log, "Unknown PSCI node: ");
                Puts(state.log, name);
                if (!address.empty())
                {
                    Puts(state.log, "  Address: ");
                    Puts(state.log, address);
                }
                Puts(state.log, "\n");
                return SkipNode(state);
            },
            [&](ParseState& state, std::string_view name, std::span<BE<uint32_t> const> value)
            {
                if (name == "method")
                {
                    // Handle memory region
                    Puts(state.log, "  Method found\n");
                    method = value;
                }
                return true;
            }
        ))
    {
        return false;
    }
    if (!method.empty())
    {
        Puts(state.log, "  Method: ");
        Puts(state.log, reinterpret_cast<char const*>(method.data()));
        Puts(state.log, "\n");
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
                Puts(state.log, "Memory node found\n");
                return ParseMemoryNode(state);
            }
            else if (name == "psci")
            {
                Puts(state.log, "PSCI node found\n");
                return ParsePsciNode(state);
            }
            else
            {
                Puts(state.log, "Unknown root node: ");
                Puts(state.log, name);
                if (!address.empty())
                {
                    Puts(state.log, "  Address: ");
                    Puts(state.log, address);
                }
                Puts(state.log, "\n");
                return SkipNode(state);
            }
        },
        [](ParseState& state, std::string_view name, std::span<BE<uint32_t> const> value)
        {
            Puts(state.log, "  Property: ");
            Puts(state.log, name);
            Puts(state.log, "\n");
            return true;
        }
    );
}

void ParseDeviceTree(uintptr_t dtb, Stream::Out const& log)
{
    if (dtb == 0)
    {
        Puts(log, "No DeviceTree found\n");
        return;
    }
    auto const hdr = reinterpret_cast<Header const*>(dtb);
    if (hdr->magic != FDT_MAGIC)
    {
        Puts(log, "Invalid DTB magic\n");
        return;
    }

    Puts(log, "DTB magic found\n");
    Puts(log, "DTB total size:                    "); PutDec(log, static_cast<uint32_t>(hdr->totalsize        )); Puts(log, " bytes\n");
    Puts(log, "DTB structure offset:              "); PutDec(log, static_cast<uint32_t>(hdr->off_dt_struct    )); Puts(log, " bytes\n");
    Puts(log, "DTB strings offset:                "); PutDec(log, static_cast<uint32_t>(hdr->off_dt_strings   )); Puts(log, " bytes\n");
    Puts(log, "DTB version:                       "); PutDec(log, static_cast<uint32_t>(hdr->version          )); Puts(log, "\n");
    Puts(log, "DTB last compatible version:       "); PutDec(log, static_cast<uint32_t>(hdr->last_comp_version)); Puts(log, "\n");
    Puts(log, "DTB boot CPU ID:                   "); PutDec(log, static_cast<uint32_t>(hdr->boot_cpuid_phys  )); Puts(log, "\n");
    Puts(log, "DTB size of strings:               "); PutDec(log, static_cast<uint32_t>(hdr->size_dt_strings  )); Puts(log, " bytes\n");
    Puts(log, "DTB size of structure:             "); PutDec(log, static_cast<uint32_t>(hdr->size_dt_struct   )); Puts(log, " bytes\n");
    Puts(log, "DTB memory reservation map offset: "); PutDec(log, static_cast<uint32_t>(hdr->off_mem_rsvmap   )); Puts(log, " bytes\n");

    if (hdr->off_mem_rsvmap != 0)
    {
        // TODO: Do something with these.
        BE<uint64_t>* mem_rsvmap = reinterpret_cast<BE<uint64_t>*>(dtb + hdr->off_mem_rsvmap);
        while (mem_rsvmap[0] != 0 && mem_rsvmap[1] != 0) {
            Puts(log, "Memory reservation: base=");
            PutHex(log, static_cast<uint64_t>(mem_rsvmap[0]));
            Puts(log, ", size=");
            PutHex(log, static_cast<uint64_t>(mem_rsvmap[1]));
            Puts(log, "\n");
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
                Puts(log, "Root node expected, but found: ");
                Puts(log, name);
                Puts(log, "\n");
                return;
            }
        } else if (token == FDT_END) {
            break;
        } else if (token == FDT_NOP) {
            continue;
        } else {
            Puts(log, "Unknown expected token: ");
            PutHex(log, token);
            Puts(log, "\n");
            break;
        }
    }
}

}
// namespace BootLib::DeviceTree
