
#include <BootLib/DeviceTree.h>

#include <BootLib/ArrayVector.h>
#include <BootLib/Uart.h>

#include <algorithm>
#include <array>
#include <concepts>
#include <span>
#include <string_view>

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#pragma GCC optimize("no-tree-vectorize,no-tree-slp-vectorize")

namespace BootLib::DeviceTree
{

Model           model           = Model          ::Invalid;
CpuWakeupMethod cpuWakeupMethod = CpuWakeupMethod::Invalid;

ArrayVector<MemoryRange,   16> memoryRanges{};
ArrayVector<Cpu        ,   16> cpus        {};
ArrayVector<Device     , 1024> devices     {};

// ----------------------------------------------------------------------------

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

// ----------------------------------------------------------------------------

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
    Stream::Out         log          {};
    char const*         strings      = nullptr;
    BE<uint32_t> const* struct_block = nullptr;

    struct Level
    {
        std::string_view                  name         {};
        uint32_t                          phandle      = UINT32_MAX;
        uint32_t                          addressCells = 1;
        uint32_t                          sizeCells    = 1;
        ArrayVector<MemoryRange      , 4> reg          {};
        ArrayVector<DeviceMemoryRange, 4> ranges       {};
    };
    ArrayVector<Level, 8> levels
    {
        {
            .name         {},
            .addressCells = 1,
            .sizeCells    = 1,
        }
    };

    void PushLevel(std::string_view const name)
    {
        auto& parent = levels.back();
        auto& newLevel = levels.push_back(Level{});
        newLevel.name         = name;
        newLevel.addressCells = parent.addressCells;
        newLevel.sizeCells    = parent.sizeCells;
    }

    void PopLevel()
    {
        levels.pop_back();
    }

    Level&       CurrentLevel()       { return levels.back(); }
    Level const& CurrentLevel() const { return levels.back(); }
    Level const& ParentLevel () const { return levels.size() >= 2 ? levels[levels.size() - 2] : levels.back(); }

    void LogName()
    {
        for (size_t i = 1; i < levels.size(); ++i)
        {
            if (i > 1)
            {
                Puts(log, ".");
            }
            Puts(log, levels[i].name);
        }
    }
};

void ParseRegProperty(ParseState& state, std::span<BE<uint32_t> const> reg, auto&& callback)
{
    for (size_t i = 0; i < reg.size();)
    {
        auto const sizePos = std::min(i       + state.ParentLevel().addressCells, reg.size());
        auto const sizeEnd = std::min(sizePos + state.ParentLevel().sizeCells   , reg.size());
        callback(
            reg.subspan(i      , sizePos - i      ),
            reg.subspan(sizePos, sizeEnd - sizePos)
        );
        i = sizeEnd;
    }
}

void ParseRangesProperty(ParseState& state, std::span<BE<uint32_t> const> ranges, auto&& callback)
{
    Puts(state.log, "Parsing 'ranges'. Current("); PutDec(state.log, state.CurrentLevel().addressCells); Puts(state.log, ", Size"); PutDec(state.log, state.CurrentLevel().sizeCells); Puts(state.log, ") Parent("); PutDec(state.log, state.ParentLevel().addressCells); Puts(state.log, ", Size"); PutDec(state.log, state.ParentLevel().sizeCells); Puts(state.log, ")\n");
    for (size_t i = 0; i < ranges.size();)
    {
        auto const parentPos = std::min(i         + state.CurrentLevel().addressCells, ranges.size());
        auto const sizePos   = std::min(parentPos + state.ParentLevel ().addressCells, ranges.size());
        auto const sizeEnd   = std::min(sizePos   + state.CurrentLevel().sizeCells   , ranges.size());
        callback(
            ranges.subspan(i        , parentPos - i        ),
            ranges.subspan(parentPos, sizePos   - parentPos),
            ranges.subspan(sizePos  , sizeEnd   - sizePos  )
        );
        i = sizeEnd;
    }
}

std::string_view ParseStringProperty(std::span<BE<uint32_t> const> data)
{
    std::string_view value{ reinterpret_cast<char const*>(data.data()), data.size() * sizeof(data[0]) };
    auto const nulPos = value.find_last_not_of('\0');
    if (nulPos != value.npos)
    {
        return value.substr(0, nulPos + 1);
    }
    else
    {
        return value;
    }
}

bool EnumerateStringListProperty(std::span<BE<uint32_t> const> data, auto&& callback)
{
    std::string_view value = ParseStringProperty(data);
    while (!value.empty())
    {
        auto const nulPos = value.find('\0');
        if (nulPos == value.npos)
        {
            return callback(value);
        }
        else
        {
            if (nulPos > 0)
            {
                if (callback(value.substr(0, nulPos)))
                {
                    return true;
                }
            }
            value = value.substr(nulPos + 1);
        }
    }
    return false;
}

template < typename F > concept NodeFunction     = std::predicate<F, ParseState&, std::string_view /* name */, std::string_view /* address */>;
template < typename F > concept PropertyFunction = std::predicate<F, ParseState&, std::string_view /* name */, std::span<BE<uint32_t> const>>;

bool ParseNode(ParseState& state, NodeFunction auto&& node, PropertyFunction auto&& property)
{
    uint32_t phandle = UINT32_MAX;
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
            state.PushLevel(name);
            if (!node(state, name, address))
            {
                Puts(state.log, "  Failed node: ");
                state.LogName();
                Puts(state.log, ".");
                Puts(state.log, name);
                Puts(state.log, "\n");
                state.PopLevel();
                return false;
            }
            state.PopLevel();
            break;
        }
        case FDT_PROP: {
            uint32_t         const len = *state.struct_block++;
            std::string_view const name = state.strings + *state.struct_block++;
            std::span value{ state.struct_block, (len + 3) / 4 };
            state.struct_block += value.size();
            bool handled = true;
            if      (name == "phandle"       ) phandle = value[0];
            else if (name == "linux,phandle" ) phandle = value[0];
            else if (name == "reg"           ) reg    = value;
            else if (name == "ranges"        ) ranges = value;
            else if (name == "#address-cells") state.CurrentLevel().addressCells = value[0];
            else if (name == "#size-cells"   ) state.CurrentLevel().sizeCells    = value[0];
            else
            {
                // Not a standard property we handle here.
                handled = false;
            }

            if (!property(state, name, value) && !handled)
            {
                Puts(state.log, "  Skipped property: ");
                state.LogName();
                Puts(state.log, "->");
                Puts(state.log, name);
                Puts(state.log, "\n");
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
                auto& levelReg = state.CurrentLevel().reg;
                ParseRegProperty(state, reg,
                    [&](std::span<BE<uint32_t> const> const baseArray, std::span<BE<uint32_t> const> const sizeArray)
                    {
                        uintptr_t base = 0;
                        uintptr_t size = 0;
                        for (auto&& value : baseArray)
                        {
                            base = (base << 32) + value;
                        }
                        for (auto&& value : sizeArray)
                        {
                            size = (size << 32) + value;
                        }
                        //Puts(state.log, "  'reg' -- Base: "); PutHex(state.log, base); Puts(state.log, " Size: "); PutHex(state.log, size); Puts(state.log, "\n");
                        if (levelReg.size() < levelReg.capacity())
                        {
                            levelReg.push_back({
                                .base = base,
                                .size = size,
                                .type = MemoryType::Invalid, // We just don't know here. To be determined at a higher level.
                            });
                        }
                    }
                );
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
                auto& levelRanges = state.CurrentLevel().ranges;
                ParseRangesProperty(state, ranges,
                    [&](std::span<BE<uint32_t> const> const baseArray, std::span<BE<uint32_t> const> const parentArray, std::span<BE<uint32_t> const> const sizeArray)
                    {
                        uintptr_t base = 0;
                        uintptr_t parent = 0;
                        uintptr_t size = 0;
                        for (auto&& value : baseArray)
                        {
                            base = (base << 32) + value;
                        }
                        for (auto&& value : parentArray)
                        {
                            parent = (parent << 32) + value;
                        }
                        for (auto&& value : sizeArray)
                        {
                            size = (size << 32) + value;
                        }
                        Puts(state.log, "  'ranges' -- Base: "); PutHex(state.log, base); Puts(state.log, " Parent: "); PutHex(state.log, parent); Puts(state.log, " Size: "); PutHex(state.log, size); Puts(state.log, "\n");
                        if (levelRanges.size() < levelRanges.capacity())
                        {
                            levelRanges.push_back({
                                .range{
                                    .base = parent,
                                    .size = size,
                                    .type = MemoryType::Invalid, // We just don't know here. To be determined at a higher level.
                                },
                                .deviceAddress = base,
                                .dmaAddress    = base,
                            });
                        }
                    }
                );
            }

            state.CurrentLevel().phandle = phandle;

            return true;
        }
        case FDT_END:
            Puts(state.log, "Unexpected end of device tree\n");
            return false;
        case FDT_NOP:      break;
        default:
            Puts(state.log, "Unknown token\n");
            PutDec(state.log, token);
            return false;
        }
    }
}

bool SkipNode(ParseState& state)
{
    return ParseNode(state,
        [](ParseState& state, std::string_view name, std::string_view address)
        {
            Puts(state.log, "Unknown skipped node: ");
            state.LogName();
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
            return false;
        }
    );
}

// ----------------------------------------------------------------------------

bool ParseMemoryNode(ParseState& state)
{
    std::span<BE<uint32_t> const> reg;
    if (!ParseNode(state,
            [](ParseState& state, std::string_view name, std::string_view address)
            {
                Puts(state.log, "Unknown memory node: ");
                state.LogName();
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
                if (name == "reg") reg = value;
                else return false;
                return true;
            }
        ))
    {
        return false;
    }
    if (!reg.empty())
    {
        for (size_t i = 0; i < reg.size();)
        {
            uintptr_t base = 0;
            uintptr_t size = 0;
            for (size_t j = 0; j < state.ParentLevel().addressCells; ++j, ++i)
            {
                base = (base << 32) + reg[i];
            }
            for (size_t j = 0; j < state.ParentLevel().sizeCells; ++j, ++i)
            {
                size = (size << 32) + reg[i];
            }
            if (size > 0)
            {
                if (!memoryRanges.empty() &&
                    memoryRanges.back().type == MemoryType::Normal &&
                    memoryRanges.back().base + memoryRanges.back().size == base)
                {
                    memoryRanges.back().size += size;
                }
                else
                {
                    memoryRanges.push_back({ base, size, MemoryType::Normal });
                }
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
                state.LogName();
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
                if (name == "method") method = value;
                else return false;
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
        auto const methodName = ParseStringProperty(method);
        if (methodName == "hvc") cpuWakeupMethod = CpuWakeupMethod::PsciHvc;
        else if (methodName == "smc") cpuWakeupMethod = CpuWakeupMethod::PsciSmc;
        if (cpuWakeupMethod != CpuWakeupMethod::Invalid)
        {
            Puts(state.log, "  CPU wakeup method: ");
            Puts(state.log, GetName(cpuWakeupMethod));
            Puts(state.log, "\n");
        }
    }
    return true;
}

static constexpr auto ModelList =
    []() constexpr {
        std::pair<std::string_view, Model> models[] =
        {
            { "raspberrypi,3-model-b" , Model::RaspberryPi3B },
            { "Raspberry Pi 3 Model B", Model::RaspberryPi3B },
            { "raspberrypi,4-model-b" , Model::RaspberryPi4B },
            { "Raspberry Pi 4 Model B", Model::RaspberryPi4B },
            { "linux,dummy-virt"      , Model::QemuVirtual   },
        };
        std::sort(std::begin(models), std::end(models), [](auto const& a, auto const& b) { return a.first < b.first; });
        std::array<std::pair<std::string_view, Model>, std::size(models)> result;
        std::copy(std::begin(models), std::end(models), std::begin(result));
        return result;
    }();

void ParseModelProperty(ParseState& state, std::span<BE<uint32_t> const> data)
{
    if (data.empty() || model != Model::Invalid)
    {
        return;
    }

    std::string_view const modelString = ParseStringProperty(data);
    Puts(state.log, "  Model: ");
    Puts(state.log, modelString);
    Puts(state.log, "\n");
    auto const it = std::find_if(std::begin(ModelList), std::end(ModelList), [&](auto const& pair) { return pair.first == modelString; });
    if (it != std::end(ModelList))
    {
        Puts(state.log, "  Model enum: ");
        Puts(state.log, GetName(it->second));
        Puts(state.log, "\n");
        model = it->second;
    }
}

void ParseCompatibleProperty(ParseState& state, std::span<BE<uint32_t> const> data)
{
    if (data.empty() || model != Model::Invalid)
    {
        return;
    }

    if (!data.empty())
    {
        EnumerateStringListProperty(data,
            [&](std::string_view const value)
            {
                Puts(state.log, "  Compatible: ");
                Puts(state.log, value);
                Puts(state.log, "\n");
                auto const it = std::find_if(std::begin(ModelList), std::end(ModelList), [&](auto const& pair) { return pair.first == value; });
                if (it != std::end(ModelList))
                {
                    Puts(state.log, "  Compatible enum: ");
                    Puts(state.log, GetName(it->second));
                    Puts(state.log, "\n");
                    model = it->second;
                    return true;
                }
                return false;
            }
        );
    }
}

bool ParseDeviceNode(ParseState& state, std::string_view name, std::string_view address)
{
    Puts(state.log, "Child node: ");
    state.LogName();
    if (!address.empty())
    {
        Puts(state.log, "  Address: ");
        Puts(state.log, address);
    }
    Puts(state.log, "\n");

    auto const devicesBegin = devices.end();
    std::span<BE<uint32_t> const> compatible;
    if (!ParseNode(state,
        &ParseDeviceNode,
        [&](ParseState& state, std::string_view name, std::span<BE<uint32_t> const> value)
        {
            if      (name == "compatible") compatible = value;
            else return false;
            return true;
        }
    ))
    {
        return false;
    }
    ParseCompatibleProperty(state, compatible);
    auto const devicesEnd = devices.end();

    auto& nodeLevel = state.CurrentLevel();
    if (!nodeLevel.reg.empty())
    {
        auto& device = devices.push_back(
            {
                .name       = name,
                .compatible = ParseStringProperty(compatible),
                .phandle    = nodeLevel.phandle,
            }
        );
        if (device.phandle == UINT32_MAX)
        {
            device.phandle = static_cast<uint32_t>(UINT32_MAX - devices.size());
        }
        for (auto& reg : nodeLevel.reg)
        {
            device.mmio.push_back({
                .range = { .base = reg.base, .size = reg.size },
                .deviceAddress = reg.base,
                // .dmaAddress = reg.base, // TODO: DMA
            });
        }
        Puts(state.log, "  Device node: ");
        state.LogName();
        Puts(state.log, "\n");
    }
    if (!nodeLevel.ranges.empty())
    {
        for (auto& device : std::span{ devicesBegin, devicesEnd })
        {
            for (auto& mmio : device.mmio)
            {
                for (auto& range : nodeLevel.ranges)
                {
                    if (range.deviceAddress <= mmio.range.base && mmio.range.base < range.deviceAddress + range.range.size)
                    {
                        if (device.name == "mailbox")
                        {
                            Puts(state.log, "Adjusting mailbox MMIO to parent bus "); state.LogName(); Puts(state.log, "\n");
                            Puts(state.log, "Old MMIO base: ");
                            PutHex(state.log, mmio.range.base);
                            Puts(state.log, "\n");
                            Puts(state.log, "Range device base: ");
                            PutHex(state.log, range.deviceAddress);
                            Puts(state.log, "\n");
                            Puts(state.log, "Range parent base: ");
                            PutHex(state.log, range.range.base);
                            Puts(state.log, "\n");
                        }
                        mmio.range.base = range.range.base + (mmio.range.base - range.deviceAddress);
                        if (device.name == "mailbox")
                        {
                            Puts(state.log, "New MMIO base: ");
                            PutHex(state.log, mmio.range.base);
                            Puts(state.log, "\n");
                        }
                        break;
                    }
                }
            }
        }
        Puts(state.log, "  Bus node: ");
        state.LogName();
        Puts(state.log, "\n");
    }
    
    if (nodeLevel.reg.empty() && nodeLevel.ranges.empty())
    {
        Puts(state.log, "  Non-device node: ");
        state.LogName();
        Puts(state.log, "\n");
    }

    return true;
}

bool ParseRootNode(ParseState& state)
{
    std::span<BE<uint32_t> const> model;
    std::span<BE<uint32_t> const> compatible;
    if (!ParseNode(state,
        [&](ParseState& state, std::string_view name, std::string_view address)
        {
            if (name == "memory")
            {
                state.LogName();
                Puts(state.log, " - Memory node found\n");
                return ParseMemoryNode(state);
            }
            else if (name == "psci")
            {
                state.LogName();
                Puts(state.log, " - PSCI node found\n");
                return ParsePsciNode(state);
            }
            else
            {
                return ParseDeviceNode(state, name, address);
            }
        },
        [&](ParseState& state, std::string_view name, std::span<BE<uint32_t> const> value)
        {
            if      (name == "model"     ) model      = value;
            else if (name == "compatible") compatible = value;
            else return false;
            return true;
        }
    ))
    {
        return false;
    }
    ParseModelProperty     (state, model);
    ParseCompatibleProperty(state, compatible);
    return true;
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
