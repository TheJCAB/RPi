#pragma once

#include <stdint.h>
#include <stddef.h>

#include <concepts>
#include <string_view>

namespace BootLib::Stream
{

struct Out
{
    uintptr_t context;
    void (*putc)(uintptr_t context, char);
    void (*puts)(uintptr_t context, std::string_view);

    friend void Puts(Out const& out, std::string_view s) { out.puts(out.context, s); }

    friend void PutHex(Out const& out, std::unsigned_integral auto value);
    friend void PutBin(Out const& out, std::unsigned_integral auto value);
    friend void PutDec(Out const& out, std::unsigned_integral auto value);

    friend void PutHex(Out const& out, void const volatile* value) { PutHex(out, reinterpret_cast<uintptr_t>(value)); }
};

}
// namespace BootLib::Stream
