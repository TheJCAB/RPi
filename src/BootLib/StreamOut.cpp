#include <BootLib/StreamOut.h>

namespace BootLib::Stream
{

void PutHex(Out const& out, std::unsigned_integral auto value)
{
    if constexpr (std::is_same_v<decltype(value), bool>)
    {
        out.puts(out.context, value ? "0x1" : "0x0");
        return;
    }

    char const* hexDigits = "0123456789ABCDEF";

    char buffer[1 + sizeof(value) * 2 + sizeof(value) / 2];
    size_t size = 2;

    buffer[0] = '0';
    buffer[1] = 'x';

    for (int i = sizeof(value) * 8 - 4; i >= 0; i -= 4)
    {
        buffer[size++] = hexDigits[(value >> i) & 0xF];
        if (i > 0 && i % 16 == 0)
        {
            buffer[size++] = '\''; // Add digit separator for readability
        }
    }
    out.puts(out.context, std::string_view( buffer, buffer + size ));
}

void PutBin(Out const& out, std::unsigned_integral auto value)
{
    if constexpr (std::is_same_v<decltype(value), bool>)
    {
        out.puts(out.context, value ? "0b1" : "0b0");
        return;
    }

    char buffer[1 + sizeof(value) * 8 + sizeof(value) * 2];
    size_t size = 2;

    buffer[0] = '0';
    buffer[1] = 'b';

    for (int i = sizeof(value) * 8 - 1; i >= 0; --i)
    {
        buffer[size++] = '0' + ((value >> i) & 0x1);
        if (i > 0 && i % 4 == 0)
        {
            buffer[size++] = '\''; // Add digit separator for readability
        }
    }
    out.puts(out.context, std::string_view(buffer, buffer + size));
}

void PutDec(Out const& out, std::unsigned_integral auto value)
{
    if (value == 0)
    {
        out.putc(out.context, '0');
        return;
    }

    if constexpr (std::is_same_v<decltype(value), bool>)
    {
        out.putc(out.context, '1');
        return;
    }

    char buffer[25]; // Enough for 64-bit integer
    int index = sizeof(buffer);

    for (int i = 0; value > 0; ++i)
    {
        if (i > 0 && i % 3 == 0)
        {
            buffer[--index] = '\''; // Add digit separator for readability
        }
        buffer[--index] = '0' + (value % 10);
        value /= 10;
    }
    out.puts(out.context, std::string_view(buffer + index, buffer + sizeof(buffer)));
}

template void PutHex(Out const& out, uint64_t);
template void PutHex(Out const& out, uint32_t);
template void PutHex(Out const& out, uint16_t);
template void PutHex(Out const& out, uint8_t );
template void PutHex(Out const& out, bool    );

template void PutBin(Out const& out, uint64_t);
template void PutBin(Out const& out, uint32_t);
template void PutBin(Out const& out, uint16_t);
template void PutBin(Out const& out, uint8_t );
template void PutBin(Out const& out, bool    );

template void PutDec(Out const& out, uint64_t);
template void PutDec(Out const& out, uint32_t);
template void PutDec(Out const& out, uint16_t);
template void PutDec(Out const& out, uint8_t );
template void PutDec(Out const& out, bool    );

}
// namespace BootLib::Stream

extern "C" 
size_t strlen(char const* s)
{
    char const* p = s;
    while (*p != '\0')
    {
        ++p;
    }
    return static_cast<size_t>(p - s);
}
