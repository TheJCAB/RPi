#pragma once

#include <chrono>
#include <string_view>

using namespace std::literals::chrono_literals;

namespace Driver
{

class CharacterIo
{
public:
    virtual ~CharacterIo() = 0 {}

    virtual char Getc(std::chrono::microseconds timeout = std::chrono::microseconds::max()) = 0;
    virtual char TryGetc() = 0;

    virtual void Putc(char) = 0;
    virtual void Puts(std::string_view) = 0;
};

}
// namespace Driver