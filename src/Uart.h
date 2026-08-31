#pragma once

#include <BootLib/Uart.h>

#include <stdint.h>
#include <stddef.h>

#include <concepts>

namespace Uart
{

using BootLib::PL011Uart;

extern bool useMutex;

void Init(PL011Uart* uart);

namespace Raw
{

void NoMmuPutc(char c);
void NoMmuPuts(char const* str);

void Putc(char c);
char Getc();
char TryGetc();
void Puts(char const* str);
void PutHex(std::integral auto value);
void PutHex(void const volatile* value);
void PutBin(std::integral auto value);
void PutDec(std::integral auto value);

}
// namespace Raw

void Putc(char c);
char Getc();
char TryGetc();
void Puts(char const* str);
void PutHex(std::integral auto value);
void PutHex(void const volatile* value);
void PutBin(std::integral auto value);
void PutDec(std::integral auto value);

class LockedStream
{
public:
    LockedStream(bool tryOnly = false);
    ~LockedStream();

    LockedStream(LockedStream const&) = delete;
    LockedStream& operator=(LockedStream const&) = delete;

    LockedStream(LockedStream&&) = delete;
    LockedStream& operator=(LockedStream&&) = delete;

    explicit operator bool() const { return locked; }

    void Putc(char c);
    char Getc();
    char TryGetc();
    void Puts(char const* str);
    void PutHex(std::integral auto value);
    void PutHex(void const volatile* value);
    void PutBin(std::integral auto value);
    void PutDec(std::integral auto value);

    void Unlock();

private:
    bool locked     = false;
    bool wasEnabled = false;
};

}
// namespace Uart
