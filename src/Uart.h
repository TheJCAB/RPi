#pragma once

#include <stdint.h>
#include <stddef.h>

namespace Uart
{

extern bool useMutex;

void Init();

namespace Raw
{

void NoMmuPutc(char c);
void NoMmuPuts(char const* str);

void Putc(char c);
char Getc();
char TryGetc();
void Puts(char const* str);
void PutHex(auto value);
void PutBin(auto value);
void PutDec(auto value);

}
// namespace Raw

void Putc(char c);
char Getc();
char TryGetc();
void Puts(char const* str);
void PutHex(auto value);
void PutBin(auto value);
void PutDec(auto value);

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
    void PutHex(auto value);
    void PutBin(auto value);
    void PutDec(auto value);

private:
    bool locked = false;
};

}
// namespace Uart
