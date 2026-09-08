#include "Uart.h"

#include "Timer.h"

#include "Cpu.h"
#include "Mmio.h"
#include "Gpio.h"

#include <atomic>
#include <mutex>

namespace Uart
{

using BootLib::PL011Uart;

PL011Uart* Uart0 = nullptr;

bool useMutex = false;
//std::atomic<bool> Mutex;
std::mutex Mutex;

void Init(PL011Uart* uart)
{
    Uart0 = uart;
}

namespace Raw
{

char Getc   ()                           { return Uart0 ? Uart0->Getc() : char{0}; }
char TryGetc()                           { return Uart0 ? Uart0->TryGetc() : char{0}; }
void Puts   (std::string_view str)       { Puts(Uart0, str); }
void PutHex (std::integral auto value)   { PutHex(Uart0, value); }
void PutHex (void const volatile* value) { PutHex(Uart0, reinterpret_cast<uintptr_t>(value)); }
void PutBin (std::integral auto value)   { PutBin(Uart0, value); }
void PutDec (std::integral auto value)   { PutDec(Uart0, value); }

template void PutHex(uint64_t value);
template void PutHex(uint32_t value);
template void PutHex(uint16_t value);
template void PutHex(uint8_t  value);
template void PutHex(bool     value);

template void PutBin(uint64_t value);
template void PutBin(uint32_t value);
template void PutBin(uint16_t value);
template void PutBin(uint8_t  value);
template void PutBin(bool     value);

template void PutDec(uint64_t value);
template void PutDec(uint32_t value);
template void PutDec(uint16_t value);
template void PutDec(uint8_t  value);
template void PutDec(bool     value);

} // namespace Raw

LockedStream::LockedStream(bool tryOnly)
{
    if (useMutex)
    {
        wasEnabled = Cpu::DisableInterrupts();
        if (tryOnly)
        {
            locked = Mutex.try_lock();
            if (!locked) Cpu::RestoreInterrupts(wasEnabled);
        }
        else
        {
            // Lock the mutex
            Mutex.lock();
            locked = true;
        }
    }
    else
    {
        locked = true;
    }
}

LockedStream::~LockedStream()
{
    if (locked && useMutex)
    {
        Mutex.unlock();
        Cpu::RestoreInterrupts(wasEnabled);
    }
}

void LockedStream::Unlock()
{
    if (locked && useMutex)
    {
        Mutex.unlock();
        Cpu::RestoreInterrupts(wasEnabled);
    }
    locked = false;
}

char LockedStream::Getc()
{
    if (!locked) return 0;
    return Raw::Getc();
}

char LockedStream::TryGetc()
{
    if (!locked) return 0;
    return Raw::TryGetc();
}

void LockedStream::Puts(std::string_view str)
{
    if (locked) Raw::Puts(str);
}

void LockedStream::PutHex(std::integral auto value)
{
    if (locked) Raw::PutHex(value);
}

void LockedStream::PutHex(void const volatile* value)
{
    if (locked) Raw::PutHex(value);
}

void LockedStream::PutBin(std::integral auto value)
{
    if (locked) Raw::PutBin(value);
}

void LockedStream::PutDec(std::integral auto value)
{
    if (locked) Raw::PutDec(value);
}

template void LockedStream::PutHex(uint64_t value);
template void LockedStream::PutHex(uint32_t value);
template void LockedStream::PutHex(uint16_t value);
template void LockedStream::PutHex(uint8_t  value);
template void LockedStream::PutHex(bool     value);

template void LockedStream::PutBin(uint64_t value);
template void LockedStream::PutBin(uint32_t value);
template void LockedStream::PutBin(uint16_t value);
template void LockedStream::PutBin(uint8_t  value);
template void LockedStream::PutBin(bool     value);

template void LockedStream::PutDec(uint64_t value);
template void LockedStream::PutDec(uint32_t value);
template void LockedStream::PutDec(uint16_t value);
template void LockedStream::PutDec(uint8_t  value);
template void LockedStream::PutDec(bool     value);


char Getc()
{
    return LockedStream{}.Getc();
}

char TryGetc()
{
    return LockedStream{}.TryGetc();
}

void Puts(std::string_view str)
{
    LockedStream{}.Puts(str);
}

void PutHex(std::integral auto value)
{
    LockedStream{}.PutHex(value);
}

void PutHex(void const volatile* value)
{
    LockedStream{}.PutHex(value);
}

void PutBin(std::integral auto value)
{
    LockedStream{}.PutBin(value);
}

void PutDec(std::integral auto value)
{
    LockedStream{}.PutDec(value);
}

template void PutHex(uint64_t value);
template void PutHex(uint32_t value);
template void PutHex(uint16_t value);
template void PutHex(uint8_t  value);
template void PutHex(bool     value);

template void PutBin(uint64_t value);
template void PutBin(uint32_t value);
template void PutBin(uint16_t value);
template void PutBin(uint8_t  value);
template void PutBin(bool     value);

template void PutDec(uint64_t value);
template void PutDec(uint32_t value);
template void PutDec(uint16_t value);
template void PutDec(uint8_t  value);
template void PutDec(bool     value);


}
// namespace Uart

extern "C" void RawPuts(std::string_view str)
{
    Uart::Raw::Puts(str);
}

extern "C" void RawPutHex64(uint64_t value)
{
    Uart::Raw::PutHex(value);
}