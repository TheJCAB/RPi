#pragma once

#include <stdint.h>
#include <stddef.h>

class SdCard
{
    union Registers;
public:
    static constexpr uint32_t RegistersOffset = 0x30'0000u;
    //static constexpr uint32_t RegistersOffset = 0x34'0000u;

    SdCard(uintptr_t registersBase) : registers(*reinterpret_cast<Registers*>(registersBase)) {}

    bool Init();
    bool ReadBlock(uint32_t block, void* buffer, uint32_t count = 1);
    void WriteBlock(uint32_t block, void const* buffer);

private:
    Registers& registers;
};
