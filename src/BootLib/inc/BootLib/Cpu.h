#pragma once

#include <stdint.h>
#include <stddef.h>

#include <string_view>
#include <array>

namespace BootLib::Cpu
{

bool IsRpi4();

uint64_t GetPerformanceFrequency();

uint64_t GetPerformanceCounter();

inline uint64_t GetPerformanceTicksForUs(uint64_t us)
{
    return (us * GetPerformanceFrequency() / 1'000'000u);
}

void DelayInMicroseconds(uint64_t us);
void DelayInMilliseconds(uint64_t ms);

inline void Yield() { asm volatile("yield"); }

[[noreturn]] inline void Halt()
{
    // Halt the CPU by entering an infinite loop
    while (true)
    {
        asm volatile("wfe"); // Wait for event (low power state)
    }
}

template < typename T, char const* RegisterName >
struct SystemRegisterProxy
{
    consteval SystemRegisterProxy() = default;

    SystemRegisterProxy(SystemRegisterProxy&&) = delete;
    SystemRegisterProxy& operator=(SystemRegisterProxy&&) = delete;

    static inline T get()
    {
        uint64_t value = 0;
        asm volatile (
            "mrs %0, %1"
            : "=r"(value)
            : "i"(RegisterName)
            : "memory"
        );
        return reinterpret_cast<T&>(value);
    }

    static inline T set(T const& value) requires (!std::is_const_v<T>)
    {
        asm volatile (
            "msr %1, %0"
            :
            : "r"(reinterpret_cast<uint64_t const&>(value)),
              "i"(RegisterName)
            : "memory"
        );
        return value;
    }

    static inline auto modify(std::invocable<T&> auto&& modify) requires (!std::is_const_v<T>)
    {
        struct ModifyProxy
        {
            T value;
            ~ModifyProxy() { set(value); }
        };
        ModifyProxy proxy{ .value = get() };
        return modify(proxy.value);
    }

    inline operator T() const { return get(); }

    inline T operator=(T const& value) const requires (!std::is_const_v<T>) { return set(value); }

    inline auto operator->() const requires (std::is_const_v<T>)
    {
        struct DereferenceProxy
        {
            T const* operator->() { return &value; }
            T const value;
        };
        return DereferenceProxy{ .value = SystemRegisterProxy::get() };
    }

    inline T operator&=(T const& value) const requires (!std::is_const_v<T>) { return *this = *this & value; }
    inline T operator|=(T const& value) const requires (!std::is_const_v<T>) { return *this = *this | value; }
    inline T operator^=(T const& value) const requires (!std::is_const_v<T>) { return *this = *this ^ value; }
    inline T operator+=(T const& value) const requires (!std::is_const_v<T>) { return *this = *this + value; }
    inline T operator-=(T const& value) const requires (!std::is_const_v<T>) { return *this = *this - value; }
};

template < char const* RegisterName >
struct ImmediateSystemRegisterProxy
{
    consteval ImmediateSystemRegisterProxy() = default;

    ImmediateSystemRegisterProxy(ImmediateSystemRegisterProxy&&) = delete;
    ImmediateSystemRegisterProxy& operator=(ImmediateSystemRegisterProxy&&) = delete;

    inline void operator=(uint64_t value) const
    {
        asm volatile (
            "msr %1, %0"
            :
            : "n"(value),
              "i"(RegisterName)
        );
    }

};

#define DEFINE_SYSREG_PROXY(name, type) \
    namespace SysRegName { extern "C" inline constexpr char name[] = #name; } \
    inline constexpr SystemRegisterProxy<type, SysRegName::name> name

#define DEFINE_IMMSYSREG_PROXY(name) \
    namespace ImmSysRegName { extern "C" inline constexpr char name[] = #name; } \
    inline constexpr ImmediateSystemRegisterProxy<ImmSysRegName::name> name

// System Register format structures
namespace SysRegData
{

struct CurrentEL
{
    uint64_t Reserved0 :  2;
    uint64_t EL        :  2; // Exception Level (0b00 = EL0, 0b01 = EL1, 0b10 = EL2, 0b11 = EL3)
    uint64_t Reserved  : 60;
};

struct sctlr_el1
{
    uint64_t M         :  1; // Memory Management Enable
    uint64_t A         :  1; // Alignment Check Enable
    uint64_t C         :  1; // Cache Enable
    uint64_t SA        :  1; // Stack Alignment Check Enable
    uint64_t Reserved0 :  8;
    uint64_t I         :  1; // Instruction Cache Enable
    uint64_t Reserved1 : 51;

};

struct cpacr
{
    uint64_t Reserved0 : 20;
    uint64_t FPEN      :  2; // Floating Point Enable (0b00 = Disabled, 0b01 = EL0, 0b10 = EL1, 0b11 = EL2/EL3)
    uint64_t Reserved1 : 42;

};

}
// namespace SysRegData

DEFINE_SYSREG_PROXY(CurrentEL       , SysRegData::CurrentEL const);
DEFINE_SYSREG_PROXY(sctlr_el1       , SysRegData::sctlr_el1);
DEFINE_SYSREG_PROXY(daif            , uint64_t);
DEFINE_SYSREG_PROXY(cpacr_el1       , SysRegData::cpacr);
DEFINE_SYSREG_PROXY(cptr_el2        , SysRegData::cpacr);

DEFINE_IMMSYSREG_PROXY(daifclr);

inline void InstructionSynchronizationBarrier()
{
    // Ensure that all previous instructions are completed before continuing
    asm volatile ("isb");
}

inline void InnerDataSynchronizationBarrier()
{
    // Ensure that all previous memory accesses are completed and visible by other cores before continuing
    asm volatile ("dsb ish");
}

inline void FullDataSynchronizationBarrier()
{
    // Ensure that all previous memory accesses are completed and visible everywhere (GPU? DRAM?) before continuing
    asm volatile ("dsb sy");
}

}
// namespace BootLib::Cpu
