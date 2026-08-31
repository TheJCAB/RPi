#pragma once

#include <stdint.h>
#include <stddef.h>

#include <type_traits>
#include <compare>
#include <concepts>

namespace BootLib::Cpu
{

bool IsRpi4();
bool IsQemu();

enum class PerformanceTime     : uint64_t {};
enum class PerformanceTimeDiff : int64_t  {};

constexpr PerformanceTime operator+ (PerformanceTime a, std::integral auto b) { return static_cast<PerformanceTime>(static_cast<uint64_t>(a) + static_cast<uint64_t>(b)); }
constexpr PerformanceTime operator- (PerformanceTime a, std::integral auto b) { return static_cast<PerformanceTime>(static_cast<uint64_t>(a) - static_cast<uint64_t>(b)); }

constexpr PerformanceTime operator+ (PerformanceTime a, PerformanceTimeDiff b) { return static_cast<PerformanceTime>(static_cast<uint64_t>(a) + static_cast<uint64_t>(b)); }
constexpr PerformanceTime operator- (PerformanceTime a, PerformanceTimeDiff b) { return static_cast<PerformanceTime>(static_cast<uint64_t>(a) - static_cast<uint64_t>(b)); }

constexpr PerformanceTime& operator+=(PerformanceTime& a, std::integral auto b) { return a = a + b; }
constexpr PerformanceTime& operator-=(PerformanceTime& a, std::integral auto b) { return a = a - b; }

constexpr PerformanceTime& operator+=(PerformanceTime& a, PerformanceTimeDiff b) { return a = a + b; }
constexpr PerformanceTime& operator-=(PerformanceTime& a, PerformanceTimeDiff b) { return a = a - b; }

constexpr PerformanceTimeDiff operator-  (PerformanceTime a, PerformanceTime b) { return static_cast<PerformanceTimeDiff>(static_cast<int64_t>(a) - static_cast<int64_t>(b)); }
constexpr auto                operator<=>(PerformanceTime a, PerformanceTime b) { return static_cast<int64_t>(a - b) <=> int64_t{0}; }

uint64_t        GetPerformanceFrequency();
PerformanceTime GetPerformanceCounter  ();
void            DelayUntilPerformanceTime (PerformanceTime);

inline PerformanceTime GetMaximumFuturePerformanceTime(PerformanceTime currentTime) { return currentTime + INT64_MAX; }
inline PerformanceTime GetFarFuturePerformanceTime    (PerformanceTime currentTime) { return currentTime + INT64_MAX/2; }

inline PerformanceTime GetMaximumFuturePerformanceTime() { return Cpu::GetPerformanceCounter() + INT64_MAX; }
inline PerformanceTime GetFarFuturePerformanceTime    () { return Cpu::GetPerformanceCounter() + INT64_MAX/2; }

inline PerformanceTimeDiff GetPerformanceTicksForUs(std::unsigned_integral auto us) { return static_cast<PerformanceTimeDiff>(us * GetPerformanceFrequency() / 1'000'000u); }
inline PerformanceTimeDiff GetPerformanceTicksForMs(std::unsigned_integral auto ms) { return static_cast<PerformanceTimeDiff>(ms * GetPerformanceFrequency() /     1'000u); }

inline PerformanceTimeDiff GetPerformanceTicksForUs(std::signed_integral auto us) { return static_cast<PerformanceTimeDiff>(us * static_cast<int64_t>(GetPerformanceFrequency()) / 1'000'000); }
inline PerformanceTimeDiff GetPerformanceTicksForMs(std::signed_integral auto ms) { return static_cast<PerformanceTimeDiff>(ms * static_cast<int64_t>(GetPerformanceFrequency()) /     1'000); }

inline int64_t GetUsForPerformanceTicks(PerformanceTimeDiff timeDiff) { return static_cast<int64_t>(timeDiff) * 1'000'000 / static_cast<int64_t>(GetPerformanceFrequency()); }
inline int64_t GetMsForPerformanceTicks(PerformanceTimeDiff timeDiff) { return static_cast<int64_t>(timeDiff) *     1'000 / static_cast<int64_t>(GetPerformanceFrequency()); }

inline void DelayInMicroseconds(uint64_t us) { DelayUntilPerformanceTime(GetPerformanceCounter() + GetPerformanceTicksForUs(us)); }
inline void DelayInMilliseconds(uint64_t ms) { DelayUntilPerformanceTime(GetPerformanceCounter() + GetPerformanceTicksForMs(ms)); }

inline void Yield() { asm volatile("yield"); }

[[noreturn]] void Halt();

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

    inline auto operator->() const
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
//
    //inline void operator=(uint64_t value) const
    //{
    //    asm volatile (
    //        "msr %1, %0"
    //        :
    //        : "n"(value),
    //          "i"(RegisterName)
    //    );
    //}

    template < uint64_t value >
    inline void set() const
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
    static_assert(std::is_standard_layout_v<type>, "System register type must be standard layout"); \
    static_assert(sizeof(type) == 8, "System register type must be 64 bits"); \
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

struct mpidr_el1
{
    uint64_t CoreId    :  2; // RPi SoCs have just 4 cores.
    uint64_t Reserved0 : 62; // The rest is unused.
};

struct cntv_ctl_el0
{
    uint64_t Enable    :  1; // Timer Enable
    uint64_t IMASK     :  1; // Interrupt Mask
    uint64_t ISTATUS   :  1; // Interrupt Status
    uint64_t Reserved0 : 61; // The rest is unused.
};

struct spsr_el1
{
    uint64_t SP        :  1; // When EL1... 0: SP_EL0, 1: SP_EL1
    uint64_t Zero      :  1;
    uint64_t EL        :  2; // Mode (0b00 = EL0, 0b01 = EL1, 0x10 = EL1 with NV)
    uint64_t A32       :  1; // 0: AArch64, 1: AArch32 (Note: the rest of bits below are AArch64 specific)
    uint64_t Reserved0 :  1;
    uint64_t F         :  1; // FIQ Enable
    uint64_t I         :  1; // IRQ Enable
    uint64_t A         :  1; // Asynchronous Abort Enable
    uint64_t D         :  1; // Debug Enable
    uint64_t BType     :  2;
    uint64_t SSBS      :  1;
    uint64_t AllInt    :  1;
    uint64_t Reserved1 :  6;
    uint64_t IL        :  1;
    uint64_t SS        :  1;
    uint64_t PAN       :  1;
    uint64_t UAO       :  1;
    uint64_t DIT       :  1;
    uint64_t TCO       :  1;
    uint64_t Reserved2 :  2;
    uint64_t V         :  1;
    uint64_t C         :  1;
    uint64_t Z         :  1;
    uint64_t N         :  1;
    uint64_t PM        :  1;
    uint64_t PPEnd     :  1;
    uint64_t ExLock    :  1;
    uint64_t PacM      :  1;
    uint64_t Reserved3 : 28; // The rest is unused.
};

struct esr_el1
{
    uint64_t ISS      : 25; // Instruction Specific Syndrome
    uint64_t IL       :  1;
    uint64_t EC       :  6; // Exception Class
    uint64_t Reserved : 32; // The rest is unused.
};

}
// namespace SysRegData

DEFINE_SYSREG_PROXY(CurrentEL       , SysRegData::CurrentEL const);
DEFINE_SYSREG_PROXY(sctlr_el1       , SysRegData::sctlr_el1);
DEFINE_SYSREG_PROXY(daif            , uint64_t);
DEFINE_SYSREG_PROXY(cpacr_el1       , SysRegData::cpacr);
DEFINE_SYSREG_PROXY(cptr_el2        , SysRegData::cpacr);
DEFINE_SYSREG_PROXY(mpidr_el1       , SysRegData::mpidr_el1);
DEFINE_SYSREG_PROXY(spsr_el1        , SysRegData::spsr_el1);
DEFINE_SYSREG_PROXY(esr_el1         , SysRegData::esr_el1);

// Virtual timer.
DEFINE_SYSREG_PROXY(cntv_tval_el0   , uint64_t);
DEFINE_SYSREG_PROXY(cntv_ctl_el0    , SysRegData::cntv_ctl_el0);

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

inline void Breakpoint()
{
    // Break into the debugger.
    asm volatile ("brk #0");
}

}
// namespace BootLib::Cpu
