#pragma once

#include <BootLib/Mmio.h>

#include <stdint.h>

#include <concepts>
#include <type_traits>
#include <utility>

// This file defines the MMIO (Memory-Mapped I/O) memory regions for the Raspberry Pi,
// As well as convenience types for accessing MMIO registers.
namespace Mmio
{

// We handle the various MMIO bases dynamically so we can relocate them via the MMU.

extern uintptr_t Base;      // 0x2000'0000u for Raspberry Pi 1, 0x3F00'0000u for Raspberry Pi 2/3, 0x7E00'0000u or 0x4'7E00'0000u for Raspberry Pi 4
extern uintptr_t QA7Base;   // 0x4000'0000u for Raspberry Pi 1/2/3, 0xFF80'0000u or 0x4'C000'0000ull for Raspberry Pi 4

// All registers are 32-bit wide.
// We admit as a register any type that fits and is trivial to copy.
template < typename T >
concept RegisterType = (std::is_trivially_copyable_v<std::remove_const_t<T>>) && (sizeof(T) == sizeof(uint32_t));

template < uintptr_t const& Base, bool isConst, uint32_t Offset = static_cast<uint32_t>(-1) >
struct RegisterProxyBase
{
    auto& RefUint32() const 
    {
        if constexpr (isConst)
        {
            // If T is const, we return a const reference to the register.
            return *reinterpret_cast<uint32_t const volatile*>(Base + Offset);
        }
        else
        {
            // If T is not const, we return a non-const reference to the register.
            return *reinterpret_cast<uint32_t volatile*>(Base + Offset);
        }
    }
};

template < uintptr_t const& Base, bool isConst >
struct RegisterProxyBase<Base, isConst, static_cast<uint32_t>(-1)>
{
    uint32_t Offset;

    constexpr explicit RegisterProxyBase(uint32_t offs) : Offset(offs) {}

    auto& RefUint32() const
    {
        if constexpr (isConst)
        {
            // If T is const, we return a const reference to the register.
            return *reinterpret_cast<uint32_t const volatile*>(Base + Offset);
        }
        else
        {
            // If T is not const, we return a non-const reference to the register.
            return *reinterpret_cast<uint32_t volatile*>(Base + Offset);
        }
    }
};

/// @brief Proxy template for accessing memory-mapped I/O (MMIO) registers.
///
/// This template provides a type-safe and convenient interface for reading and writing
/// 32-bit hardware registers mapped into memory, such as those used on Raspberry Pi platforms.
/// The base address of the MMIO region is provided as a reference, allowing for dynamic relocation
/// (e.g., via the MMU). The template enforces that only trivially copyable types of 32 bits can be used,
/// ensuring safe and predictable register access.
///
/// Usage:
///   - Instantiate RegisterProxy with a reference to the MMIO base and a register type.
///   - Use assignment operators to write values or modify the register in-place.
///   - Use the conversion operator or dereference to read the current register value.
///
/// Example:
///   uintptr_t Base = 0x3F00'0000u;         // Raspberry Pi 3 peripheral Mmio registers
///   BaseRegisterProxy<Base, uint32_t> reg(0x10); // Access register at offset 0x10 from Base
///   reg = 0x12345678;                      // Write a whole value to the register
///   uint32_t val = reg;                    // Read the register
///   reg = [](auto& v) { v |= 0x1; };       // Read the register, modify the value then write it back
///
/// @tparam Base Reference to the MMIO base address variable.
/// @tparam T    Register type (must conform to the RegisterType concept).
///
template < uintptr_t const& Base, RegisterType T, uint32_t Offset = static_cast<uint32_t>(-1) >
struct RegisterProxy : RegisterProxyBase<Base, std::is_const_v<T>, Offset>
{
    using RegisterProxyBase<Base, std::is_const_v<T>, Offset>::RegisterProxyBase;
    using RegisterProxyBase<Base, std::is_const_v<T>, Offset>::RefUint32;

    T get() const
    {
        uint32_t const result = RefUint32();
        return reinterpret_cast<T const&>(result);
    }

    T set(const T& value) const requires (!std::is_const_v<T>)
    {
        RefUint32() = reinterpret_cast<uint32_t const&>(value);
        return value;
    }

    T operator=(std::integral auto value) const requires (!std::is_const_v<T>) && (sizeof(value) <= sizeof(uint32_t))
    {
        uint32_t v = static_cast<uint32_t>(value);
        RefUint32() = v;
        return reinterpret_cast<T const&>(v);
    }

    T operator=(const T& value) const requires (!std::is_const_v<T>)
    {
        RefUint32() = reinterpret_cast<uint32_t const&>(value);
        return value;
    }

    T operator=(std::invocable<T&> auto&& modify) const requires (!std::is_const_v<T>)
    {
        T value = get();
        modify(value);
        *this = value;
        return value;
    }

    operator T() const
    {
        return get();
    }

    const T operator*() const
    {
        return get();
    }
};

template < uintptr_t const& Base, RegisterType T, uint32_t Count, uint32_t Span, uint32_t Offset = static_cast<uint32_t>(-1) >
struct RegisterArrayProxy
{
    RegisterProxy<Base, T> operator[](uint32_t index) const
    {
        // assert(index >= Count)
        return RegisterProxy<Base, T>{ Offset + index * (Span * uint32_t{ sizeof(T) }) };
    }
};

template < uintptr_t const& Base, RegisterType T, uint32_t Count, uint32_t Span >
struct RegisterArrayProxy<Base, T, Count, Span, static_cast<uint32_t>(-1)>
{
    uint32_t Offset;

    constexpr explicit RegisterArrayProxy(uint32_t offs) : Offset(offs) {}

    RegisterProxy<Base, T> operator[](uint32_t index) const
    {
        // assert(index >= Count)
        return RegisterProxy<Base, T>{ Offset + index * (Span * uint32_t{ sizeof(T) }) };
    }
};



template < typename T, uint32_t Offset = static_cast<uint32_t>(-1) > requires (sizeof(T) == 4) using BaseRegisterProxy = RegisterProxy<Mmio::Base   , T, Offset>;
template < typename T, uint32_t Offset = static_cast<uint32_t>(-1) > requires (sizeof(T) == 4) using QA7RegisterProxy  = RegisterProxy<Mmio::QA7Base, T, Offset>;

template < typename T, uint32_t Count, uint32_t Span, uint32_t Offset = static_cast<uint32_t>(-1) > requires (sizeof(T) == 4) using BaseRegisterArrayProxy = RegisterArrayProxy<Mmio::Base   , T, Count, Span, Offset>;
template < typename T, uint32_t Count, uint32_t Span, uint32_t Offset = static_cast<uint32_t>(-1) > requires (sizeof(T) == 4) using QA7RegisterArrayProxy  = RegisterArrayProxy<Mmio::QA7Base, T, Count, Span, Offset>;

}
// namespace Mmio
