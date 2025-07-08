#pragma once

#include <stdint.h>

#include <type_traits>
#include <concepts>

// This file defines the MMIO (Memory-Mapped I/O) memory regions for the Raspberry Pi,
namespace BootLib
{

// All registers are 32-bit wide.
// We admit as a register any type that fits and is trivial to copy.
template < typename T >
concept RegisterType = (std::is_trivially_copyable_v<std::remove_const_t<T>>) && (sizeof(T) == sizeof(uint32_t));

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
template < RegisterType T, uint32_t Offset = 0 >
struct Register
{
    Register() = default;

    Register(Register&&) = delete;
    Register& operator=(Register&&) = delete;

    auto& RefUint32()       { return *(reinterpret_cast<uint32_t       volatile*>(this) + Offset / sizeof(uint32_t)); }
    auto& RefUint32() const { return *(reinterpret_cast<uint32_t const volatile*>(this) + Offset / sizeof(uint32_t)); }

    T get() const
    {
        uint32_t const result = RefUint32();
        return reinterpret_cast<T const&>(result);
    }

    T set(const T& value)
    {
        RefUint32() = reinterpret_cast<uint32_t const&>(value);
        return value;
    }

    T operator=(std::integral auto value) requires (sizeof(value) <= sizeof(uint32_t))
    {
        uint32_t v = static_cast<uint32_t>(value);
        RefUint32() = v;
        return reinterpret_cast<T const&>(v);
    }

    T operator=(const T& value)
    {
        RefUint32() = reinterpret_cast<uint32_t const&>(value);
        return value;
    }

    T operator=(std::invocable<T&> auto&& modify)
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

template < RegisterType T, uint32_t Offset, uint32_t Count, uint32_t Stride = 4 >
struct RegisterArray
{
    RegisterArray() = default;
    RegisterArray(RegisterArray&&) = delete;
    RegisterArray& operator=(RegisterArray&&) = delete;

    Register<T>& operator[](uint32_t index)
    {
        // assert(index >= Count)
        return *reinterpret_cast<Register<T>*>(reinterpret_cast<uint32_t*>(this) + (Offset + index * Stride) / sizeof(uint32_t));
    }
    Register<T> const& operator[](uint32_t index) const
    {
        // assert(index >= Count)
        return *reinterpret_cast<Register<T> const*>(reinterpret_cast<uint32_t const*>(this) + (Offset + index * Stride) / sizeof(uint32_t));
    }
};

}
// namespace BootLib
