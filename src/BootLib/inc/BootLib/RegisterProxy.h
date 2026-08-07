#pragma once

#include <stdint.h>
#include <stddef.h>

#include <type_traits>
#include <concepts>

// This file defines the MMIO (Memory-Mapped I/O) memory regions for the Raspberry Pi,
namespace BootLib
{

// All registers can be any size from 8 bits to 64 (only powers of 2).
// We admit as a register any type that fits and is trivial to copy.
template < typename T >
concept RegisterType = (std::is_trivially_copyable_v<std::remove_const_t<T>>) && (sizeof(T) <= sizeof(uint64_t));

template < size_t Size > struct RawRegisterTypeT;
template <> struct RawRegisterTypeT<1> { using type =  uint8_t; };
template <> struct RawRegisterTypeT<2> { using type = uint16_t; };
template <> struct RawRegisterTypeT<4> { using type = uint32_t; };
template <> struct RawRegisterTypeT<8> { using type = uint64_t; };
template < size_t Size > using RawRegisterType = typename RawRegisterTypeT<Size>::type;

/// @brief Proxy template for accessing memory-mapped I/O (MMIO) registers.
///
/// This template provides a type-safe and convenient interface for reading and writing
/// 8- to 64-bit hardware registers mapped into memory.
/// The address of the Register class must be the address in memory of the register itself.
/// The optional offset allows for multiple registers in a region to be grouped as a convenient union.
/// The template enforces that only trivially copyable types of 8 to 64 bits can be used,
/// ensuring safe and predictable register access.
///
/// Usage:
///   - Instantiate Register at the MMIO location withthe appropriate register type and optional offset.
///   - Use assignment operators to write values or modify the register in-place.
///   - Use the conversion operator or dereference to read the current register value.
///
/// Example:
///   uintptr_t Base = 0x3F00'0000u;         // Raspberry Pi 3 peripheral Mmio registers
///   Register<uint32_t>& reg = *reinterpret_cast<Register<uint32_t>*>(Base + 0x10); // Access register at offset 0x10 from Base
///   auto& reg2 = *reinterpret_cast<Register<uint32_t, 0x10>*>(Base); // The offset can be put in the type
///   reg = 0x12345678;                      // Write a whole value to the register
///   uint32_t val = reg;                    // Read the register. Note: `reg` is a proxy of sorts so `auto` won't do.
///   reg = [](auto& v) { v |= 0x1; };       // Read the register, modify the value then write it back
///
/// @tparam T      Register type (must conform to the RegisterType concept).
/// @tparam Offset Optional offset from the register proxy's location.
///
template < RegisterType T, uint32_t Offset = 0 >
struct Register
{
    using Raw = RawRegisterType<sizeof(T)>;

    Register() = delete;

    Register(Register&&) = delete;
    Register& operator=(Register&&) = delete;

    inline auto& RefRaw()       { return *(reinterpret_cast<Raw       volatile*>(this) + Offset / sizeof(Raw)); }
    inline auto& RefRaw() const { return *(reinterpret_cast<Raw const volatile*>(this) + Offset / sizeof(Raw)); }

    inline T get() const
    {
        Raw const result = RefRaw();
        return reinterpret_cast<T const&>(result);
    }

    inline T set(const T& value) requires (!std::is_const_v<T>)
    {
        RefRaw() = reinterpret_cast<Raw const&>(value);
        return value;
    }

    inline T operator=(std::integral auto value) requires (!std::is_const_v<T> && sizeof(value) <= sizeof(Raw))
    {
        Raw v = static_cast<Raw>(value);
        RefRaw() = v;
        return reinterpret_cast<T const&>(v);
    }

    inline T operator=(const T& value) requires (!std::is_const_v<T>)
    {
        RefRaw() = reinterpret_cast<Raw const&>(value);
        return value;
    }

    inline T operator=(std::invocable<T&> auto&& modify) requires (!std::is_const_v<T>)
    {
        T value = get();
        modify(value);
        *this = value;
        return value;
    }

    inline operator T() const { return get(); }
    inline const T operator*() const { return get(); }


    inline auto operator->() const
    {
        struct DereferenceProxy
        {
            T const* operator->() { return &value; }
            T const value;
        };
        return DereferenceProxy{ .value = get() };
    }

    inline T operator&=(T const& value) requires (!std::is_const_v<T>) { return set(get() & value); }
    inline T operator|=(T const& value) requires (!std::is_const_v<T>) { return set(get() | value); }
    inline T operator^=(T const& value) requires (!std::is_const_v<T>) { return set(get() ^ value); }
    inline T operator+=(T const& value) requires (!std::is_const_v<T>) { return set(get() + value); }
    inline T operator-=(T const& value) requires (!std::is_const_v<T>) { return set(get() - value); }
};

template < RegisterType T, uint32_t Offset, uint32_t Count, uint32_t Stride = sizeof(T) >
struct RegisterArray
{
    using Raw = RawRegisterType<sizeof(T)>;

    RegisterArray() = delete;
    RegisterArray(RegisterArray&&) = delete;
    RegisterArray& operator=(RegisterArray&&) = delete;

    Register<T>& operator[](uint32_t index)
    {
        // assert(index >= Count)
        return *reinterpret_cast<Register<T>*>(reinterpret_cast<Raw*>(this) + (Offset + index * Stride) / sizeof(Raw));
    }
    Register<T> const& operator[](uint32_t index) const
    {
        // assert(index >= Count)
        return *reinterpret_cast<Register<T> const*>(reinterpret_cast<Raw const*>(this) + (Offset + index * Stride) / sizeof(Raw));
    }
};

}
// namespace BootLib
