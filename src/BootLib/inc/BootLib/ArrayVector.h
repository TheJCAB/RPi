
#pragma once

#include <stdint.h>
#include <stddef.h>

#include <array>
#include <initializer_list>

namespace BootLib
{

template < typename T, size_t N >
struct ArrayVector
{
    constexpr ArrayVector() = default;
    constexpr ArrayVector(std::initializer_list<T> init)
    {
        for (auto const& value : init)
        {
            push_back(value);
        }
    }

    using iterator       = typename std::array<T, N>::iterator;
    using const_iterator = typename std::array<T, N>::const_iterator;

    constexpr iterator       begin()       { return data.begin(); }
    constexpr const_iterator begin() const { return data.begin(); }
    constexpr iterator       end()         { return data.begin() + count; }
    constexpr const_iterator end()   const { return data.begin() + count; }

    constexpr T& push_back(T const& value)
    {
        if (count < N)
        {
            return data[count++] = value;
        }
        else
        {
            return data[count - 1];
        }
    }

    constexpr void pop_back()
    {
        if (count > 0)
        {
            --count;
        }
    }

    constexpr size_t size    () const { return count; }
    constexpr bool   empty   () const { return count == 0; }
    constexpr size_t capacity() const { return N; }

    constexpr void clear() { count = 0; }

    constexpr T&       front()       { return data[0]; }
    constexpr T const& front() const { return data[0]; }

    constexpr T&       back()       { return data[count - 1]; }
    constexpr T const& back() const { return data[count - 1]; }

    constexpr T&       operator[](size_t index)       { return data[index]; }
    constexpr T const& operator[](size_t index) const { return data[index]; }

private:
    std::array<T, N> data{};
    size_t           count = 0;
};

}
// namespace BootLib
