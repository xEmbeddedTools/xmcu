#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <type_traits>

// hkm
#include <xmcu/assertion.hpp>

namespace xmcu {
template<typename Value_t, Value_t factor_t> class Duration
{
public:
    using Value_type = Value_t;
    static constexpr Value_t factor = factor_t;
    constexpr Duration() = default;
    constexpr Duration(Duration&&) = default;
    constexpr Duration(const Duration&) = default;
    constexpr Duration(Value_t a_v)
        : v(a_v)
    {
    }
    template<Value_t rhs_factor, std::enable_if_t<(factor_t < rhs_factor), bool> = true>
    constexpr Duration(const Duration<Value_t, rhs_factor>& a_v)
        : v(a_v.get() * rhs_factor)
    {
    }
    template<Value_t rhs_factor, std::enable_if_t<(factor_t < rhs_factor), bool> = true>
    explicit constexpr Duration(Duration<Value_t, rhs_factor>&& a_v)
        : v(a_v.get() * rhs_factor)
    {
    }

    constexpr Duration& operator=(const Duration&) = default;
    constexpr Duration& operator=(Duration&&) = default;
    template<Value_t rhs_factor> constexpr bool operator==(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t == a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor> constexpr bool operator!=(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t != a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor> constexpr bool operator>=(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t >= a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor> constexpr bool operator<=(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t <= a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor> constexpr bool operator>(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t > a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor> constexpr bool operator<(Duration<Value_t, rhs_factor> a_rhs)
    {
        return this->v * factor_t < a_rhs.get() * rhs_factor;
    }
    template<Value_t rhs_factor, std::enable_if_t<(factor_t < rhs_factor), bool> = true>
    constexpr Duration<Value_t, factor>& operator=(const Duration<Value_t, rhs_factor>& a_rhs)
    {
        this->v = a_rhs.v * rhs_factor;
        return *this;
    }
    template<Value_t rhs_factor, std::enable_if_t<(factor_t < rhs_factor), bool> = true>
    constexpr Duration<Value_t, factor_t>& operator=(Duration<Value_t, rhs_factor>&& a_rhs)
    {
        this->v = a_rhs.v * rhs_factor;
        return *this;
    }
    constexpr Duration& operator++()
    {
        this->v++;
        return *this;
    }
    constexpr Duration operator++(int)
    {
        return Duration(this->v++);
    }
    constexpr Duration& operator--()
    {
        hkm_assert(this->v != 0x0u);

        this->v--;
        return *this;
    }
    constexpr Duration operator--(int)
    {
        hkm_assert(this->v != 0x0u);

        return Duration(this->v--);
    }
    constexpr Duration& operator+=(const Duration& a_rhs)
    {
        this->v += a_rhs.v;
        return *this;
    }
    constexpr Duration& operator-=(const Duration& a_rhs)
    {
        hkm_assert(this->v >= a_rhs.v);

        this->v -= a_rhs.v;
        return *this;
    }
    constexpr Duration& operator*=(Value_t a_rhs)
    {
        this->v *= a_rhs;
        return *this;
    }
    constexpr Duration& operator/=(Value_t a_rhs)
    {
        hkm_assert(a_rhs > 0x0u);
        // we don't use negative Durations?
        // compilation error. . . :
        // request for member 'get' in 'a_rhs', which is of non-class type 'long long unsigned int'
        // hkm_assert(a_rhs.get() != 0x0u);

        this->v /= a_rhs;
        return *this;
    }
    constexpr Duration& operator%=(Value_t a_rhs)
    {
        this->v %= a_rhs;
        return *this;
    }
    constexpr Duration& operator%=(const Duration& a_rhs)
    {
        this->v %= a_rhs.v;
        return *this;
    }

    constexpr Value_t get() const
    {
        return this->v;
    }

    template<typename Ret_type_t> Ret_type_t get_in() const = delete;

private:
    Value_t v = static_cast<Value_t>(0);
};

using Microseconds = Duration<std::uint64_t, 1u>;
using Milliseconds = Duration<std::uint64_t, 1000u>;
using Seconds = Duration<std::uint64_t, 1000000u>;

template<> template<> inline Microseconds Milliseconds::get_in() const
{
    return this->v * 1000u;
}

template<> template<> inline Milliseconds Seconds::get_in() const
{
    return this->v * 1000u;
}

template<> template<> inline Seconds Milliseconds::get_in() const
{
    return this->v / 1000u;
}

inline Microseconds operator-(Microseconds a_lhs, Microseconds a_rhs)
{
    hkm_assert(a_lhs.get() >= a_rhs.get());

    return { a_lhs.get() - a_rhs.get() };
}
inline Microseconds operator-(Microseconds a_lhs, Milliseconds a_rhs)
{
    hkm_assert(a_lhs.get() >= a_rhs.get_in<Microseconds>().get());

    return { a_lhs - a_rhs.get_in<Microseconds>() };
}
inline Microseconds operator-(Milliseconds a_lhs, Microseconds a_rhs)
{
    hkm_assert(a_lhs.get_in<Microseconds>().get() >= a_rhs.get());

    return { a_lhs.get_in<Microseconds>() - a_rhs };
}
inline Milliseconds operator-(Milliseconds a_lhs, Milliseconds a_rhs)
{
    hkm_assert(a_lhs.get() >= a_rhs.get());

    return { a_lhs.get() - a_rhs.get() };
}
inline Milliseconds operator-(Seconds a_lhs, Milliseconds a_rhs)
{
    hkm_assert(a_lhs.get_in<xmcu::Milliseconds>().get() >= a_rhs.get());

    return { a_lhs.get_in<xmcu::Milliseconds>() - a_rhs };
}
inline Milliseconds operator-(Milliseconds a_lhs, Seconds a_rhs)
{
    hkm_assert(a_lhs.get() >= a_rhs.get_in<xmcu::Milliseconds>().get());

    return { a_lhs - a_rhs.get_in<xmcu::Milliseconds>() };
}
inline Seconds operator-(Seconds a_lhs, Seconds a_rhs)
{
    hkm_assert(a_lhs.get() >= a_rhs.get());

    return { a_lhs.get() - a_rhs.get() };
}

inline Microseconds operator+(Microseconds a_lhs, Microseconds a_rhs)
{
    return { a_lhs.get() + a_rhs.get() };
}
inline Microseconds operator+(Microseconds a_lhs, Milliseconds a_rhs)
{
    return { a_lhs + a_rhs.get_in<Microseconds>() };
}
inline Microseconds operator+(Milliseconds a_lhs, Microseconds a_rhs)
{
    return { a_lhs.get_in<Microseconds>() + a_rhs };
}
inline Milliseconds operator+(Milliseconds a_lhs, Milliseconds a_rhs)
{
    return { a_lhs.get() + a_rhs.get() };
}
inline Milliseconds operator+(Seconds a_lhs, Milliseconds a_rhs)
{
    return { a_lhs.get_in<xmcu::Milliseconds>() + a_rhs };
}
inline Milliseconds operator+(Milliseconds a_lhs, Seconds a_rhs)
{
    return { a_lhs + a_rhs.get_in<xmcu::Milliseconds>() };
}
inline Seconds operator+(Seconds a_lhs, Seconds a_rhs)
{
    return { a_lhs.get() + a_rhs.get() };
}

inline Microseconds operator*(Microseconds a_lhs, Microseconds::Value_type a_rhs)
{
    return { a_lhs.get() * a_rhs };
}
inline Microseconds operator*(Microseconds::Value_type a_lhs, Microseconds a_rhs)
{
    return { a_lhs * a_rhs.get() };
}

inline Milliseconds operator*(Milliseconds a_lhs, Milliseconds::Value_type a_rhs)
{
    return { a_lhs.get() * a_rhs };
}
inline Milliseconds operator*(Milliseconds::Value_type a_lhs, Milliseconds a_rhs)
{
    return { a_lhs * a_rhs.get() };
}

inline Seconds operator*(Seconds a_lhs, Seconds::Value_type a_rhs)
{
    return { a_lhs.get() * a_rhs };
}
inline Seconds operator*(Seconds::Value_type a_lhs, Seconds a_rhs)
{
    return { a_lhs * a_rhs.get() };
}
} // namespace xmcu

constexpr inline xmcu::Microseconds operator"" _us(unsigned long long a_value)
{
    return { a_value };
}
constexpr inline xmcu::Milliseconds operator"" _ms(unsigned long long a_value)
{
    return { a_value };
}
constexpr inline xmcu::Seconds operator"" _s(unsigned long long a_value)
{
    return { a_value };
}
