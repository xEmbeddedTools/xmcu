#pragma once

/*
 */

// std
#include <cstdint>

namespace xmcu {
using Frequency = std::uint32_t;
}

constexpr xmcu::Frequency operator"" _Hz(std::uint64_t a_value)
{
    return a_value;
}

constexpr xmcu::Frequency operator"" _kHz(std::uint64_t a_value)
{
    return a_value * 1000u;
}

constexpr xmcu::Frequency operator"" _MHz(std::uint64_t a_value)
{
    return a_value * 1000000u;
}
