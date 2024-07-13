#pragma once

/*
 */

// std
#include <cstdint>
#include <limits>
#include <type_traits>

// xmcu
#include <xmcu/non_constructible.hpp>

namespace xmcu {
struct various : private non_constructible
{
    template<typename Type_t> constexpr static Type_t get_enum_incorrect_value()
    {
        static_assert(true == std::is_enum<Type_t>::value);
        return static_cast<Type_t>(std::numeric_limits<std::underlying_type_t<Type_t>>::max());
    }

    // TODO: replace with std::to_underlying() when we migrate to cpp23 and gcc11+
    template<typename t_enum_type>
    constexpr static std::underlying_type_t<t_enum_type> to_underlying(t_enum_type a_enum)
    {
        return static_cast<std::underlying_type_t<t_enum_type>>(a_enum);
    }
};
} // namespace xmcu
