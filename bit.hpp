#pragma once

/*
 */

// std
#include <cstdint>

// xmcu
#include <xmcu/Non_constructible.hpp>

namespace xmcu {
class bit : private Non_constructible
{
public:
    template<typename Register_t> constexpr static bool is(Register_t a_register, uint8_t a_index)
    {
        const Register_t flag = static_cast<Register_t>(0x1u) << a_index;
        return flag == (a_register & flag);
    }

    template<typename Register_t, typename Mask_t> constexpr static bool is_any(Register_t a_register, Mask_t a_mask)
    {
        return static_cast<Register_t>(0u) != (a_register & a_mask);
    }

    template<typename Register_t> constexpr static void set(Register_t* a_p_register, uint8_t a_index)
    {
        (*a_p_register) = (*a_p_register) | (static_cast<Register_t>(0x1u) << a_index);
    }

    template<typename Register_t> constexpr static void clear(Register_t* a_p_register, uint8_t a_index)
    {
        (*a_p_register) = (*a_p_register) & ~(static_cast<Register_t>(0x1u) << a_index);
    }

    template<typename Register_t> constexpr static void toggle(Register_t* a_p_register, uint8_t a_index)
    {
        (*a_p_register) = (*a_p_register) ^ (static_cast<Register_t>(0x1u) << a_index);
    }
};
} // namespace xmcu
