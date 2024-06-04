#pragma once

/*
 */

// std
#include <cstdint>

// xmcu
#include <xmcu/Non_constructible.hpp>
#include <xmcu/various.hpp>

namespace xmcu {
class bit_flag : private Non_constructible
{
public:
    template<typename Register_t, typename Flag_t> constexpr static bool is(Register_t a_register, Flag_t a_flag)
    {
        return a_flag == (a_register & a_flag);
    }

    template<typename Register_t, typename Mask_t> constexpr static Mask_t get(Register_t a_register, Mask_t a_mask)
    {
        return (a_register & a_mask);
    }

    template<typename Register_t, typename Flag_t> constexpr static void set(Register_t* a_p_register, Flag_t a_flag)
    {
        (*a_p_register) = (*a_p_register) | a_flag;
    }

    template<typename Register_t, typename Clear_mask_t, typename Flag_t>
    constexpr static void set(Register_t* a_p_register, Clear_mask_t a_clear_mask, Flag_t a_set_flag)
    {
        // HERE BE DRAGONS - storing `const Register_t temp = *a_p_register` in separate line optimized away setting
        // GPIO MODER when initializing UART under -Os on L0
        (*a_p_register) = ((*a_p_register & (~a_clear_mask)) | a_set_flag);
    }

    template<typename Register_t, typename Flag_t> constexpr static void clear(Register_t* a_p_register, Flag_t a_flag)
    {
        (*a_p_register) = (*a_p_register) & ~a_flag;
    }
};
} // namespace xmcu
