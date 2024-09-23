#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// xmcu
#include <xmcu/non_constructible.hpp>

namespace xmcu {
class bit : private non_constructible
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

    class flag : private non_constructible
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

        template<typename Register_t, typename Flag_t>
        constexpr static void set(Register_t* a_p_register, Flag_t a_flag)
        {
            (*a_p_register) = (*a_p_register) | a_flag;
        }

        template<typename Register_t, typename Clear_mask_t, typename Flag_t>
        constexpr static void set(Register_t* a_p_register, Clear_mask_t a_clear_mask, Flag_t a_set_flag)
        {
            (*a_p_register) = ((*a_p_register & (~a_clear_mask)) | a_set_flag);
        }

        template<typename Register_t, typename Flag_t>
        constexpr static void clear(Register_t* a_p_register, Flag_t a_flag)
        {
            (*a_p_register) = (*a_p_register) & ~a_flag;
        }
    };
};
} // namespace xmcu
