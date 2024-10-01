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
    template<typename Register_t, typename Index_t>
    [[nodiscard]] constexpr static bool is(Register_t register_a, Index_t index_a)
    {
        const Register_t flag = static_cast<Register_t>(0x1u) << index_a;
        return flag == (register_a & flag);
    }

    template<typename Register_t, typename Mask_t>
    [[nodiscard]] constexpr static bool is_any(Register_t register_a, Mask_t mask_a)
    {
        return static_cast<Mask_t>(0u) != (register_a & mask_a);
    }

    template<typename Register_t, typename Index_t> constexpr static void set(Register_t* p_register_a, Index_t index_a)
    {
        (*p_register_a) = (*p_register_a) | (0x1u << index_a);
    }

    template<typename Register_t, typename Index_t>
    constexpr static void clear(Register_t* p_register_a, Index_t index_a)
    {
        (*p_register_a) = (*p_register_a) & ~(0x1u << index_a);
    }

    template<typename Register_t, typename Index_t>
    constexpr static void toggle(Register_t* p_register_a, Index_t index_a)
    {
        (*p_register_a) = (*p_register_a) ^ (0x1u << index_a);
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
