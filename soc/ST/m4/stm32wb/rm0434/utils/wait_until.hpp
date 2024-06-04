#pragma once

/**/

// std
#include <cstdint>

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace utils {
struct wait_until : private Non_constructible
{
    template<typename Register_t, typename Mask_t>
    static void all_bits_are_set(volatile const Register_t& a_register, Mask_t a_mask)
    {
        while (false == bit_flag::is(a_register, a_mask)) continue;
    }
    template<typename Register_t, typename Mask_t>
    static void any_bit_is_set(volatile const Register_t& a_register, Mask_t a_mask)
    {
        while (false == bit::is_any(a_register, a_mask)) continue;
    }
    template<typename Register_t, typename Mask_t>
    static void all_bits_are_cleared(volatile const Register_t& a_register, Mask_t a_mask)
    {
        while (false == bit_flag::is(~a_register, a_mask)) continue;
    }
    template<typename Register_t, typename Mask_t>
    static void any_bit_is_cleared(volatile const std::uint32_t& a_register, uint32_t a_mask)
    {
        while (false == bit::is_any(~a_register, a_mask)) continue;
    }
    template<typename Register_t, typename Mask_t>
    static bool any_bit_is_set(volatile const Register_t& a_register, Mask_t a_mask, Milliseconds a_timeout)
    {
        const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
        bool status = false;

        while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
        {
            status = bit::is_any(a_register, a_mask);
        }

        return status;
    }
    template<typename Register_t, typename Mask_t> static bool
    all_bits_are_cleared(volatile const Register_t& a_register, Mask_t a_mask, Milliseconds a_timeout)
    {
        const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
        bool status = false;

        while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
        {
            status = bit_flag::is(~a_register, a_mask);
        }

        return status;
    }
    template<typename Register_t, typename Mask_t>
    static bool any_bit_is_cleared(volatile const Register_t& a_register, Mask_t a_mask, Milliseconds a_timeout)
    {
        const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
        bool status = false;

        while (tick_counter<Milliseconds>::get() < timeout_end && status == false)
        {
            status = bit::is_any(~a_register, a_mask);
        }

        return status;
    }
    template<typename Register_t, typename Mask_t>
    static bool all_bits_are_set(volatile const Register_t& a_register, Mask_t a_mask, Milliseconds a_timeout)
    {
        const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
        bool status = false;

        while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
        {
            status = bit_flag::is(a_register, a_mask);
        }

        return status;
    }
};
} // namespace utils
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu