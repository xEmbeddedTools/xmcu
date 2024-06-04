/**/

// this
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/wait_until.hpp>

// xmcu
#include <xmcu/bit.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/tick_counter.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace utils {
using namespace xmcu;

void wait_until::all_bits_are_set(volatile const std::uint32_t& a_register, uint32_t a_mask)
{
    while (false == bit_flag::is(a_register, a_mask)) continue;
}

void wait_until::any_bit_is_set(volatile const std::uint32_t& a_register, uint32_t a_mask)
{
    while (false == bit::is_any(a_register, a_mask)) continue;
}

void wait_until::masked_bits_are_set(volatile const std::uint32_t& a_register, uint32_t a_mask, uint32_t a_value)
{
    while (bit_flag::get(a_register, a_mask) != a_value) continue;
}

void wait_until::all_bits_are_cleared(volatile const std::uint32_t& a_register, uint32_t a_mask)
{
    while (false == bit_flag::is(~a_register, a_mask)) continue;
}

void wait_until::any_bit_is_cleared(volatile const std::uint32_t& a_register, uint32_t a_mask)
{
    while (false == bit::is_any(~a_register, a_mask)) continue;
}

bool wait_until::all_bits_are_set(volatile const std::uint32_t& a_register, uint32_t a_mask, Milliseconds a_timeout)
{
    const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
    bool status                     = false;

    while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
    {
        status = bit_flag::is(a_register, a_mask);
    }

    return status;
}

bool wait_until::masked_bits_are_set(volatile const std::uint32_t& a_register,
                                     uint32_t a_mask,
                                     uint32_t a_value,
                                     Milliseconds a_timeout)
{
    const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
    bool status                     = false;

    while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
    {
        status = (bit_flag::get(a_register, a_mask) == a_value);
    }

    return status;
}

bool wait_until::any_bit_is_set(volatile const std::uint32_t& a_register, uint32_t a_mask, Milliseconds a_timeout)
{
    const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
    bool status                     = false;

    while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
    {
        status = bit::is_any(a_register, a_mask);
    }

    return status;
}

bool wait_until::all_bits_are_cleared(volatile const std::uint32_t& a_register, uint32_t a_mask, Milliseconds a_timeout)
{
    const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
    bool status                     = false;

    while (tick_counter<Milliseconds>::get() < timeout_end && false == status)
    {
        status = bit_flag::is(~a_register, a_mask);
    }

    return status;
}

bool wait_until::any_bit_is_cleared(volatile const std::uint32_t& a_register, uint32_t a_mask, Milliseconds a_timeout)
{
    const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();
    bool status                     = false;

    while (tick_counter<Milliseconds>::get() < timeout_end && status == false)
    {
        status = bit::is_any(~a_register, a_mask);
    }

    return status;
}

} // namespace utils
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
