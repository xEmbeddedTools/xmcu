/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lse.hpp>

// xmcu
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace sources {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::utils;

void lse::xtal::enable(Drive a_drive_config)
{
    bit_flag::set(&(PWR->CR1), PWR_CR1_DBP);
    wait_until::all_bits_are_set(PWR->CR1, PWR_CR1_DBP);
    bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSEDRV, static_cast<std::uint32_t>(a_drive_config));

    bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSEON);
    wait_until::all_bits_are_set(RCC->BDCR, RCC_BDCR_LSERDY);
}
bool lse::xtal::enable(Drive a_drive_config, Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(PWR->CR1), PWR_CR1_DBP);
    if (true == wait_until::all_bits_are_set(
                    PWR->CR1, PWR_CR1_DBP, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSEDRV, static_cast<std::uint32_t>(a_drive_config));

        bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSEON);
        return wait_until::all_bits_are_set(
            RCC->BDCR, RCC_BDCR_LSERDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
    }

    return false;
}
void lse::bypass::enable()
{
    bit_flag::set(&(PWR->CR1), PWR_CR1_DBP);
    wait_until::all_bits_are_set(PWR->CR1, PWR_CR1_DBP);
    bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSEBYP);

    bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSEON);
    wait_until::all_bits_are_set(RCC->BDCR, RCC_BDCR_LSERDY);
}
bool lse::bypass::enable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(PWR->CR1), PWR_CR1_DBP);
    if (true == wait_until::all_bits_are_set(
                    PWR->CR1, PWR_CR1_DBP, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSEBYP);

        bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSEON);
        return wait_until::all_bits_are_set(
            RCC->BDCR, RCC_BDCR_LSERDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
    }

    return false;
}
void lse::disable()
{
    bit_flag::clear(&(RCC->BDCR), RCC_BDCR_LSEON);
    wait_until::all_bits_are_cleared(RCC->BDCR, RCC_BDCR_LSERDY);
    bit_flag::clear(&(RCC->BDCR), RCC_BDCR_LSEBYP);
}
bool lse::disable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::clear(&(RCC->BDCR), RCC_BDCR_LSEON);
    if (true == wait_until::all_bits_are_cleared(
                    RCC->BDCR, RCC_BDCR_LSERDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSEBYP);
        return true;
    }

    return false;
}
} // namespace sources
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif