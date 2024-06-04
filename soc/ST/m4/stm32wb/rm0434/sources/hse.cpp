/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hse.hpp>

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

namespace {
constexpr std::uint32_t hse_control_unlock_key = 0xCAFECAFEu;
}

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace sources {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::utils;

void hse::enable()
{
    bit_flag::set(&(RCC->CR), RCC_CR_HSEON);

    wait_until::all_bits_are_set(RCC->CR, RCC_CR_HSERDY);
}
bool hse::enable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(RCC->CR), RCC_CR_HSEON);

    return wait_until::all_bits_are_set(
        RCC->CR, RCC_CR_HSERDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}
void hse::disable()
{
    bit_flag::set(&(RCC->CR), RCC_CR_HSEON);
    wait_until::all_bits_are_cleared(RCC->CR, RCC_CR_HSERDY);
}
bool hse::disable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(RCC->CR), RCC_CR_HSEON);
    return wait_until::all_bits_are_cleared(
        RCC->CR, RCC_CR_HSERDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}
void hse::tune::set(Capacitor_tuning a_capacitor_tuning)
{
    RCC->HSECR = hse_control_unlock_key;
    bit_flag::set(&(RCC->HSECR), RCC_HSECR_HSETUNE, a_capacitor_tuning.get() << RCC_HSECR_HSETUNE_Pos);
}
void hse::tune::set(Current_control_max_limit a_current_control_max_limit)
{
    RCC->HSECR = hse_control_unlock_key;
    bit_flag::set(&(RCC->HSECR), RCC_HSECR_HSEGMC, static_cast<std::uint32_t>(a_current_control_max_limit));
}
void hse::tune::set(Amplifier_threshold a_amplifier_threshold)
{
    RCC->HSECR = hse_control_unlock_key;
    bit_flag::set(&(RCC->HSECR), RCC_HSECR_HSES, static_cast<std::uint32_t>(a_amplifier_threshold));
}
} // namespace sources
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif