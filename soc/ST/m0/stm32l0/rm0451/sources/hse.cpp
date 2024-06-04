/**/

// this
#include <xmcu/soc/ST/m0/stm32l0/rm0451/sources/hse.hpp>

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/wait_until.hpp>

namespace {
constexpr std::uint32_t hse_control_unlock_key = 0xCAFECAFEu;
}

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace sources {
using namespace xmcu;
using namespace xmcu::soc::m0::stm32l0::rm0451::utils;

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

} // namespace sources
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
