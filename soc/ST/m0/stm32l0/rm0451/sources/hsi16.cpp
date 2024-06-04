/**/

// this
#include <xmcu/soc/ST/m0/stm32l0/rm0451/sources/hsi16.hpp>

// xmcu
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/wait_until.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace sources {
using namespace xmcu;
using namespace xmcu::soc::m0::stm32l0::rm0451::utils;

void hsi16::enable()
{
    bit_flag::set(&(RCC->CR), RCC_CR_HSION, RCC_CR_HSION);
    wait_until::all_bits_are_set(RCC->CR, RCC_CR_HSIRDY);
}
bool hsi16::enable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(RCC->CR), RCC_CR_HSION, RCC_CR_HSION);
    return wait_until::all_bits_are_set(
        RCC->CR, RCC_CR_HSIRDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}
void hsi16::disable()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_HSION);
    wait_until::all_bits_are_cleared(RCC->CR, RCC_CR_HSIRDY);
}
bool hsi16::disable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::clear(&(RCC->CR), RCC_CR_HSION);
    return wait_until::all_bits_are_cleared(
        RCC->CR, RCC_CR_HSIRDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}
} // namespace sources
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
