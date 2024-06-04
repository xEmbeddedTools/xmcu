/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hsi16.hpp>

// xmcu
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace sources {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::utils;

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
    wait_until::all_bits_are_set(RCC->CR, RCC_CR_HSIRDY);
}
bool hsi16::disable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::clear(&(RCC->CR), RCC_CR_HSION);
    return wait_until::all_bits_are_set(
        RCC->CR, RCC_CR_HSIRDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}
} // namespace sources
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif