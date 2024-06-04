/**/

// this
#include <xmcu/soc/ST/m0/stm32l0/rm0451/sources/msi.hpp>

// xmcu
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/wait_until.hpp>

namespace {
constexpr std::uint32_t msi_frequency_hz_lut[] = { 100_kHz, 200_kHz, 400_kHz, 800_kHz, 1_MHz,  2_MHz,
                                                   4_MHz,   8_MHz,   16_MHz,  24_MHz,  32_MHz, 48_MHz };
}

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace sources {
using namespace xmcu;
using namespace xmcu::soc::m0::stm32l0::rm0451::utils;

void msi::enable(Frequency a_frequency)
{
    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);
    wait_until::all_bits_are_cleared(RCC->CR, RCC_CR_MSIRDY);

    bit_flag::set(&(RCC->ICSCR), RCC_ICSCR_MSIRANGE, static_cast<std::uint32_t>(a_frequency));
    bit_flag::set(&(RCC->CR), RCC_CR_MSION);

    wait_until::all_bits_are_set(RCC->CR, RCC_CR_MSIRDY);

    bit_flag::clear(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);

    wait_until::all_bits_are_set(RCC->CR, RCC_CR_MSIRDY);
}

bool msi::enable(Frequency a_frequency, Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);
    if (true == wait_until::all_bits_are_cleared(
                    RCC->CR, RCC_CR_MSIRDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        bit_flag::set(&(RCC->ICSCR), RCC_ICSCR_MSIRANGE, static_cast<std::uint32_t>(a_frequency));
        bit_flag::set(&(RCC->CR), RCC_CR_MSION);

        if (true == wait_until::all_bits_are_set(
                        RCC->CR, RCC_CR_MSIRDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
        {
            bit_flag::clear(&(RCC->ICSCR), RCC_ICSCR_MSITRIM);

            return wait_until::all_bits_are_set(
                RCC->CR, RCC_CR_MSIRDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
        }
    }

    return false;
}

void msi::disable()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);
    wait_until::all_bits_are_cleared(RCC->CR, RCC_CR_MSIRDY);
}

bool msi::disable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::clear(&(RCC->CR), RCC_CR_MSION);
    return wait_until::all_bits_are_cleared(
        RCC->CR, RCC_CR_MSIRDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}

std::uint32_t msi::get_frequency_Hz()
{
    if (true == is_enabled())
    {
        return msi_frequency_hz_lut[bit_flag::get(RCC->ICSCR, RCC_ICSCR_MSIRANGE) >> RCC_ICSCR_MSIRANGE_Pos];
    }

    return 0u;
}
} // namespace sources
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
