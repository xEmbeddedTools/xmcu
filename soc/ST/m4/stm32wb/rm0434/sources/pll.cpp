/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/pll.hpp>

// xmcu
#include <xmcu/bit.hpp>
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/hsem/hsem.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::sources;
using namespace xmcu::soc::m4::stm32wb::utils;

void enable_PLL(std::uint32_t a_source,
                pll::M a_M,
                Limited<std::uint32_t, 8u, 86u> a_N,
                const pll::r::Enable_config& a_R,
                const pll::q::Enable_config& a_Q,
                const pll::p::Enable_config& a_P)
{
    hkm_assert(0x0u != a_source);
    hkm_assert(various::get_enum_incorrect_value<pll::r::Enable_config::Divider>() != a_R.divider);
    hkm_assert(various::get_enum_incorrect_value<pll::r::Enable_config::Output>() != a_R.output);

    hkm_assert(various::get_enum_incorrect_value<pll::q::Enable_config::Divider>() != a_Q.divider);
    hkm_assert(various::get_enum_incorrect_value<pll::q::Enable_config::Output>() != a_Q.output);

    hkm_assert(various::get_enum_incorrect_value<pll::p::Enable_config::Output>() != a_P.output);

    RCC->PLLCFGR = (a_N << RCC_PLLCFGR_PLLN_Pos) | a_source | static_cast<std::uint32_t>(a_M) |
                   static_cast<std::uint32_t>(a_R.divider) | static_cast<std::uint32_t>(a_R.output) |
                   static_cast<std::uint32_t>(a_Q.divider) | static_cast<std::uint32_t>(a_Q.output) |
                   (a_P.divider - 1) << RCC_PLLCFGR_PLLP_Pos | static_cast<std::uint32_t>(a_P.output);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLON);
    wait_until::all_bits_are_set(RCC->CR, RCC_CR_PLLRDY);
}

bool enable_PLL(std::uint32_t a_source,
                pll::M a_M,
                Limited<std::uint32_t, 8u, 86u> a_N,
                const pll::r::Enable_config& a_R,
                const pll::q::Enable_config& a_Q,
                const pll::p::Enable_config& a_P,
                Milliseconds a_timeout)
{
    hkm_assert(0x0u != a_source);
    hkm_assert(various::get_enum_incorrect_value<pll::r::Enable_config::Divider>() != a_R.divider);
    hkm_assert(various::get_enum_incorrect_value<pll::r::Enable_config::Output>() != a_R.output);

    hkm_assert(various::get_enum_incorrect_value<pll::q::Enable_config::Divider>() != a_Q.divider);
    hkm_assert(various::get_enum_incorrect_value<pll::q::Enable_config::Output>() != a_Q.output);

    hkm_assert(various::get_enum_incorrect_value<pll::p::Enable_config::Output>() != a_P.output);

    std::uint64_t start = tick_counter<Milliseconds>::get();

    RCC->PLLCFGR = (a_N << RCC_PLLCFGR_PLLN_Pos) | a_source | static_cast<std::uint32_t>(a_M) |
                   static_cast<std::uint32_t>(a_R.divider) | static_cast<std::uint32_t>(a_R.output) |
                   static_cast<std::uint32_t>(a_Q.divider) | static_cast<std::uint32_t>(a_Q.output) |
                   (a_P.divider - 1) << RCC_PLLCFGR_PLLP_Pos | static_cast<std::uint32_t>(a_P.output);

    bit_flag::set(&(RCC->CR), RCC_CR_PLLON);
    return wait_until::all_bits_are_set(
        RCC->CR, RCC_CR_PLLRDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}

std::uint32_t calculate_PLL_channel_frequency_Hz(std::uint32_t a_div)
{
    const std::uint32_t m = (bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1u;
    const std::uint32_t n = bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;

    switch (bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC))
    {
        case RCC_PLLCFGR_PLLSRC_0: {
            hkm_assert(true == msi::is_enabled());
            return ((msi::get_frequency_Hz() / m) * n) / a_div;
        }

        case RCC_PLLCFGR_PLLSRC_1: {
            hkm_assert(true == hsi16::is_enabled());
            return ((hsi16::get_frequency_Hz() / m) * n) / a_div;
        }

        case RCC_PLLCFGR_PLLSRC_0 | RCC_PLLCFGR_PLLSRC_1: {
            hkm_assert(true == hse::is_enabled());
            if (false == bit_flag::is(RCC->CR, RCC_CR_HSEPRE))
            {
                return ((hse::get_frequency_Hz() / m) * n) / a_div;
            }
            else
            {
                return (((hse::get_frequency_Hz() / 2u) / m) * n) / a_div;
            }
        }
        break;
    }

    return 0;
}

} // namespace

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace sources {
using namespace xmcu;

template<> void pll::enable<msi>(M a_M,
                                 Limited<std::uint32_t, 8u, 86u> a_N,
                                 const r::Enable_config& a_R,
                                 const q::Enable_config& a_Q,
                                 const p::Enable_config& a_P)
{
    enable_PLL(RCC_PLLCFGR_PLLSRC_0, a_M, a_N, a_R, a_Q, a_P);
}
template<> void pll::enable<hsi16>(M a_M,
                                   Limited<std::uint32_t, 8u, 86u> a_N,
                                   const r::Enable_config& a_R,
                                   const q::Enable_config& a_Q,
                                   const p::Enable_config& a_P)
{
    enable_PLL(RCC_PLLCFGR_PLLSRC_1, a_M, a_N, a_R, a_Q, a_P);
}
template<> void pll::enable<hse, hse::Prescaler::_1>(M a_M,
                                                     Limited<std::uint32_t, 8u, 86u> a_N,
                                                     const r::Enable_config& a_R,
                                                     const q::Enable_config& a_Q,
                                                     const p::Enable_config& a_P)
{
    bit_flag::clear(&(RCC->CR), RCC_CR_HSEPRE);
    enable_PLL(RCC_PLLCFGR_PLLSRC_0 | RCC_PLLCFGR_PLLSRC_1, a_M, a_N, a_R, a_Q, a_P);
}
template<> void pll::enable<hse, hse::Prescaler::_2>(M a_M,
                                                     Limited<std::uint32_t, 8u, 86u> a_N,
                                                     const r::Enable_config& a_R,
                                                     const q::Enable_config& a_Q,
                                                     const p::Enable_config& a_P)
{
    bit_flag::set(&(RCC->CR), RCC_CR_HSEPRE);
    enable_PLL(RCC_PLLCFGR_PLLSRC_0 | RCC_PLLCFGR_PLLSRC_1, a_M, a_N, a_R, a_Q, a_P);
}
template<> bool pll::enable<msi>(M a_M,
                                 Limited<std::uint32_t, 8u, 86u> a_N,
                                 const r::Enable_config& a_R,
                                 const q::Enable_config& a_Q,
                                 const p::Enable_config& a_P,
                                 Milliseconds a_timeout)
{
    return enable_PLL(RCC_PLLCFGR_PLLSRC_0, a_M, a_N, a_R, a_Q, a_P, a_timeout);
}
template<> bool pll::enable<hsi16>(M a_M,
                                   Limited<std::uint32_t, 8u, 86u> a_N,
                                   const r::Enable_config& a_R,
                                   const q::Enable_config& a_Q,
                                   const p::Enable_config& a_P,
                                   Milliseconds a_timeout)
{
    return enable_PLL(RCC_PLLCFGR_PLLSRC_1, a_M, a_N, a_R, a_Q, a_P, a_timeout);
}
template<> bool pll::enable<hse, hse::Prescaler::_1>(M a_M,
                                                     Limited<std::uint32_t, 8u, 86u> a_N,
                                                     const r::Enable_config& a_R,
                                                     const q::Enable_config& a_Q,
                                                     const p::Enable_config& a_P,
                                                     Milliseconds a_timeout)
{
    bit_flag::clear(&(RCC->CR), RCC_CR_HSEPRE);
    return enable_PLL(RCC_PLLCFGR_PLLSRC_0 | RCC_PLLCFGR_PLLSRC_1, a_M, a_N, a_R, a_Q, a_P, a_timeout);
}
template<> bool pll::enable<hse, hse::Prescaler::_2>(M a_M,
                                                     Limited<std::uint32_t, 8u, 86u> a_N,
                                                     const r::Enable_config& a_R,
                                                     const q::Enable_config& a_Q,
                                                     const p::Enable_config& a_P,
                                                     Milliseconds a_timeout)
{
    bit_flag::set(&(RCC->CR), RCC_CR_HSEPRE);
    return enable_PLL(RCC_PLLCFGR_PLLSRC_0 | RCC_PLLCFGR_PLLSRC_1, a_M, a_N, a_R, a_Q, a_P, a_timeout);
}
void pll::disable()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_PLLON);

    sai1::disable();

    RCC->PLLCFGR = 0x0u;
}
std::uint32_t pll::r::get_frequency_Hz()
{
    return calculate_PLL_channel_frequency_Hz((bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) +
                                              1u);
}

std::uint32_t pll::q::get_frequency_Hz()
{
    return calculate_PLL_channel_frequency_Hz((bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos) +
                                              1u);
}

std::uint32_t pll::p::get_frequency_Hz()
{
    return calculate_PLL_channel_frequency_Hz((bit_flag::get(RCC->PLLCFGR, RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) +
                                              1u);
}

void pll::sai1::enable(Limited<std::uint32_t, 8u, 86u> a_N,
                       const r::Enable_config& a_R,
                       const q::Enable_config& a_Q,
                       const p::Enable_config& a_P)
{
    hkm_assert(false);
}

bool pll::sai1::enable(Limited<std::uint32_t, 8u, 86u> a_N,
                       const r::Enable_config& a_R,
                       const q::Enable_config& a_Q,
                       const p::Enable_config& a_P,
                       Milliseconds a_timeout)
{
    hkm_assert(false);

    return false;
}

void pll::sai1::disable()
{
    bit_flag::clear(&(RCC->CR), RCC_CR_PLLSAI1ON);
}
} // namespace sources
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif