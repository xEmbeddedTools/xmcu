/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/mcu/mcu.hpp>

// xmcu
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/hsem/hsem.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

namespace {
constexpr std::uint32_t hclk_dividers[] = { 1u, 3u, 5u, 1u, 1u, 6u, 10u, 32u, 2u, 4u, 8u, 16u, 64u, 128u, 256u, 512u };
constexpr std::uint32_t pclk_dividers[] = { 2u, 4u, 8u, 16u };
} // namespace

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::sources;
using namespace xmcu::soc::m4::stm32wb::system;
using namespace xmcu::soc::m4::stm32wb::utils;

template<> void rcc<mcu<1u>>::clk48::set<hsi48>()
{
    hkm_assert(true == hsi48::is_enabled());

    Scoped_guard<hsem::_1_step> sem5_guard(0x5u);
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL);
}
template<> bool rcc<mcu<1u>>::clk48::set<hsi48>(Milliseconds a_timeout)
{
    hkm_assert(true == hsi48::is_enabled());

    Scoped_guard<hsem::_1_step> sem5_guard(0x5u, a_timeout);

    if (true == sem5_guard.is_locked())
    {
        bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL);
        return true;
    }
    return false;
}
template<> bool rcc<mcu<1u>>::clk48::is_source<hsi48>()
{
    return 0x0 == bit_flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL);
}
template<> void rcc<mcu<1u>>::clk48::set<msi>()
{
    hkm_assert(true == msi::is_enabled());
    hkm_assert(48_MHz == msi::get_frequency_Hz());

    Scoped_guard<hsem::_1_step> sem5_guard(0x5u);
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1);
}
template<> bool rcc<mcu<1u>>::clk48::set<msi>(Milliseconds a_timeout)
{
    hkm_assert(true == msi::is_enabled());
    hkm_assert(48_MHz == msi::get_frequency_Hz());

    Scoped_guard<hsem::_1_step> sem5_guard(0x5u, a_timeout);

    if (true == sem5_guard.is_locked())
    {
        bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1);
        return true;
    }
    return false;
}
template<> bool rcc<mcu<1u>>::clk48::is_source<msi>()
{
    return (RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1) == bit_flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL);
}
template<> void rcc<mcu<1u>>::clk48::set<pll::q>()
{
    hkm_assert(pll::q::Enable_config::Output::enabled == pll::q::get_Enable_config().output);
    hkm_assert(48_MHz == pll::q::get_frequency_Hz());

    Scoped_guard<hsem::_1_step> sem5_guard(0x5u);

    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_1);
}
template<> bool rcc<mcu<1u>>::clk48::set<pll::q>(Milliseconds a_timeout)
{
    hkm_assert(pll::q::Enable_config::Output::enabled == pll::q::get_Enable_config().output);
    hkm_assert(48_MHz == pll::q::get_frequency_Hz());

    Scoped_guard<hsem::_1_step> sem5_guard(0x5u, a_timeout);

    if (true == sem5_guard.is_locked())
    {
        bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_1);
        return true;
    }
    return false;
}
template<> bool rcc<mcu<1u>>::clk48::is_source<pll::q>()
{
    return RCC_CCIPR_CLK48SEL_1 == bit_flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL);
}
template<> void rcc<mcu<1u>>::clk48::set<pll::sai1::q>()
{
    hkm_assert(pll::sai1::q::Enable_config::Output::enabled == pll::sai1::q::get_Enable_config().output);
    hkm_assert(48_MHz == pll::sai1::q::get_frequency_Hz());

    Scoped_guard<hsem::_1_step> sem5_guard(0x5u);
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_0);
}
template<> bool rcc<mcu<1u>>::clk48::set<pll::sai1::q>(Milliseconds a_timeout)
{
    hkm_assert(pll::sai1::q::Enable_config::Output::enabled == pll::sai1::q::get_Enable_config().output);
    hkm_assert(48_MHz == pll::sai1::q::get_frequency_Hz());

    Scoped_guard<hsem::_1_step> sem5_guard(0x5u, a_timeout);

    if (true == sem5_guard.is_locked())
    {
        bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_CLK48SEL, RCC_CCIPR_CLK48SEL_0);
        return true;
    }
    return false;
}
template<> bool rcc<mcu<1u>>::clk48::is_source<pll::sai1::q>()
{
    return RCC_CCIPR_CLK48SEL_0 == bit_flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL);
}

std::uint32_t rcc<mcu<1u>>::clk48::get_frequency_Hz()
{
    switch (bit_flag::get(RCC->CCIPR, RCC_CCIPR_CLK48SEL))
    {
        case 0x0u: {
            return hsi48::get_frequency_Hz();
        }
        break;

        case RCC_CCIPR_CLK48SEL_0: {
            return pll::sai1::q::get_frequency_Hz();
        }
        break;

        case RCC_CCIPR_CLK48SEL_1: {
            return pll::q::get_frequency_Hz();
        }
        break;

        case RCC_CCIPR_CLK48SEL_0 | RCC_CCIPR_CLK48SEL_1: {
            return msi::get_frequency_Hz();
        }
        break;
    }

    return 0;
}

void rcc<mcu<1u>>::hclk<1u>::set(Prescaler a_prescaler)
{
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_HPRE, static_cast<std::uint32_t>(a_prescaler));
    wait_until::all_bits_are_set(RCC->CFGR, RCC_CFGR_HPREF);
}
bool rcc<mcu<1u>>::hclk<1u>::set(Prescaler a_prescaler, Milliseconds a_timeout)
{
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_HPRE, static_cast<std::uint32_t>(a_prescaler));
    return wait_until::all_bits_are_set(RCC->CFGR, RCC_CFGR_HPREF, a_timeout);
}
rcc<mcu<1u>>::hclk<1u>::Prescaler rcc<mcu<1u>>::hclk<1u>::get_Prescaler()
{
    return Prescaler();
}
std::uint32_t rcc<mcu<1u>>::hclk<1u>::get_frequency_Hz()
{
    return SystemCoreClock / hclk_dividers[bit_flag::get(RCC->CFGR, RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
}

void rcc<mcu<1u>>::hclk<2u>::set(Prescaler a_prescaler)
{
    bit_flag::set(&(RCC->EXTCFGR), RCC_EXTCFGR_C2HPRE, static_cast<std::uint32_t>(a_prescaler));
    wait_until::all_bits_are_set(RCC->EXTCFGR, RCC_EXTCFGR_C2HPREF);
}
bool rcc<mcu<1u>>::hclk<2u>::set(Prescaler a_prescaler, Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(RCC->EXTCFGR), RCC_EXTCFGR_C2HPRE, static_cast<std::uint32_t>(a_prescaler));
    return wait_until::all_bits_are_set(
        RCC->EXTCFGR, RCC_EXTCFGR_C2HPREF, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}
std::uint32_t rcc<mcu<1u>>::hclk<2u>::get_frequency_Hz()
{
    return SystemCoreClock / hclk_dividers[bit_flag::get(RCC->EXTCFGR, RCC_EXTCFGR_C2HPRE) >> RCC_EXTCFGR_C2HPRE_Pos];
}

void rcc<mcu<1u>>::hclk<4u>::set(Prescaler a_prescaler)
{
    bit_flag::set(&(RCC->EXTCFGR), RCC_EXTCFGR_SHDHPRE, static_cast<std::uint32_t>(a_prescaler));
    wait_until::all_bits_are_set(RCC->EXTCFGR, RCC_EXTCFGR_SHDHPREF);
}
bool rcc<mcu<1u>>::hclk<4u>::set(Prescaler a_prescaler, Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(RCC->EXTCFGR), RCC_EXTCFGR_SHDHPRE, static_cast<std::uint32_t>(a_prescaler));
    return wait_until::all_bits_are_set(
        RCC->EXTCFGR, RCC_EXTCFGR_SHDHPREF, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}
std::uint32_t rcc<mcu<1u>>::hclk<4u>::get_frequency_Hz()
{
    return SystemCoreClock / hclk_dividers[bit_flag::get(RCC->EXTCFGR, RCC_EXTCFGR_SHDHPRE) >> RCC_EXTCFGR_SHDHPRE_Pos];
}

void rcc<mcu<1u>>::pclk<1u>::set(Prescaler a_prescaler)
{
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<std::uint32_t>(a_prescaler));
    wait_until::all_bits_are_set(RCC->CFGR, RCC_CFGR_PPRE1F);
}
bool rcc<mcu<1u>>::pclk<1u>::set(Prescaler a_prescaler, Milliseconds a_timeout)
{
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<std::uint32_t>(a_prescaler));
    return wait_until::all_bits_are_set(RCC->CFGR, RCC_CFGR_PPRE1F, a_timeout);
}
std::uint32_t rcc<mcu<1u>>::pclk<1u>::get_frequency_Hz()
{
    std::uint32_t index = bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;

    if (index >= 4)
    {
        return SystemCoreClock / pclk_dividers[index - 4];
    }

    return SystemCoreClock;
}
std::uint32_t rcc<mcu<1u>>::pclk<1u>::get_Tim_frequency_Hz()
{
    std::uint32_t index = bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;

    if (index >= 4)
    {
        return SystemCoreClock * 2 / pclk_dividers[index - 4];
    }

    return SystemCoreClock;
}

void rcc<mcu<1u>>::pclk<2u>::set(Prescaler a_prescaler)
{
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE2, static_cast<std::uint32_t>(a_prescaler));
    wait_until::all_bits_are_set(RCC->CFGR, RCC_CFGR_PPRE2F);
}
bool rcc<mcu<1u>>::pclk<2u>::set(Prescaler a_prescaler, Milliseconds a_timeout)
{
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE2, static_cast<std::uint32_t>(a_prescaler));
    return wait_until::all_bits_are_set(RCC->CFGR, RCC_CFGR_PPRE2F, a_timeout);
}
rcc<mcu<1u>>::pclk<2u>::Prescaler rcc<mcu<1u>>::pclk<2u>::get_Prescaler()
{
    return Prescaler();
}
std::uint32_t rcc<mcu<1u>>::pclk<2u>::get_frequency_Hz()
{
    std::uint32_t index = bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;

    if (index >= 4)
    {
        return SystemCoreClock / pclk_dividers[index - 4];
    }

    return SystemCoreClock;
}
std::uint32_t rcc<mcu<1u>>::pclk<2u>::get_Tim_frequency_Hz()
{
    std::uint32_t index = bit_flag::get(RCC->CFGR, RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;

    if (index >= 4)
    {
        return SystemCoreClock * 2 / pclk_dividers[index - 4];
    }

    return SystemCoreClock;
}

template<> void rcc<mcu<1u>>::set_system_clock_source<msi>()
{
    hkm_assert(true == msi::is_enabled());

    bit_flag::clear(&(RCC->CFGR), RCC_CFGR_SW);
    wait_until::all_bits_are_cleared(RCC->CFGR, RCC_CFGR_SWS);

    SystemCoreClock = msi::get_frequency_Hz();
}
template<> bool rcc<mcu<1u>>::set_system_clock_source<msi>(Milliseconds a_timeout)
{
    hkm_assert(true == msi::is_enabled());

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::clear(&(RCC->CFGR), RCC_CFGR_SW);
    if (true == wait_until::all_bits_are_cleared(
                    RCC->CFGR, RCC_CFGR_SWS, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        SystemCoreClock = msi::get_frequency_Hz();
        return true;
    }

    return false;
}
template<> bool rcc<mcu<1u>>::is_system_clock_source<msi>()
{
    return false == bit_flag::is(RCC->CFGR, RCC_CFGR_SWS);
}

template<> void rcc<mcu<1u>>::set_system_clock_source<hsi16>()
{
    hkm_assert(true == hsi16::is_enabled());

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_0);
    wait_until::all_bits_are_set(RCC->CFGR, RCC_CFGR_SWS_0);

    SystemCoreClock = hsi16::get_frequency_Hz();
}
template<> bool rcc<mcu<1u>>::set_system_clock_source<hsi16>(Milliseconds a_timeout)
{
    hkm_assert(true == hsi16::is_enabled());

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_0);
    if (true == wait_until::all_bits_are_set(
                    RCC->CFGR, RCC_CFGR_SWS_0, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        SystemCoreClock = hsi16::get_frequency_Hz();
        return true;
    }

    return false;
}
template<> bool rcc<mcu<1u>>::is_system_clock_source<hsi16>()
{
    return RCC_CFGR_SWS_0 == bit_flag::get(RCC->CFGR, RCC_CFGR_SWS);
}

template<> void rcc<mcu<1u>>::set_system_clock_source<pll>()
{
    hkm_assert(true == pll::is_enabled());
    hkm_assert(pll::r::Enable_config::Output::enabled == pll::r::get_Enable_config().output);

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW_0 | RCC_CFGR_SW_1);
    wait_until::all_bits_are_set(RCC->CFGR, RCC_CFGR_SWS_0 | RCC_CFGR_SWS_1);

    SystemCoreClock = pll::r::get_frequency_Hz();
}
template<> bool rcc<mcu<1u>>::set_system_clock_source<pll>(Milliseconds a_timeout)
{
    hkm_assert(true == pll::is_enabled() &&
               pll::p::Enable_config::Output::enabled == pll::p::get_Enable_config().output);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_0 | RCC_CFGR_SW_1);
    if (true == wait_until::all_bits_are_set(RCC->CFGR,
                                             RCC_CFGR_SWS_0 | RCC_CFGR_SWS_1,
                                             a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        SystemCoreClock = pll::r::get_frequency_Hz();
        return true;
    }

    return false;
}
template<> bool rcc<mcu<1u>>::is_system_clock_source<pll>()
{
    return true == bit_flag::is(RCC->CFGR, RCC_CFGR_SWS);
}

template<> void rcc<mcu<1u>>::set_system_clock_source<hse, hse::Prescaler::_1>()
{
    hkm_assert(true == hse::is_enabled());

    bit_flag::clear(&(RCC->CR), RCC_CR_HSEPRE);
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_1);
    wait_until::all_bits_are_set(RCC->CFGR, RCC_CFGR_SWS_1);

    SystemCoreClock = hse::get_frequency_Hz();
}
template<> void rcc<mcu<1u>>::set_system_clock_source<hse, hse::Prescaler::_2>()
{
    hkm_assert(true == hse::is_enabled());

    bit_flag::set(&(RCC->CR), RCC_CR_HSEPRE);
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_1);
    wait_until::all_bits_are_set(RCC->CFGR, RCC_CFGR_SWS_1);

    SystemCoreClock = hse::get_frequency_Hz() / 2u;
}
template<> bool rcc<mcu<1u>>::set_system_clock_source<hse, hse::Prescaler::_1>(Milliseconds a_timeout)
{
    hkm_assert(true == hse::is_enabled());

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::clear(&(RCC->CR), RCC_CR_HSEPRE);
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_1);
    if (true == wait_until::all_bits_are_set(
                    RCC->CFGR, RCC_CFGR_SWS_1, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        SystemCoreClock = hse::get_frequency_Hz();
        return true;
    }

    return false;
}
template<> bool rcc<mcu<1u>>::set_system_clock_source<hse, hse::Prescaler::_2>(Milliseconds a_timeout)
{
    hkm_assert(true == hse::is_enabled());

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(RCC->CR), RCC_CR_HSEPRE);
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_1);
    if (true == wait_until::all_bits_are_set(
                    RCC->CFGR, RCC_CFGR_SWS_1, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        SystemCoreClock = hse::get_frequency_Hz() / 2u;
        return true;
    }

    return false;
}
template<> bool rcc<mcu<1u>>::is_system_clock_source<hse>()
{
    return RCC_CFGR_SWS_1 == bit_flag::get(RCC->CFGR, RCC_CFGR_SWS_0);
}

std::uint32_t rcc<mcu<1u>>::get_system_clock_frequency_Hz()
{
    return SystemCoreClock;
}

template<> void rcc<mcu<1u>>::set_wakeup_clock_source<msi>()
{
    bit_flag::clear(&(RCC->CFGR), RCC_CFGR_STOPWUCK);
}
template<> void rcc<mcu<1u>>::set_wakeup_clock_source<hsi16>()
{
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_STOPWUCK, RCC_CFGR_STOPWUCK);
}

template<> void rcc<mcu<2u>>::set_wakeup_clock_source<lse>()
{
    bit_flag::set(&(RCC->CSR), RCC_CSR_RFWKPSEL, RCC_CSR_RFWKPSEL_0);
}
template<> void rcc<mcu<2u>>::set_wakeup_clock_source<hse>()
{
    bit_flag::set(&(RCC->CSR), RCC_CSR_RFWKPSEL_0 | RCC_CSR_RFWKPSEL_1);
}

template<> void rcc<mcu<1u>>::hclk<5u>::set<hsi16>()
{
    bit_flag::clear(&(RCC->EXTCFGR), RCC_EXTCFGR_RFCSS);
}
template<> void rcc<mcu<1u>>::hclk<5u>::set<hse, hse::Prescaler::_2>()
{
    bit_flag::set(&(RCC->EXTCFGR), RCC_EXTCFGR_RFCSS);
}
namespace system {

const char* to_string(mcu<1u>::Reset_source a_source)
{
    switch (a_source)
    {
        case mcu<1u>::Reset_source::low_power:
            return "low_power";
        case mcu<1u>::Reset_source::window_watchdog:
            return "window_watchdog";
        case mcu<1u>::Reset_source::independent_window_watchdog:
            return "independent_window_watchdog";
        case mcu<1u>::Reset_source::software:
            return "software";
        case mcu<1u>::Reset_source::bor:
            return "bor";
        case mcu<1u>::Reset_source::pin:
            return "pin";
        case mcu<1u>::Reset_source::option_byte:
            return "option_byte";
        default:
            return nullptr;
    }
}
}
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif