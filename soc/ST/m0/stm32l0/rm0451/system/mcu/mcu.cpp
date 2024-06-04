/**/

// this
#include <xmcu/soc/ST/m0/stm32l0/rm0451/system/mcu/mcu.hpp>

// xmcu
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/wait_until.hpp>

namespace {
constexpr std::uint32_t hclk_dividers[] = { 1u, 3u, 5u, 1u, 1u, 6u, 10u, 32u, 2u, 4u, 8u, 16u, 64u, 128u, 256u, 512u };
constexpr std::uint32_t pclk_dividers[] = { 2u, 4u, 8u, 16u };
} // namespace

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {

using namespace xmcu;
using namespace xmcu::soc::m0::stm32l0::rm0451::sources;
using namespace xmcu::soc::m0::stm32l0::rm0451::system;
using namespace xmcu::soc::m0::stm32l0::rm0451::utils;

void rcc<mcu<1u>>::hclk<1u>::set(Prescaler a_prescaler)
{
    std::uint32_t prescaler_value = static_cast<std::uint32_t>(a_prescaler);
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_HPRE, prescaler_value);
    wait_until::all_bits_are_set(RCC->CFGR, prescaler_value);
}
bool rcc<mcu<1u>>::hclk<1u>::set(Prescaler a_prescaler, Milliseconds a_timeout)
{
    std::uint32_t prescaler_value = static_cast<std::uint32_t>(a_prescaler);
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_HPRE, prescaler_value);
    return wait_until::masked_bits_are_set(RCC->CFGR, RCC_CFGR_HPRE, prescaler_value, a_timeout);
}
rcc<mcu<1u>>::hclk<1u>::Prescaler rcc<mcu<1u>>::hclk<1u>::get_Prescaler()
{
    return Prescaler();
}
std::uint32_t rcc<mcu<1u>>::hclk<1u>::get_frequency_Hz()
{
    return SystemCoreClock / hclk_dividers[bit_flag::get(RCC->CFGR, RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
}

void rcc<mcu<1u>>::pclk<1u>::set(Prescaler a_prescaler)
{
    std::uint32_t prescaler_value = static_cast<std::uint32_t>(a_prescaler);
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE1, static_cast<std::uint32_t>(a_prescaler));
    // RM doesn't tell anything about waiting for changed PCLK but not doing so feels wrong
    wait_until::masked_bits_are_set(RCC->CFGR, RCC_CFGR_PPRE1, prescaler_value);
}
bool rcc<mcu<1u>>::pclk<1u>::set(Prescaler a_prescaler, Milliseconds a_timeout)
{
    std::uint32_t prescaler_value = static_cast<std::uint32_t>(a_prescaler);
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE1, prescaler_value);
    // RM doesn't tell anything about waiting for changed PCLK but not doing so feels wrong
    return wait_until::masked_bits_are_set(RCC->CFGR, RCC_CFGR_PPRE1, prescaler_value, a_timeout);
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

void rcc<mcu<1u>>::pclk<2u>::set(Prescaler a_prescaler)
{
    std::uint32_t prescaler_value = static_cast<std::uint32_t>(a_prescaler);
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE2, prescaler_value);
    // RM doesn't tell anything about waiting for changed PCLK but not doing so feels wrong
    wait_until::masked_bits_are_set(RCC->CFGR, RCC_CFGR_PPRE2, prescaler_value);
}
bool rcc<mcu<1u>>::pclk<2u>::set(Prescaler a_prescaler, Milliseconds a_timeout)
{
    std::uint32_t prescaler_value = static_cast<std::uint32_t>(a_prescaler);
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_PPRE2, prescaler_value);
    // RM doesn't tell anything about waiting for changed PCLK but not doing so feels wrong
    return wait_until::masked_bits_are_set(RCC->CFGR, RCC_CFGR_PPRE2, prescaler_value, a_timeout);
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

template<> void rcc<mcu<1u>>::set_system_clock_source<msi>()
{
    hkm_assert(true == msi::is_enabled());

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_MSI);
    wait_until::masked_bits_are_set(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_MSI);

    SystemCoreClock = msi::get_frequency_Hz();
}
template<> bool rcc<mcu<1u>>::set_system_clock_source<msi>(Milliseconds a_timeout)
{
    hkm_assert(true == msi::is_enabled());

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_MSI);
    if (true ==
        wait_until::masked_bits_are_set(
            RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_MSI, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        SystemCoreClock = msi::get_frequency_Hz();
        return true;
    }

    return false;
}
template<> bool rcc<mcu<1u>>::is_system_clock_source<msi>()
{
    return RCC_CFGR_SWS_MSI == bit_flag::get(RCC->CFGR, RCC_CFGR_SWS);
}

template<> void rcc<mcu<1u>>::set_system_clock_source<hsi16>()
{
    hkm_assert(true == hsi16::is_enabled());

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_HSI);
    wait_until::masked_bits_are_set(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI);

    SystemCoreClock = hsi16::get_frequency_Hz();
}
template<> bool rcc<mcu<1u>>::set_system_clock_source<hsi16>(Milliseconds a_timeout)
{
    hkm_assert(true == hsi16::is_enabled());

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_HSI);
    if (true ==
        wait_until::masked_bits_are_set(
            RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI, a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
    {
        SystemCoreClock = hsi16::get_frequency_Hz();
        return true;
    }

    return false;
}
template<> bool rcc<mcu<1u>>::is_system_clock_source<hsi16>()
{
    return RCC_CFGR_SWS_HSI == bit_flag::get(RCC->CFGR, RCC_CFGR_SWS);
}

template<> void rcc<mcu<1u>>::set_system_clock_source<hse>()
{
    hkm_assert(true == hse::is_enabled());

    bit_flag::set(&(RCC->CFGR), RCC_CFGR_SW, RCC_CFGR_SW_HSE);
    wait_until::masked_bits_are_set(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE);

    SystemCoreClock = hse::get_frequency_Hz();
}

template<> bool rcc<mcu<1u>>::is_system_clock_source<hse>()
{
    return RCC_CFGR_SWS_HSE == bit_flag::get(RCC->CFGR, RCC_CFGR_SWS);
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

} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
