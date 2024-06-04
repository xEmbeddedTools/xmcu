/**/

// this
#include <xmcu/soc/ST/m0/stm32l0/rm0451/system/pwr/pwr.hpp>

// xmcu
#include <xmcu/soc/ST/m0/stm32l0/rm0451/peripherals/internal_flash/internal_flash.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/rcc.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/system/mcu/mcu.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/wait_until.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace system {
using namespace xmcu;
using namespace xmcu::soc::m0::stm32l0::rm0451::sources;
using namespace xmcu::soc::m0::stm32l0::rm0451::system;
using namespace xmcu::soc::m0::stm32l0::rm0451::utils;
using namespace xmcu::soc::m0::stm32l0::rm0451::peripherals;

void pwr<mcu<1u>>::stop_mode::enter(Type a_type, Method a_method, Sleeponexit a_sleeponexit)
{
    bit_flag::set(&PWR->CR, PWR_CR_CWUF | PWR_CR_LPSDSR | PWR_CR_ULP);
    bit_flag::set(&SCB->SCR, static_cast<std::uint32_t>(a_sleeponexit) | SCB_SCR_SLEEPDEEP_Msk);
    switch (a_method)
    {
        case Method::wfe: {
            __SEV();
            __WFE();
            __WFE();
        }
        break;

        case Method::wfi: {
            __WFI();
        }
        break;

        case Method::none: {
        }
        break;
    }
}

void pwr<mcu<1u>>::stop_mode::exit()
{
    if (0 == bit_flag::is(RCC->CFGR, RCC_CFGR_STOPWUCK))
    {
        rcc<mcu<1u>>::set_system_clock_source<msi>();
    }
    else
    {
        rcc<mcu<1u>>::set_system_clock_source<hsi16>();
    }
}

void pwr<mcu<1u>>::set_voltage_scaling(Voltage_scaling a_scaling)
{
    bit_flag::set(&(PWR->CR), PWR_CR_VOS, static_cast<std::uint32_t>(a_scaling));
    wait_until::all_bits_are_cleared(PWR->CSR, PWR_CSR_VOSF);
}

bool pwr<mcu<1u>>::set_voltage_scaling(Voltage_scaling a_scaling, Milliseconds a_timeout)
{
    bit_flag::set(&(PWR->CR), PWR_CR_VOS, static_cast<std::uint32_t>(a_scaling));
    return wait_until::all_bits_are_cleared(PWR->CSR, PWR_CSR_VOSF, a_timeout);
}

pwr<mcu<1u>>::Voltage_scaling pwr<mcu<1u>>::get_voltage_scaling()
{
    return static_cast<Voltage_scaling>(bit_flag::get(PWR->CR, PWR_CR_VOS));
}

} // namespace system
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
