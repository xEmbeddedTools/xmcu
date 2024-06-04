#pragma once

/*
 */

// hkm
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/ADC/ADC.hpp>
#elif defined(STM32L0x0)
#include <xmcu/soc/ST/m0/stm32l0/rm0451/peripherals/ADC/ADC.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using ADC = xmcu::soc::m4::stm32wb::peripherals::ADC;
#elif defined(STM32L0x0)
using ADC = xmcu::soc::m0::stm32l0::rm0451::peripherals::ADC;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu
