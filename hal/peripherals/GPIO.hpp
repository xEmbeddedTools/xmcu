#pragma once

/*
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/GPIO/GPIO.hpp>
#elif defined(STM32L0x0)
#include <xmcu/soc/ST/m0/stm32l0/rm0451/peripherals/GPIO/GPIO.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {

#if defined(STM32WB)
using GPIO = xmcu::soc::m4::stm32wb::peripherals::GPIO;
#elif defined(STM32L0x0)
using GPIO = xmcu::soc::m0::stm32l0::rm0451::peripherals::GPIO;
#endif

} // namespace peripherals
} // namespace hal
} // namespace xmcu