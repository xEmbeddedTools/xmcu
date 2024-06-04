#pragma once

/*
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/rtc/rtc.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using rtc = xmcu::soc::m4::stm32wb::peripherals::rtc;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu
