#pragma once

/*
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/rng/rng.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using rng = xmcu::soc::m4::stm32wb::peripherals::rng;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu