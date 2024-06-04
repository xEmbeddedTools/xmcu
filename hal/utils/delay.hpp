#pragma once

/*
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/delay.hpp>
#elif defined(STM32L0)
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/delay.hpp>
#endif

namespace xmcu {
namespace hal {
namespace utils {
#if defined(STM32WB)
using delay = xmcu::soc::m4::stm32wb::utils::delay;
#elif defined(STM32L0)
using delay = xmcu::soc::m0::stm32l0::rm0451::utils::delay;
#endif
} // namespace utils
} // namespace hal
} // namespace xmcu
