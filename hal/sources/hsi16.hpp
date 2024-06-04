#pragma once

/*
 */

//xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hsi16.hpp>
#elif defined(STM32L0)
#include <xmcu/soc/ST/m0/stm32l0/rm0451/sources/hsi16.hpp>
#endif

namespace xmcu {
namespace hal {
namespace sources {
#if defined(STM32WB)
using hsi16 = xmcu::soc::m4::stm32wb::sources::hsi16;
#elif defined(STM32L0)
using hsi16 = xmcu::soc::m0::stm32l0::rm0451::sources::hsi16;
#endif
} // namespace sources
} // namespace hal
} // namespace xmcu
