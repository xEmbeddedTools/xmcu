#pragma once

/*
 */

//hkmmu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/sources/hsi48.hpp>
#endif

namespace xmcu {
namespace hal {
namespace sources {
#if defined(STM32WB)
using hsi48 = xmcu::soc::m4::stm32wb::sources::hsi48;
#endif
} // namespace sources
} // namespace hal
} // namespace xmcu