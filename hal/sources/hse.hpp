#pragma once

/*
 */

//xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hse.hpp>
#endif

namespace xmcu {
namespace hal {
namespace sources {
#if defined(STM32WB)
using hse = xmcu::soc::m4::stm32wb::sources::hse;
#endif
} // namespace sources
} // namespace hal
} // namespace xmcu