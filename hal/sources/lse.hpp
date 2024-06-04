#pragma once

/*
 */

//xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lse.hpp>
#endif

namespace xmcu {
namespace hal {
namespace sources {
#if defined(STM32WB)
using lse = xmcu::soc::m4::stm32wb::sources::lse;
#endif
} // namespace sources
} // namespace hal
} // namespace xmcu