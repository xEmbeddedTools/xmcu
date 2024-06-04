#pragma once

/*
 */

//xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lsi.hpp>
#endif

namespace xmcu {
namespace hal {
namespace sources {
#if defined(STM32WB)
using lsi = xmcu::soc::m4::stm32wb::sources::lsi;
#endif
} // namespace sources
} // namespace hal
} // namespace xmcu