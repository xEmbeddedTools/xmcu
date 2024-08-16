#pragma once

/*
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/system/hsem/hsem.hpp>
#endif

namespace xmcu {
namespace hal {
namespace system {
#if defined(STM32WB)
using hsem = xmcu::soc::m4::stm32wb::system::hsem;
#endif
} // namespace system
} // namespace hal
} // namespace xmcu