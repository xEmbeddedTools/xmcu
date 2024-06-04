#pragma once

/*
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/internal_flash/option_bytes.hpp>
#elif defined(STM32L0x0)
#include <xmcu/soc/ST/m0/stm32l0/rm0451/peripherals/internal_flash/option_bytes.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using option_bytes = xmcu::soc::m4::stm32wb::peripherals::option_bytes;
#elif defined(STM32L0x0)
using option_bytes = xmcu::soc::m0::stm32l0::rm0451::peripherals::option_bytes;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu
