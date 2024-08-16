#pragma once

/*
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/peripherals/SPI/SPI.hpp>
#elif defined(STM32L0x0)
#include <xmcu/soc/ST/arm/m0/stm32l0/rm0451/peripherals/SPI/SPI.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using SPI = xmcu::soc::m4::stm32wb::peripherals::SPI;
#elif defined(STM32L0x0)
using SPI = xmcu::soc::m0::stm32l0::rm0451::peripherals::SPI;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu
