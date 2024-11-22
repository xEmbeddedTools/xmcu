#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/wb/rm0434/peripherals/SPI/SPI.hpp>
#elif defined(STM32L0x0)
#include <xmcu/soc/ST/arm/m0/l0/rm0451/peripherals/SPI/SPI.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using SPI = xmcu::soc::m4::wb::rm0434::peripherals::SPI;
#elif defined(STM32L0x0)
using SPI = xmcu::soc::m0::l0::rm0451::peripherals::SPI;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu
