#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// hkm
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/wb/rm0434/peripherals/ADC/ADC.hpp>
#elif defined(STM32L0x0)
#include <xmcu/soc/ST/arm/m0/l0/rm0451/peripherals/ADC/ADC.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using ADC = xmcu::soc::m4::wb::rm0434::peripherals::ADC;
#elif defined(STM32L0x0)
using ADC = xmcu::soc::m0::l0::rm0451::peripherals::ADC;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu
