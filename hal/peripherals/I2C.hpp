 #pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/peripherals/I2C/I2C.hpp>
#elif defined(STM32L0)
#include <xmcu/soc/ST/arm/m0/stm32l0/rm0451/peripherals/I2C/I2C.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using I2C  = xmcu::soc::m4::stm32wb::rm0434::peripherals::I2C;
#elif defined(STM32L0)
// using I2C  = xmcu::soc::m0::stm32l0::rm0451::peripherals::I2C;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu
