#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/peripherals/internal_flash/internal_flash.hpp>
#elif defined(STM32L0x0)
#include <xmcu/soc/ST/arm/m0/stm32l0/rm0451/peripherals/internal_flash/internal_flash.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using internal_flash = xmcu::soc::m4::stm32wb::peripherals::internal_flash;
#elif defined(STM32L0x0)
using internal_flash = xmcu::soc::m0::stm32l0::rm0451::peripherals::internal_flash;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu
