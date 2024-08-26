#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/peripherals/iwdg/iwdg.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using iwdg = xmcu::soc::m4::stm32wb::rm0434::peripherals::iwdg;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu
