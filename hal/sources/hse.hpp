#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

//xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/sources/hse.hpp>
#endif

namespace xmcu {
namespace hal {
namespace sources {
#if defined(STM32WB)
using hse = xmcu::soc::m4::stm32wb::rm0434::sources::hse;
#endif
} // namespace sources
} // namespace hal
} // namespace xmcu