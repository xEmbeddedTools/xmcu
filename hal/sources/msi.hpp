#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

//xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/sources/msi.hpp>
#elif defined(STM32L0)
#include <xmcu/soc/ST/arm/m0/stm32l0/rm0451/sources/msi.hpp>
#endif

namespace xmcu {
namespace hal {
namespace sources {
#if defined(STM32WB)
using msi = xmcu::soc::m4::stm32wb::rm0434::sources::msi;
#elif defined(STM32L0)
using msi = xmcu::soc::m0::stm32l0::rm0451::sources::msi;
#endif
} // namespace sources
} // namespace hal
} // namespace xmcu
