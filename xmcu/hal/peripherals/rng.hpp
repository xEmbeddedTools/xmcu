#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB)
#include <soc/ST/arm/m4/wb/rm0434/peripherals/rng/rng.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using rng = xmcu::soc::m4::wb::rm0434::peripherals::rng;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu