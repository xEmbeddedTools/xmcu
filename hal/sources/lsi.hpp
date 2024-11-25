#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

//xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/wb/rm0434/sources/lsi.hpp>
#endif

namespace xmcu {
namespace hal {
namespace sources {
#if defined(STM32WB)
using lsi = xmcu::soc::m4::wb::rm0434::sources::lsi;
#endif
} // namespace sources
} // namespace hal
} // namespace xmcu