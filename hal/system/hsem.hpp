#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/system/hsem/hsem.hpp>
#endif

namespace xmcu {
namespace hal {
namespace system {
#if defined(STM32WB)
using hsem = xmcu::soc::m4::stm32wb::system::hsem;
#endif
} // namespace system
} // namespace hal
} // namespace xmcu