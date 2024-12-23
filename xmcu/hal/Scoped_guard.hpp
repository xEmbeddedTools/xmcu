#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB) || defined(STM32L0)
#include <xmcu/soc/Scoped_guard.hpp>
#endif

namespace xmcu {
namespace hal {
#if defined(STM32WB) || defined(STM32L0)
template<typename Lock_t> using Scoped_guard = xmcu::soc::Scoped_guard<Lock_t>;
#endif
} // namespace hal
} // namespace xmcu
