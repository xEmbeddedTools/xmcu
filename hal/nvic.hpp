#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(M4)
#include <xmcu/soc/ST/arm/m4/nvic.hpp>
#elif defined(M0) || defined(M0_PLUS)
#include <xmcu/soc/ST/arm/m0/nvic.hpp>
#endif

namespace xmcu {
namespace hal {
#if defined(M4)
using nvic = xmcu::soc::m4::nvic;
#elif defined(M0) || defined(M0_PLUS)
using nvic = xmcu::soc::m0::nvic;
#endif
} // namespace hal
} // namespace xmcu
