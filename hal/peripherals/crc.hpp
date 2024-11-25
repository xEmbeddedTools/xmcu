#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

//xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/wb/rm0434/peripherals/crc/crc.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
template<std::size_t length_t = 0x0u> using crc = xmcu::soc::m4::wb::rm0434::peripherals::crc<length_t>;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu