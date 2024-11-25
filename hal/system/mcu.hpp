#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/wb/rm0434/system/mcu/mcu.hpp>
#elif defined(STM32L0)
#include <xmcu/soc/ST/arm/m0/l0/rm0451/system/mcu/mcu.hpp>
#endif

namespace xmcu {
namespace hal {
namespace system {
#if defined(STM32WB)
template<std::size_t id> using mcu = xmcu::soc::m4::wb::rm0434::system::mcu<id>;
#elif defined(STM32L0)
template<std::size_t id> using mcu = xmcu::soc::m0::l0::rm0451::system::mcu<id>;
#endif
} // namespace system
} // namespace hal
} // namespace xmcu
