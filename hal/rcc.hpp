#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/wb/rm0434/rcc.hpp>
#elif defined(STM32L0)
#include <xmcu/soc/ST/arm/m0/l0/rm0451/rcc.hpp>
#endif

namespace xmcu {
namespace hal {
#if defined(STM32WB)
template<typename Perihperal_t, std::size_t id_t = std::numeric_limits<std::size_t>::max()> using rcc =
    xmcu::soc::m4::wb::rm0434::rcc<Perihperal_t, id_t>;
#elif defined(STM32L0)
template<typename Perihperal_t, std::size_t id_t = std::numeric_limits<std::size_t>::max()> using rcc =
    xmcu::soc::m0::l0::rm0451::rcc<Perihperal_t, id_t>;
#endif
} // namespace hal
} // namespace xmcu
