#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/DMA.hpp>
#elif defined(STM32L0)
#include <xmcu/soc/ST/arm/m0/l0/rm0451/DMA.hpp>
#endif

namespace xmcu {
namespace hal {
#if defined(STM32WB)
template<typename Perihperal_t = void*> using DMA = xmcu::soc::m4::stm32wb::rm0434::DMA<Perihperal_t>;
#elif defined(STM32L0)
template<typename Perihperal_t = void*> using DMA = xmcu::soc::m0::l0::rm0451::DMA<Perihperal_t>;
#endif
} // namespace hal
} // namespace xmcu
