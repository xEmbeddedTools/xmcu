#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/wb/rm0434/peripherals/USART/DMA.hpp>
#include <xmcu/soc/ST/arm/m4/wb/rm0434/peripherals/USART/LPUART.hpp>
#include <xmcu/soc/ST/arm/m4/wb/rm0434/peripherals/USART/USART.hpp>
#elif defined(STM32L0x0)
#include <xmcu/soc/ST/arm/m0/l0/rm0451/peripherals/USART/DMA.hpp>
#include <xmcu/soc/ST/arm/m0/l0/rm0451/peripherals/USART/LPUART.hpp>
#include <xmcu/soc/ST/arm/m0/l0/rm0451/peripherals/USART/USART.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using LPUART = xmcu::soc::m4::wb::rm0434::peripherals::LPUART;
using USART = xmcu::soc::m4::wb::rm0434::peripherals::USART;
#elif defined(STM32L0x0)
using LPUART = xmcu::soc::m0::l0::rm0451::peripherals::LPUART;
using USART = xmcu::soc::m0::l0::rm0451::peripherals::USART;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu
