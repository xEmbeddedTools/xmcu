#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/peripherals/Timer/LPTIM.hpp>
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/peripherals/Timer/TIM.hpp>
#elif defined(STM32L0)
#include <xmcu/soc/ST/arm/m0/stm32l0/rm0451/peripherals/Timer/LPTIM.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
using LPTIM = xmcu::soc::m4::stm32wb::peripherals::LPTIM;
using TIM_ADV = xmcu::soc::m4::stm32wb::peripherals::TIM_ADV;
using TIM_G16 = xmcu::soc::m4::stm32wb::peripherals::TIM_G16;
#elif defined(STM32L0)
using LPTIM = xmcu::soc::m0::stm32l0::rm0451::peripherals::LPTIM;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu
