#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

#if defined(XMCU_SOC_MODEL_STM32L010F4P6) || defined(XMCU_SOC_MODEL_STM32L010C6T6)
// CMSIS
#include <stm32l0xx.h>

// xmcu
#include <soc/st/arm/m0/l0/rm0451/config.hpp>
#endif

#if defined(XMCU_SOC_MODEL_STM32WB35CEU6A)
//CMSIS
#include <stm32wbxx.h>

// xmcu
#include <soc/st/arm/m4/wb/rm0434/config.hpp>
#endif

#define DECORATE_INCLUDE_PATH(x) <x>