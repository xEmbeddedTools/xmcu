#pragma once

#if defined(STM32L010F4P6) || defined(STM32L010C6T6)
#include <xmcu/soc/ST/arm/m0/l0/rm0451/config.hpp>
#endif

#if defined(STM32WB35CEU6A)
#include <xmcu/soc/ST/arm/m4/wb/rm0434/config.hpp>
#endif

#define DECORATE_INCLUDE_PATH(x) <x>