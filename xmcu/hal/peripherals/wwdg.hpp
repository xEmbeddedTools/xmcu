#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#include <xmcu/config.hpp>

// xmcu
#if defined(STM32WB)
#include <soc/st/arm/m4/wb/rm0434/peripherals/wwdg/wwdg.hpp>
#endif

namespace xmcu::hal::peripherals {
using wwdg = xmcu::soc::XMCU_SOC_VENDOR::XMCU_SOC_ARCHITECTURE::XMCU_SOC_CORE_FAMILY::XMCU_SOC_VENDOR_FAMILY::
    XMCU_SOC_VENDOR_FAMILY_RM::peripherals::wwdg;
} // namespace xmcu::hal::peripherals
