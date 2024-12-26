#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#include <xmcu/config.hpp>

#if defined(XMCU_HSI16_PRESENT)
// soc
// clang-format off
#include DECORATE_INCLUDE_PATH(soc/XMCU_SOC_VENDOR/XMCU_SOC_ARCHITECTURE/XMCU_SOC_CORE_FAMILY/XMCU_SOC_VENDOR_FAMILY/XMCU_SOC_VENDOR_FAMILY_RM/clocks/sources/hsi16.hpp)
// clang-format on

namespace xmcu::hal::clocks::sources {
using hsi16 = xmcu::soc::XMCU_SOC_VENDOR::XMCU_SOC_ARCHITECTURE::XMCU_SOC_CORE_FAMILY::XMCU_SOC_VENDOR_FAMILY::
    XMCU_SOC_VENDOR_FAMILY_RM::clocks::sources::hsi16;
} // namespace xmcu::hal::clocks::sources
#endif