#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#include <xmcu/config.hpp>

// std
#include <cstdlib>

#if defined(XMCU_HCLK_PRESENT)
// clang-format off
// soc
#include DECORATE_INCLUDE_PATH(soc/XMCU_SOC_VENDOR/XMCU_SOC_ARCHITECTURE/XMCU_SOC_CORE_FAMILY/XMCU_SOC_VENDOR_FAMILY/XMCU_SOC_VENDOR_FAMILY_RM/clocks/hclk.hpp)
// clang-format on

namespace xmcu::hal::clocks {
template<std::size_t id> using hclk = xmcu::soc::XMCU_SOC_VENDOR::XMCU_SOC_ARCHITECTURE::XMCU_SOC_CORE_FAMILY::
    XMCU_SOC_VENDOR_FAMILY::XMCU_SOC_VENDOR_FAMILY_RM::clocks::hclk<id>;
}
#endif