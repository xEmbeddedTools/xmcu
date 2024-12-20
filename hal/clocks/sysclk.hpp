#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#include <xmcu/hal/config.hpp>

// std
#include <cstdlib>

#if defined(XMCU_SYSCLKS_PRESENT) || defined(XMCU_SYSCLK_PRESENT)
// clang-format off
// soc
#include DECORATE_INCLUDE_PATH(xmcu/soc/XMCU_SOC_VENDOR/XMCU_SOC_ARCHITECTURE/XMCU_SOC_CORE_FAMILY/XMCU_SOC_VENDOR_FAMILY/XMCU_SOC_VENDOR_FAMILY_RM/clocks/sysclk.hpp)
// clang-format on
#endif

#if defined(XMCU_SYSCLKS_PRESENT)
namespace xmcu::hal::clocks {
template<std::size_t id> using sysclk = xmcu::soc::XMCU_SOC_VENDOR::XMCU_SOC_ARCHITECTURE::XMCU_SOC_CORE_FAMILY::
    XMCU_SOC_VENDOR_FAMILY::XMCU_SOC_VENDOR_FAMILY_RM::clocks::sysclk<id>;
} // namespace xmcu::hal::clocks
#endif

#if defined(XMCU_SYSCLK_PRESENT)
namespace xmcu::hal::clocks {
using sysclk = xmcu::soc::XMCU_SOC_VENDOR::XMCU_SOC_ARCHITECTURE::XMCU_SOC_CORE_FAMILY::XMCU_SOC_VENDOR_FAMILY::
    XMCU_SOC_VENDOR_FAMILY_RM::clocks::sysclk;
} // namespace xmcu::hal::clocks
#endif