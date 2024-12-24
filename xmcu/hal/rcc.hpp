#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <limits>

// xmcu
#include <xmcu/hal/config.hpp>

// clang-format off
// soc
#include DECORATE_INCLUDE_PATH(xmcu/soc/XMCU_SOC_VENDOR/XMCU_SOC_ARCHITECTURE/XMCU_SOC_CORE_FAMILY/XMCU_SOC_VENDOR_FAMILY/XMCU_SOC_VENDOR_FAMILY_RM/rcc.hpp)
// clang-format on

namespace xmcu::hal {
template<typename Perihperal_t, std::size_t id_t = std::numeric_limits<std::size_t>::max()> using rcc =
    xmcu::soc::XMCU_SOC_VENDOR::XMCU_SOC_ARCHITECTURE::XMCU_SOC_CORE_FAMILY::XMCU_SOC_VENDOR_FAMILY::
        XMCU_SOC_VENDOR_FAMILY_RM::rcc<Perihperal_t, id_t>;
} // namespace xmcu::hal