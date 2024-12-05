#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#include <xmcu/hal/config.hpp>

// clang-format off
// soc
#include DECORATE_INCLUDE_PATH(xmcu/soc/XMCU_SOC_VENDOR/XMCU_SOC_ARCHITECTURE/XMCU_SOC_CORE_FAMILY/XMCU_SOC_VENDOR_FAMILY/XMCU_SOC_VENDOR_FAMILY_RM/peripherals/crc/crc.hpp)
// clang-format on

namespace xmcu::hal::peripherals {
template<std::size_t length_t = 0x0u> using crc = xmcu::soc::XMCU_SOC_VENDOR::XMCU_SOC_ARCHITECTURE::
    XMCU_SOC_CORE_FAMILY::XMCU_SOC_VENDOR_FAMILY::XMCU_SOC_VENDOR_FAMILY_RM::peripherals::crc<length_t>;
} // namespace xmcu::hal::peripherals