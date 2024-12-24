#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#include <xmcu/hal/config.hpp>

#if defined(XMCU_MSI_PRESENT)
// clang-format off
// soc
#include DECORATE_INCLUDE_PATH(xmcu/soc/XMCU_SOC_VENDOR/XMCU_SOC_ARCHITECTURE/XMCU_SOC_CORE_FAMILY/XMCU_SOC_VENDOR_FAMILY/XMCU_SOC_VENDOR_FAMILY_RM/clocks/sources/msi.hpp)
// clang-format on

namespace xmcu::hal::clocks::sources {
using msi = xmcu::soc::XMCU_SOC_VENDOR::XMCU_SOC_ARCHITECTURE::XMCU_SOC_CORE_FAMILY::XMCU_SOC_VENDOR_FAMILY::
    XMCU_SOC_VENDOR_FAMILY_RM::clocks::sources::msi;
} // namespace xmcu::hal::clocks::sources
#endif