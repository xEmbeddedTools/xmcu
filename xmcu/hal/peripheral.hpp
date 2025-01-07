#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#if defined(STM32WB) || defined(STM32L0)
#include <soc/peripheral.hpp>
#endif

namespace xmcu {
namespace hal {
#if defined(STM32WB) || defined(STM32L0)
template<typename Perihperal_t,
         std::uint32_t perihperal_id = std::numeric_limits<std::uint32_t>::max(),
         typename DMA_t = void*,
         std::uint32_t DMA_id = std::numeric_limits<std::uint32_t>::max()>
using peripheral = xmcu::soc::peripheral<Perihperal_t, perihperal_id, DMA_t, DMA_id>;
#endif
} // namespace hal
} // namespace xmcu
