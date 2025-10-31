#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#include <soc/peripheral.hpp>

namespace xmcu {
namespace hal {
template<typename Perihperal_t, typename Perihperal_id_t = void, typename DMA_t = void, typename DMA_id_t = void>
using peripheral = xmcu::soc::peripheral<Perihperal_t, Perihperal_id_t, DMA_t, DMA_id_t>;
} // namespace hal
} // namespace xmcu
