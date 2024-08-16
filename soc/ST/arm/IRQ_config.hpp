#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

namespace xmcu {
namespace soc {
struct IRQ_config
{
    std::uint32_t preempt_priority = 0;
    std::uint32_t sub_priority = 0;
};
} // namespace soc
} // namespace xmcu