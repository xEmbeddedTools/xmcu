#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// xmcu
#include <xmcu/non_constructible.hpp>

namespace xmcu {
namespace soc {
template<typename Lock_t> class Scoped_guard : private xmcu::non_constructible
{
};
} // namespace soc
} // namespace xmcu