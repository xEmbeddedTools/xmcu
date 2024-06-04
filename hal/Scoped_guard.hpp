#pragma once

/*
 */

// xmcu
#if defined(STM32WB) || defined(STM32L0)
#include <xmcu/soc/Scoped_guard.hpp>
#endif

namespace xmcu {
namespace hal {
#if defined(STM32WB) || defined(STM32L0)
template<typename Lock_t> using Scoped_guard = xmcu::soc::Scoped_guard<Lock_t>;
#endif
} // namespace hal
} // namespace xmcu
