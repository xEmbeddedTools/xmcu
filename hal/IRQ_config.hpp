#pragma once

/*
 */

// xmcu
#if defined(M4)
#include <xmcu/soc/ST/m4/IRQ_config.hpp>
#elif defined(M0) || defined(M0_PLUS)
#include <xmcu/soc/ST/m0/IRQ_config.hpp>
#endif

namespace xmcu {
namespace hal {
#if defined(M4)
using IRQ_config = xmcu::soc::m4::IRQ_config;
#elif defined(M0) || defined(M0_PLUS)
using IRQ_config = xmcu::soc::m0::IRQ_config;
#endif
} // namespace hal
} // namespace xmcu
