#pragma once

/*
 */

// xmcu
#if defined(M4)
#include <xmcu/soc/ST/m4/nvic.hpp>
#elif defined(M0) || defined(M0_PLUS)
#include <xmcu/soc/ST/m0/nvic.hpp>
#endif

namespace xmcu {
namespace hal {
#if defined(M4)
using nvic = xmcu::soc::m4::nvic;
#elif defined(M0) || defined(M0_PLUS)
using nvic = xmcu::soc::m0::nvic;
#endif
} // namespace hal
} // namespace xmcu
