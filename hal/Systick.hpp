#pragma once

/*
 */

#if defined(M4)
#include <xmcu/soc/ST/m4/Systick/Systick.hpp>
#elif defined(M0) || defined(M0_PLUS)
#include <xmcu/soc/ST/m0/Systick/Systick.hpp>
#endif

namespace xmcu {
namespace hal {
#if defined(M4)
using Systick = xmcu::soc::m4::Systick;
#elif defined(M0) || defined(M0_PLUS)
using Systick = xmcu::soc::m0::Systick;
#endif
} // namespace hal
} // namespace xmcu
