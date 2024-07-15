#pragma once

/*
 */

#include <xmcu/soc/ST/arm/Systick.hpp>
// #if defined(M4)
// #include <xmcu/soc/ST/m4/Systick/Systick.hpp>
// #elif defined(M0) || defined(M0_PLUS)
// #include <xmcu/soc/ST/arm/m0/Systick/Systick.hpp>
// #endif

namespace xmcu {
namespace hal {
using Systick = xmcu::soc::Systick;
// #if defined(M4)
// using Systick = xmcu::soc::m4::Systick;
// #elif defined(M0) || defined(M0_PLUS)
// using Systick = xmcu::soc::m0::Systick;
// #endif
} // namespace hal
} // namespace xmcu
