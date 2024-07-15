#pragma once

/*
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/rcc.hpp>
#elif defined(STM32L0)
#include <xmcu/soc/ST/arm/m0/stm32l0/rm0451/rcc.hpp>
#endif

namespace xmcu {
namespace hal {
#if defined(STM32WB)
template<typename Perihperal_t, std::size_t id_t = std::numeric_limits<std::size_t>::max()> using rcc =
    xmcu::soc::m4::stm32wb::rcc<Perihperal_t, id_t>;
#elif defined(STM32L0)
template<typename Perihperal_t, std::size_t id_t = std::numeric_limits<std::size_t>::max()> using rcc =
    xmcu::soc::m0::stm32l0::rm0451::rcc<Perihperal_t, id_t>;
#endif
} // namespace hal
} // namespace xmcu
