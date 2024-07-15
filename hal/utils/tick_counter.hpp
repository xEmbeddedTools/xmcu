#pragma once

/*
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/arm/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#elif defined(STM32L0)
#include <xmcu/soc/ST/arm/m0/stm32l0/rm0451/utils/tick_counter.hpp>
#endif

namespace xmcu {
namespace hal {
namespace utils {
#if defined(STM32WB)
template<typename Period_t> using tick_counter = xmcu::soc::m4::stm32wb::utils::tick_counter<Period_t>;
#elif defined(STM32L0)
template<typename Period_t> using tick_counter = xmcu::soc::m0::stm32l0::rm0451::utils::tick_counter<Period_t>;
#endif
} // namespace utils
} // namespace hal
} // namespace xmcu
