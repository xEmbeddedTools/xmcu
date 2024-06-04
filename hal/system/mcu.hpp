#pragma once

/*
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/mcu/mcu.hpp>
#elif defined(STM32L0)
#include <xmcu/soc/ST/m0/stm32l0/rm0451/system/mcu/mcu.hpp>
#endif

namespace xmcu {
namespace hal {
namespace system {
#if defined(STM32WB)
template<std::size_t id> using mcu = xmcu::soc::m4::stm32wb::system::mcu<id>;
#elif defined(STM32L0)
template<std::size_t id> using mcu = xmcu::soc::m0::stm32l0::rm0451::system::mcu<id>;
#endif
} // namespace system
} // namespace hal
} // namespace xmcu
