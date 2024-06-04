#pragma once

/*
 */

// xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/pwr/pwr.hpp>
#elif defined(STM32L0x0)
#include <xmcu/soc/ST/m0/stm32l0/rm0451/system/pwr/pwr.hpp>
#endif

namespace xmcu {
namespace hal {
namespace system {
#if defined(STM32WB)
template<typename MCU_t> using pwr = xmcu::soc::m4::stm32wb::system::pwr<MCU_t>;
#elif defined(STM32L0x0)
template<typename MCU_t> using pwr = xmcu::soc::m0::stm32l0::rm0451::system::pwr<MCU_t>;
#endif
} // namespace system
} // namespace hal
} // namespace xmcu
