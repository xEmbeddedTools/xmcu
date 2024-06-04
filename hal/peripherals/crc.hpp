#pragma once

/*
 */

//xmcu
#if defined(STM32WB)
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/crc/crc.hpp>
#endif

namespace xmcu {
namespace hal {
namespace peripherals {
#if defined(STM32WB)
template<std::size_t length_t = 0x0u> using crc = xmcu::soc::m4::stm32wb::peripherals::crc<length_t>;
#endif
} // namespace peripherals
} // namespace hal
} // namespace xmcu