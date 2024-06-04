#pragma once

/**/

// hkm
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/rcc.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/mcu/mcu.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace utils {
class delay : private Non_constructible
{
public:
    static void wait(Milliseconds a_time);
    static void wait(Seconds a_time);
    static void wait(Microseconds a_time);
};
} // namespace utils
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu