/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/delay.hpp>

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
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
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::system;

void delay::wait(Milliseconds a_time)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();
    while (tick_counter<Milliseconds>::get() - start <= a_time.get()) continue;
}

void delay::wait(Seconds a_time)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();
    while (tick_counter<Milliseconds>::get() - start <= a_time.get_in<Milliseconds>().get())
        ;
}

void delay::wait(Microseconds a_time)
{
    hkm_assert(rcc<mcu<1u>>::get_system_clock_frequency_Hz() >= 1_MHz);
    hkm_assert(mcu<1u>::DWT_mode::enabled == mcu<1u>::get_DWT_mode());

    DWT->CYCCNT = 0;
    const std::uint32_t max =
        DWT->CYCCNT + (rcc<mcu<1u>>::get_system_clock_frequency_Hz() / 1_MHz * (a_time - 1_us).get());

    while (DWT->CYCCNT < max)
        ;
}
} // namespace utils
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif
