/**/

// this
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/tick_counter.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace utils {

// using namespace common;
using namespace xmcu::soc::m0::stm32l0::rm0451::sources;
using namespace xmcu::soc::m0::stm32l0::rm0451::system;

tick_counter<Milliseconds>::Callback tick_counter<Milliseconds>::callback;
std::uint16_t auto_reload = 0x0u;

std::uint64_t tick_counter<Milliseconds>::get()
{
    return cnt;
}

std::uint32_t tick_counter<Milliseconds>::get_ticks_per_period()
{
    return ticks_per_period;
}

std::uint32_t tick_counter<Milliseconds>::get_current_period_ticks()
{
    // HACK: Assume it's systick. Can be fixed by enabling cpp20 and defining p_timer as Tick_source* - needs concepts
    // for compile-time interfaces so that utils::delay fns don't need to know the clock type.
    Systick* p_tick_source = reinterpret_cast<Systick*>(p_timer);
    std::uint32_t ticks = p_tick_source->polling.get_value();
    return ticks;
}

void tick_counter<Milliseconds>::register_callback(const Callback& a_callback)
{
    callback = a_callback;
}

void tick_counter<Milliseconds>::update(void*)
{
    cnt = cnt + 1u;

    if (nullptr != callback.function)
    {
        callback.function(cnt, callback.p_user_data);
    }
}

template<> void tick_counter<Milliseconds>::enable<Systick>(Systick* a_p_timer,
                                                            const IRQ_config& a_irq_config,
                                                            std::uint64_t a_start_cnt)
{
    ticks_per_period = (rcc<mcu<1u>>::hclk<1u>::get_frequency_Hz() / 1000u);
    a_p_timer->enable(ticks_per_period - 1, Systick::Prescaler::_1);
    a_p_timer->interrupt.enable(a_irq_config);
    a_p_timer->interrupt.register_callback({ tick_counter::update, nullptr });
    p_timer = a_p_timer;
    cnt = a_start_cnt;
}

template<> void tick_counter<Milliseconds>::disable<Systick>()
{
    (reinterpret_cast<Systick*>(p_timer))->disable();
    p_timer = nullptr;
    cnt = 0u;
    ticks_per_period = 0u;
}

template<> void tick_counter<Milliseconds>::start<Systick>(bool a_call_handler_on_start)
{
    reinterpret_cast<Systick*>(p_timer)->start();

    if (true == a_call_handler_on_start && nullptr != callback.function)
    {
        callback.function(cnt, callback.p_user_data);
    }
}

template<> void tick_counter<Milliseconds>::stop<Systick>()
{
    reinterpret_cast<Systick*>(p_timer)->stop();
}

} // namespace utils
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
