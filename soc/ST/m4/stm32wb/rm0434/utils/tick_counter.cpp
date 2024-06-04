/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace utils {
// using namespace common;
using namespace xmcu::soc::m4::stm32wb::sources;
using namespace xmcu::soc::m4::stm32wb::system;

tick_counter<Milliseconds>::Callback tick_counter<Milliseconds>::callback;
std::uint16_t auto_reload = 0x0u;

std::uint64_t tick_counter<Milliseconds>::get()
{
    return cnt;
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
    a_p_timer->enable((rcc<mcu<1u>>::hclk<1u>::get_frequency_Hz() / 1000u) - 1, Systick::Prescaler::_1);
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
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif
