#pragma once

/**/

// std
#include <cstdint>

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/soc/ST/m4/IRQ_config.hpp>
#include <xmcu/soc/ST/m4/Systick/Systick.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/rcc.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hsi16.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lse.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lsi.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/mcu/mcu.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace utils {
template<typename Period_t> class tick_counter : private Non_constructible
{
};
template<> class tick_counter<Milliseconds> : private Non_constructible
{
public:
    struct Callback
    {
        using Function = void (*)(std::uint64_t a_cnt, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };

    template<typename Timer_t, typename Clock_t = void>
    static void enable(Timer_t* a_p_timer, const soc::m4::IRQ_config& a_irq_config, std::uint64_t a_start_cnt) = delete;
    template<typename Timer_t> static void disable()                                                           = delete;

    template<typename Timer_t> static void start(bool a_call_handler_on_start = false) = delete;
    template<typename Timer_t> static void stop()                                      = delete;

    static std::uint64_t get();

    static void register_callback(const Callback& a_callback);

private:
    static void update(void*);

    static inline volatile std::uint64_t cnt = 0u;
    static inline void* p_timer              = nullptr;

    static Callback callback;
};

template<> void tick_counter<Milliseconds>::enable<soc::m4::Systick>(soc::m4::Systick* a_p_timer,
                                                                             const soc::m4::IRQ_config& a_irq_config,
                                                                             std::uint64_t a_start_cnt);
template<> void tick_counter<Milliseconds>::disable<soc::m4::Systick>();

template<> void tick_counter<Milliseconds>::start<Systick>(bool a_start_handler_immediately);
template<> void tick_counter<Milliseconds>::stop<Systick>();
} // namespace utils
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu