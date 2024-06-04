#pragma once

/**/

// std
#include <cstdint>

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m4/IRQ_config.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/rcc.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lse.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lsi.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/mcu/mcu.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {
class rng : private Non_constructible
{
public:
    enum class Event_flag : std::uint32_t
    {
        none = 0x0u,
        seed_error = 0x1u,
        clock_error = 0x2u
    };

    class polling : Non_constructible
    {
    public:
        struct Result
        {
            Event_flag event = Event_flag::none;
            std::uint32_t data = 0x0u;
        };

        static Result get();
        static Result get(Milliseconds a_timeout);
    };
    class interrupt : Non_constructible
    {
    public:
        struct Callback
        {
            using Function = void (*)(std::uint32_t a_data, Event_flag a_flag, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        static void enable(const IRQ_config& a_config);
        static void disable();

        static void start(const Callback& a_callback);
        static void stop();
    };

    static void enable();
    static bool enable(Milliseconds a_timeout);

    static void disable();
    static bool disable(Milliseconds a_timeout);

private:
    friend void RNG_interrupt_handler();
};
void RNG_interrupt_handler();

constexpr rng::Event_flag operator&(rng::Event_flag a_f1, rng::Event_flag a_f2)
{
    return static_cast<rng::Event_flag>(static_cast<std::uint32_t>(a_f1) & static_cast<std::uint32_t>(a_f2));
}

constexpr rng::Event_flag operator|(rng::Event_flag a_f1, rng::Event_flag a_f2)
{
    return static_cast<rng::Event_flag>(static_cast<std::uint32_t>(a_f1) | static_cast<std::uint32_t>(a_f2));
}

constexpr rng::Event_flag operator|=(rng::Event_flag& a_f1, rng::Event_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}
} // namespace peripherals
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu
namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
template<> class rcc<peripherals::rng> : private Non_constructible
{
public:
    template<typename Source_t> static void enable(bool a_enable_in_lp) = delete;
    static void disable();
};

template<> void rcc<peripherals::rng>::enable<rcc<system::mcu<1u>>::clk48>(bool a_enable_in_lp);
template<> void rcc<peripherals::rng>::enable<sources::lsi>(bool a_enable_in_lp);
template<> void rcc<peripherals::rng>::enable<sources::lse>(bool a_enable_in_lp);
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu
