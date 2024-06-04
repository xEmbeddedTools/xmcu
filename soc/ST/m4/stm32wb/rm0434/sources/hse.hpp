#pragma once

/**/

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Frequency.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/Non_constructible.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace sources {
class hse : private Non_constructible
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = 0x0u,
        _2 = RCC_CR_HSEPRE
    };

    struct tune : private Non_constructible
    {
        using Capacitor_tuning = Limited<std::uint32_t, 0, 63>;

        enum class Current_control_max_limit : std::uint32_t
        {
            _0_18 = 0x0u,
            _0_57 = RCC_HSECR_HSEGMC0,
            _0_78 = RCC_HSECR_HSEGMC1,
            _1_13 = RCC_HSECR_HSEGMC1 | RCC_HSECR_HSEGMC1,
            _0_61 = RCC_HSECR_HSEGMC2,
            _1_65 = RCC_HSECR_HSEGMC0 | RCC_HSECR_HSEGMC2,
            _2_12 = RCC_HSECR_HSEGMC1 | RCC_HSECR_HSEGMC2,
            _2_84 = RCC_HSECR_HSEGMC0 | RCC_HSECR_HSEGMC1 | RCC_HSECR_HSEGMC2
        };

        enum class Amplifier_threshold : std::uint32_t
        {
            _1_2 = 0x0u,
            _3_4 = RCC_HSECR_HSES
        };

        static void set(Capacitor_tuning a_capacitor_tuning);
        static void set(Current_control_max_limit a_current_control_max_limit);
        static void set(Amplifier_threshold a_amplifier_threshold);

        static Capacitor_tuning get_Capacitor_tuning();
        static Current_control_max_limit get_Current_control_max_limit();
        static Amplifier_threshold get_amplifier_threshold();
    };

    static void enable();
    static bool enable(Milliseconds a_timeout);

    static void disable();
    static bool disable(Milliseconds a_timeout);

    static bool is_enabled()
    {
        return bit_flag::is(RCC->CR, RCC_CR_HSERDY);
    }

    static std::uint32_t get_frequency_Hz()
    {
        if (true == is_enabled())
        {
            return 32_MHz;
        }

        return 0u;
    }
};
} // namespace sources
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu
