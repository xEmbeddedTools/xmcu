#pragma once

/**/

// external
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/bit_flag.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace sources {
class lse : private Non_constructible
{
public:
    struct xtal : private Non_constructible
    {
        enum class Drive : std::uint32_t
        {
            low         = 0x0u,
            medium_low  = RCC_BDCR_LSEDRV_0,
            medium_high = RCC_BDCR_LSEDRV_1,
            high        = RCC_BDCR_LSEDRV_0 | RCC_BDCR_LSEDRV_1
        };

        static void enable(Drive a_drive_config);
        static bool enable(Drive a_drive_config, Milliseconds a_timeout);
    };

    struct bypass : private Non_constructible
    {
        static void enable();
        static bool enable(Milliseconds a_timeout);
    };

    static void disable();
    static bool disable(Milliseconds a_timeout);

    static bool is_enabled()
    {
        return bit_flag::is(RCC->BDCR, RCC_BDCR_LSERDY);
    }

    static std::uint32_t get_frequency_Hz()
    {
        if (true == is_enabled())
        {
            return 32768u;
        }

        return 0u;
    }
};
} // namespace sources
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu