#pragma once

/**/

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/Frequency.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace sources {
class hsi48 : private Non_constructible
{
public:
    static void enable();
    static bool enable(Milliseconds a_timeout);

    static void disable();
    static bool disable(Milliseconds a_timeout);

    static bool is_enabled()
    {
        return bit_flag::is(RCC->CRRCR, RCC_CRRCR_HSI48RDY);
    }

    static std::uint32_t get_frequency_Hz()
    {
        if (true == is_enabled())
        {
            return 48_MHz;
        }

        return 0u;
    }
};
} // namespace sources
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu