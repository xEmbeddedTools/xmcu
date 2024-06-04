#pragma once

/**/

// externals
#include <stm32l0xx.h>

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Frequency.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/Non_constructible.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace sources {
class hse : private Non_constructible
{
public:
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
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
