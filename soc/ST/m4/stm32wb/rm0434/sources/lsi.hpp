#pragma once

/**/

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/Frequency.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace sources {
class lsi : private Non_constructible
{
public:
    enum class Id : std::uint32_t
    {
        _1 = 0x0u,
        _2 = 0x2u
    };

    static void enable(Id a_id);
    static bool enable(Id a_id, Milliseconds a_timeout);

    static void disable(Id a_id);
    static bool disable(Id a_id, Milliseconds a_timeout);

    static bool is_selected(Id a_id);

    static bool is_enabled(Id a_id)
    {
        return bit_flag::is(RCC->CSR, RCC_CSR_LSI1RDY);
    }

    static std::uint32_t get_frequency_Hz(Id a_id)
    {
        if (true == is_enabled(a_id))
        {
            return 32_kHz;
        }

        return 0u;
    }

    static std::uint32_t get_frequency_Hz()
    {
        if (true == is_enabled(Id::_1) || true == is_enabled(Id::_2))
        {
            return 32_kHz;
        }

        return 0u;
    }
};
} // namespace sources
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu