#pragma once

/**/

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/bit_flag.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace sources {
class msi : private Non_constructible
{
public:
    enum class Frequency : std::uint32_t
    {
        _100_kHz = RCC_CR_MSIRANGE_0,
        _200_kHz = RCC_CR_MSIRANGE_1,
        _400_kHz = RCC_CR_MSIRANGE_2,
        _800_kHz = RCC_CR_MSIRANGE_3,
        _1_MHz   = RCC_CR_MSIRANGE_4,
        _2_MHz   = RCC_CR_MSIRANGE_5,
        _4_MHz   = RCC_CR_MSIRANGE_6,
        _8_MHz   = RCC_CR_MSIRANGE_7,
        _16_MHz  = RCC_CR_MSIRANGE_8,
        _24_MHz  = RCC_CR_MSIRANGE_9,
        _32_MHz  = RCC_CR_MSIRANGE_10,
        _48_MHz  = RCC_CR_MSIRANGE_11,
    };

    static void enable(Frequency a_frequency);
    static bool enable(Frequency a_frequency, Milliseconds a_timeout);

    static void disable();
    static bool disable(Milliseconds a_timeout);

    static std::uint32_t get_frequency_Hz();

    static bool is_enabled()
    {
        return bit_flag::is(RCC->CR, RCC_CR_MSIRDY);
    }
};
} // namespace sources
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu