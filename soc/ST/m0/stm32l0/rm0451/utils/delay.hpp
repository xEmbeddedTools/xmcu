#pragma once

/**/

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace utils {

class delay : private xmcu::Non_constructible
{
public:
    static void wait(xmcu::Milliseconds a_time);
    static void wait(xmcu::Seconds a_time);
    static void wait(xmcu::Microseconds a_time);
};

} // namespace utils
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
