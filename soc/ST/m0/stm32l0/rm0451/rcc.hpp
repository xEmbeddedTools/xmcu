#pragma once

/**/

// std
#include <cstddef>
#include <cstdint>
#include <limits>

// xmcu
#include <xmcu/Non_constructible.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {

template<typename Periph_t, std::uint32_t id = std::numeric_limits<std::uint32_t>::max()> class rcc
    : private xmcu::Non_constructible
{
};

} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
