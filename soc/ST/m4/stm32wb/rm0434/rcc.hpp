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
namespace m4 {
namespace stm32wb {
template<typename Periph_t, std::uint32_t id = std::numeric_limits<std::uint32_t>::max()> class rcc
    : private xmcu::Non_constructible
{
};
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu