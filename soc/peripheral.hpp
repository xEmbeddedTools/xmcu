#pragma once

/**/

// std
#include <cstddef>
#include <cstdint>
#include <limits>

// xmcu
#include <xmcu/non_constructible.hpp>

namespace xmcu {
namespace soc {
template<typename Perihperal_t,
         std::uint32_t perihperal_id_t = std::numeric_limits<std::uint32_t>::max(),
         typename DMA_t                = void*,
         std::uint32_t DMA_id_t        = std::numeric_limits<std::uint32_t>::max()>
class peripheral : private xmcu::non_constructible
{
};
} // namespace soc
} // namespace xmcu