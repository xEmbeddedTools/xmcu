#pragma once

/**/

// std
#include <cstdint>

namespace xmcu {
namespace soc {
namespace m0 {
struct IRQ_config
{
    std::uint32_t preempt_priority = 0;
    std::uint32_t sub_priority     = 0;
};
} // namespace m0
} // namespace soc
} // namespace xmcu
