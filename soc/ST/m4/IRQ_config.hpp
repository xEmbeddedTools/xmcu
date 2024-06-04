#pragma once

/**/

// std
#include <cstdint>

namespace xmcu {
namespace soc {
namespace m4 {
struct IRQ_config
{
    std::uint32_t preempt_priority = 0;
    std::uint32_t sub_priority     = 0;
};
} // namespace m4
} // namespace soc
} // namespace xmcu