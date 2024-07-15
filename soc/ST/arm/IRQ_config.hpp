#pragma once

/**/

// std
#include <cstdint>

namespace xmcu {
namespace soc {
struct IRQ_config
{
    std::uint32_t preempt_priority = 0;
    std::uint32_t sub_priority = 0;
};
} // namespace soc
} // namespace xmcu