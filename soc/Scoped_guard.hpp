#pragma once

/**/

// xmcu
#include <xmcu/non_constructible.hpp>

namespace xmcu {
namespace soc {
template<typename Lock_t> class Scoped_guard : private xmcu::non_constructible
{
};
} // namespace soc
} // namespace xmcu