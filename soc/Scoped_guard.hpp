#pragma once

/**/

// xmcu
#include <xmcu/Non_constructible.hpp>

namespace xmcu {
namespace soc {
template<typename Lock_t> class Scoped_guard : private xmcu::Non_constructible
{
};
} // namespace soc
} // namespace xmcu