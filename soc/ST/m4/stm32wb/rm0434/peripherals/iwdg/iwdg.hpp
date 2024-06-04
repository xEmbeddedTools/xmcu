#pragma once

/**/

#include <xmcu/Non_constructible.hpp>

namespace xmcu::soc::m4::stm32wb::peripherals {

class iwdg : private xmcu::Non_constructible
{
public:
    static void enable();
    static void feed();
    static bool is_active();

private:
    static inline bool is_wdg_active = false;
};

} // namespace xmcu::soc::m4::stm32wb::peripherals
