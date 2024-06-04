/**/

#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/iwdg/iwdg.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

namespace xmcu::soc::m4::stm32wb::peripherals {
using namespace xmcu::soc::m4::stm32wb::utils;

void iwdg::enable()
{
    IWDG->KR = 0x0000CCCC;       // Enable the IWDG
    IWDG->KR = 0x00005555;       // Enable register access
    IWDG->PR = IWDG_PR_PR_Msk;   // Mac prescaler: 32k/256 => 128Hz
    IWDG->RLR = IWDG_RLR_RL_Msk; // Max restart interval: 4096 ticks => 32s

    wait_until::all_bits_are_cleared(IWDG->SR, IWDG_SR_PVU_Msk | IWDG_SR_RVU_Msk | IWDG_SR_WVU_Msk);
    iwdg::feed();
    iwdg::is_wdg_active = true;
}

void iwdg::feed()
{
    IWDG->KR = 0x0000AAAA;
}

bool iwdg::is_active()
{
    return iwdg::is_wdg_active;
}

} // namespace xmcu::soc::m4::stm32wb::peripherals
