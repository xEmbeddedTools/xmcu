#pragma once

/**/

// std
#include <cstdint>

// externals
#include <stm32l0xx.h>

// xmcu
#include <xmcu/Non_constructible.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/sources/hsi16.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/sources/msi.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/system/mcu/mcu.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace system {

template<typename MCU_t> class pwr : private Non_constructible
{
};

template<> class pwr<mcu<1u>> : private Non_constructible
{
public:
    enum class Voltage_scaling : std::uint32_t
    {
        _1 = PWR_CR_VOS_0,
        _2 = PWR_CR_VOS_1,
        _3 = PWR_CR_VOS
    };

    struct stop_mode : private Non_constructible
    {
        enum class Type : std::uint32_t
        {
            stop    = 0,
            standby = PWR_CR_PDDS,
        };
        enum class Method : std::uint32_t
        {
            wfi,
            wfe,
            none
        };
        enum class Sleeponexit : std::uint32_t
        {
            disabled = 0x0u,
            enabled  = SCB_SCR_SLEEPONEXIT_Msk
        };

        static void enter(Type a_type, Method a_method, Sleeponexit a_sleeponexit);
        static void exit();
    };

    static void set_voltage_scaling(Voltage_scaling a_scaling);
    static bool set_voltage_scaling(Voltage_scaling a_scaling, Milliseconds a_timeout);

    static Voltage_scaling get_voltage_scaling();
};

} // namespace system
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu

namespace xmcu::soc::m0::stm32l0::rm0451 {

template<> class rcc<system::pwr<system::mcu<1u>>> : private Non_constructible
{
public:
    static void enable()
    {
        bit_flag::set(&RCC->APB1ENR, RCC_APB1ENR_PWREN);
    }
    static void disable()
    {
        bit_flag::clear(&RCC->APB1ENR, RCC_APB1ENR_PWREN);
    }
};

} // namespace xmcu::m0::stm32l0::rm0451
