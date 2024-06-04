#pragma once

/**/

// std
#include <cstdint>

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Non_constructible.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/internal_flash/internal_flash.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hse.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hsi16.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/msi.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/pll.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/mcu/mcu.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace system {
template<typename MCU_t> class pwr : private xmcu::Non_constructible
{
};

template<> class pwr<mcu<1u>> : private xmcu::Non_constructible
{
public:
    enum class Voltage_scaling : std::uint32_t
    {
        _1 = PWR_CR1_VOS_0,
        _2 = PWR_CR1_VOS_1
    };

    struct stop_mode : private xmcu::Non_constructible
    {
        enum class Type : std::uint32_t
        {
            _0 = 0x0u,
            _1 = PWR_CR1_LPMS_0,
            _2 = PWR_CR1_LPMS_1,
            standby = PWR_CR1_LPMS_0 | PWR_CR1_LPMS_1
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
            enabled = SCB_SCR_SLEEPONEXIT_Msk
        };

        template<typename Source_t>
        static void enter(Type a_type,
                          Method a_method,
                          Sleeponexit a_sleeponexit,
                          peripherals::internal_flash::Latency a_desired_flash_latency) = delete;
    };

    static void set_voltage_scaling(Voltage_scaling a_scaling);
    static bool set_voltage_scaling(Voltage_scaling a_scaling, xmcu::Milliseconds a_timeout);

    static Voltage_scaling get_voltage_scaling();
};

template<> class pwr<mcu<2u>> : private xmcu::Non_constructible
{
public:
    enum class Boot_after_reset_or_stop : std::uint32_t
    {
        enable = PWR_CR4_C2BOOT,
        disable = 0x0u
    };

    static void set_boot(Boot_after_reset_or_stop a_mode);
};

template<>
void pwr<mcu<1u>>::stop_mode::enter<sources::hsi16>(Type a_type,
                                                    Method a_method,
                                                    Sleeponexit a_sleeponexit,
                                                    peripherals::internal_flash::Latency a_desired_flash_latency);
} // namespace system
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu