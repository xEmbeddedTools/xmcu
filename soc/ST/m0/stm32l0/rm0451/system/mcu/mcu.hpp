#pragma once

/**/

// std
#include <cstddef>
#include <cstdint>

// externals
#include <stm32l0xx.h>

// xmcu
#include <xmcu/Non_constructible.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/rcc.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/sources/hse.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/sources/hsi16.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/sources/msi.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace system {
template<std::size_t id> class mcu : private Non_constructible
{
};
template<> class mcu<1u> : private Non_constructible
{
public:
    enum class Reset_source : std::uint32_t
    {
        low_power                   = RCC_CSR_LPWRRSTF,
        window_watchdog             = RCC_CSR_WWDGRSTF,
        independent_window_watchdog = RCC_CSR_IWDGRSTF,
        software                    = RCC_CSR_SFTRSTF,
        power_on                    = RCC_CSR_PORRSTF,
        pin                         = RCC_CSR_PINRSTF,
        option_byte                 = RCC_CSR_OBLRSTF
    };

    struct Id
    {
        std::uint8_t serial_number[12] = { 0 };
        std::uint32_t type             = 0;
    };

    static void halt()
    {
        __disable_irq();
        __builtin_trap();

        while (true) continue;
    }

    static Id get_id()
    {
        const std::uint8_t* p_id_location = reinterpret_cast<std::uint8_t*>(UID_BASE);

        return { { p_id_location[0],
                   p_id_location[1],
                   p_id_location[2],
                   p_id_location[3],
                   p_id_location[4],
                   p_id_location[5],
                   p_id_location[6],
                   p_id_location[7],
                   p_id_location[8],
                   p_id_location[9],
                   p_id_location[10],
                   p_id_location[11] },

                 DBGMCU->IDCODE };
    }

    static void set_vector_table_address(std::uint32_t a_address)
    {
        SCB->VTOR = a_address;
    }

    static void set_main_stack_pointer(std::uint32_t a_address)
    {
        __set_MSP(a_address);
    }

    static Reset_source get_reset_source()
    {
        return static_cast<Reset_source>(RCC->CSR & 0xFE000000u);
    }

    static std::uint32_t get_unique_device_number()
    {
        return static_cast<std::uint32_t>((*(reinterpret_cast<uint32_t*>(UID_BASE + 0x14))));
    }
};
} // namespace system
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
template<> class rcc<system::mcu<1u>> : private Non_constructible
{
public:
    template<std::size_t id> class hclk : private Non_constructible
    {
    };
    template<std::size_t id> class pclk : private Non_constructible
    {
    };

    template<typename Source_t> static void set_system_clock_source()                               = delete;
    template<typename Source_t> static bool set_system_clock_source(Milliseconds a_timeout) = delete;
    static bool set_system_clock_source(Milliseconds a_timeout)                             = delete;

    template<typename Source_t> static bool is_system_clock_source() = delete;
    static std::uint32_t get_system_clock_frequency_Hz();

    template<typename Source_t> static void set_wakeup_clock_source() = delete;
};

template<> class rcc<system::mcu<1u>>::hclk<1u> : private Non_constructible
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1   = RCC_CFGR_HPRE_DIV1,
        _2   = RCC_CFGR_HPRE_DIV2,
        _4   = RCC_CFGR_HPRE_DIV4,
        _8   = RCC_CFGR_HPRE_DIV8,
        _16  = RCC_CFGR_HPRE_DIV16,
        _64  = RCC_CFGR_HPRE_DIV64,
        _128 = RCC_CFGR_HPRE_DIV128,
        _256 = RCC_CFGR_HPRE_DIV256,
        _512 = RCC_CFGR_HPRE_DIV512,
    };

    static void set(Prescaler a_prescaler);
    static bool set(Prescaler a_prescaler, Milliseconds a_timeout);

    static Prescaler get_Prescaler();
    static std::uint32_t get_frequency_Hz();
};

template<> class rcc<system::mcu<1u>>::pclk<1u> : private Non_constructible
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1  = 0x0u,
        _2  = RCC_CFGR_PPRE1_2,
        _4  = RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_2,
        _8  = RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE1_2,
        _16 = RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE1_2,
    };

    static void set(Prescaler a_prescaler);
    static bool set(Prescaler a_prescaler, Milliseconds a_timeout);

    static Prescaler get_Prescaler();
    static std::uint32_t get_frequency_Hz();
};
template<> class rcc<system::mcu<1u>>::pclk<2u> : private Non_constructible
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1  = 0x0u,
        _2  = RCC_CFGR_PPRE2_2,
        _4  = RCC_CFGR_PPRE2_0 | RCC_CFGR_PPRE2_2,
        _8  = RCC_CFGR_PPRE2_1 | RCC_CFGR_PPRE2_2,
        _16 = RCC_CFGR_PPRE2_0 | RCC_CFGR_PPRE2_1 | RCC_CFGR_PPRE2_2,
    };

    static void set(Prescaler a_prescaler);
    static bool set(Prescaler a_prescaler, Milliseconds a_timeout);

    static Prescaler get_Prescaler();
    static std::uint32_t get_frequency_Hz();
};

template<> class rcc<system::mcu<2u>> : private Non_constructible
{
public:
    template<typename Source_t> static void set_wakeup_clock_source() = delete;
};

template<> void rcc<system::mcu<1u>>::set_system_clock_source<sources::msi>();
template<> bool rcc<system::mcu<1u>>::set_system_clock_source<sources::msi>(Milliseconds a_timeout);
template<> bool rcc<system::mcu<1u>>::is_system_clock_source<sources::msi>();

template<> void rcc<system::mcu<1u>>::set_system_clock_source<sources::hsi16>();
template<> bool rcc<system::mcu<1u>>::set_system_clock_source<sources::hsi16>(Milliseconds a_timeout);
template<> bool rcc<system::mcu<1u>>::is_system_clock_source<sources::hsi16>();

template<> void rcc<system::mcu<1u>>::set_system_clock_source<sources::hse>();
template<> bool rcc<system::mcu<1u>>::set_system_clock_source<sources::hse>(Milliseconds a_timeout);
template<> bool rcc<system::mcu<1>>::is_system_clock_source<sources::hse>();

template<> void rcc<system::mcu<1u>>::set_wakeup_clock_source<sources::msi>();
template<> void rcc<system::mcu<1u>>::set_wakeup_clock_source<sources::hsi16>();

} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
