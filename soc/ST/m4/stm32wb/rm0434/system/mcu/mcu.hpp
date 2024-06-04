#pragma once

/**/

// std
#include <cstddef>
#include <cstdint>

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Non_constructible.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/rcc.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hse.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hsi16.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hsi48.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lse.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/pll.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace system {
template<std::size_t id> class mcu : private Non_constructible
{
};
template<> class mcu<1u> : private Non_constructible
{
public:
    enum class FPU_mode : std::uint32_t
    {
        disabled = 0x000000u,
        privileged_access_only = 0xA00000u,
        enabled = 0xF00000u,
    };
    enum class DWT_mode : std::uint32_t
    {
        disabled,
        enabled
    };

    enum class Reset_source : std::uint32_t
    {
        low_power = RCC_CSR_LPWRRSTF,
        window_watchdog = RCC_CSR_WWDGRSTF,
        independent_window_watchdog = RCC_CSR_IWDGRSTF,
        software = RCC_CSR_SFTRSTF,
        bor = RCC_CSR_BORRSTF,
        pin = RCC_CSR_PINRSTF,
        option_byte = RCC_CSR_OBLRSTF
    };

    enum class Package : std::uint32_t
    {
        WLCSP100_or_UFBGA129 = 0x11u,
        VFQFPN68 = 0x13u,
        UFQPFN48 = 0xAu
    };

    struct Id
    {
        std::uint8_t serial_number[12] = { 0 };
        std::uint32_t type = 0;
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

    static void set_DWT_mode(DWT_mode a_mode)
    {
        switch (a_mode)
        {
            case DWT_mode::enabled: {
                bit_flag::set(&(CoreDebug->DEMCR), CoreDebug_DEMCR_TRCENA_Msk);
                bit_flag::set(&(DWT->CTRL), DWT_CTRL_CYCCNTENA_Msk);
            }
            break;

            case DWT_mode::disabled: {
                bit_flag::clear(&(CoreDebug->DEMCR), CoreDebug_DEMCR_TRCENA_Msk);
                bit_flag::clear(&(DWT->CTRL), DWT_CTRL_CYCCNTENA_Msk);
            }
            break;
        }
    }

    static void set_FPU_mode(FPU_mode a_mode)
    {
        bit_flag::set(&(SCB->CPACR), ((3u << 10u * 2u) | (3u << 11u * 2u)), static_cast<uint32_t>(a_mode));
    }

    static DWT_mode get_DWT_mode()
    {
        return static_cast<DWT_mode>(bit_flag::is(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk));
    }

    static FPU_mode get_FPU_mode()
    {
        return static_cast<FPU_mode>(SCB->CPACR);
    }

    static Reset_source get_reset_source()
    {
        auto l_reader = []() {
            volatile std::uint32_t result { RCC->CSR };
            RCC->CSR = result | RCC_CSR_RMVF;
            return static_cast<Reset_source>(result & 0xFE000000u);
        };
        static Reset_source stored_value = l_reader();
        return stored_value;
    }

    static constexpr Reset_source flags[] = {
        Reset_source::pin,
        Reset_source::bor,
        Reset_source::window_watchdog,
        Reset_source::independent_window_watchdog,
        Reset_source::software,
        Reset_source::low_power,
        Reset_source::option_byte,
    };

    static constexpr Package get_package()
    {
        return static_cast<Package>(*(reinterpret_cast<std::uint32_t*>(PACKAGE_BASE)));
    }

    static bool is_in_debug_mode()
    {
        return bit_flag::is(CoreDebug->DHCSR, CoreDebug_DHCSR_C_DEBUGEN_Msk);
    }

    static std::uint32_t get_unique_device_number()
    {
        return static_cast<std::uint32_t>(*(reinterpret_cast<std::uint32_t*>(UID64_BASE)));
    }

    static std::uint32_t get_manufacturer_id()
    {
        return static_cast<std::uint32_t>(((*reinterpret_cast<uint32_t*>(UID64_BASE) + 1u) >> 8u) & 0x00FFFFFFu);
    }

    static std::uint32_t get_device_id()
    {
        return static_cast<std::uint32_t>((*(reinterpret_cast<uint32_t*>(UID64_BASE) + 1U)) & 0x000000FFu);
    }
};

const char* to_string(mcu<1u>::Reset_source);

static inline std::uint32_t operator~(const mcu<1u>::Reset_source& a_rhs)
{
    return ~static_cast<std::uint32_t>(a_rhs);
}

static inline mcu<1u>::Reset_source operator&(const mcu<1u>::Reset_source& a_lhs, const mcu<1u>::Reset_source& a_rhs)
{
    return static_cast<mcu<1u>::Reset_source>(static_cast<std::uint32_t>(a_lhs) & static_cast<std::uint32_t>(a_rhs));
}

static inline mcu<1u>::Reset_source& operator&=(mcu<1u>::Reset_source& a_lhs, const std::uint32_t& a_rhs)
{
    a_lhs = static_cast<mcu<1u>::Reset_source>(~~a_lhs & a_rhs);
    return a_lhs;
}

static inline bool operator==(const std::uint32_t& a_lhs, const mcu<1u>::Reset_source& a_rhs)
{
    return a_lhs == static_cast<std::uint32_t>(a_rhs);
}

} // namespace system
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
template<> class rcc<system::mcu<1u>> : private Non_constructible
{
public:
    template<std::size_t id> class hclk : private Non_constructible
    {
    };
    template<std::size_t id> class pclk : private Non_constructible
    {
    };

    class clk48 : private Non_constructible
    {
    public:
        template<typename Source_t> static void set() = delete;
        template<typename Source_t> static bool set(Milliseconds a_timeout) = delete;

        template<typename Source_t> static bool is_source() = delete;
        static std::uint32_t get_frequency_Hz();
    };

    template<typename Source_t> static void set_system_clock_source() = delete;
    template<typename Source_t> static bool set_system_clock_source(Milliseconds a_timeout) = delete;

    template<typename Source_t, sources::hse::Prescaler> static void set_system_clock_source() = delete;
    template<typename Source_t, sources::hse::Prescaler>
    static bool set_system_clock_source(Milliseconds a_timeout) = delete;

    template<typename Source_t> static bool is_system_clock_source() = delete;
    static std::uint32_t get_system_clock_frequency_Hz();

    template<typename Source_t> static void set_wakeup_clock_source() = delete;
};

template<> class rcc<system::mcu<1u>>::hclk<1u> : private Non_constructible
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = 0x0u,
        _3 = RCC_CFGR_HPRE_0,
        _5 = RCC_CFGR_HPRE_1,
        _6 = RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_2,
        _10 = RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_2,
        _32 = RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_2,
        _2 = RCC_CFGR_HPRE_3,
        _4 = RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_3,
        _8 = RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_3,
        _16 = RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_3,
        _64 = RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_3,
        _128 = RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_3,
        _256 = RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_3,
        _512 = RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_3
    };

    static void set(Prescaler a_prescaler);
    static bool set(Prescaler a_prescaler, Milliseconds a_timeout);

    static Prescaler get_Prescaler();
    static std::uint32_t get_frequency_Hz();
};
template<> class rcc<system::mcu<1u>>::hclk<2u> : private Non_constructible
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = 0x0u,
        _3 = RCC_EXTCFGR_C2HPRE_0,
        _5 = RCC_EXTCFGR_C2HPRE_1,
        _6 = RCC_EXTCFGR_C2HPRE_0 | RCC_EXTCFGR_C2HPRE_2,
        _10 = RCC_EXTCFGR_C2HPRE_1 | RCC_EXTCFGR_C2HPRE_2,
        _32 = RCC_EXTCFGR_C2HPRE_0 | RCC_EXTCFGR_C2HPRE_1 | RCC_EXTCFGR_C2HPRE_2,
        _2 = RCC_EXTCFGR_C2HPRE_3,
        _4 = RCC_EXTCFGR_C2HPRE_0 | RCC_EXTCFGR_C2HPRE_3,
        _8 = RCC_EXTCFGR_C2HPRE_1 | RCC_EXTCFGR_C2HPRE_3,
        _16 = RCC_EXTCFGR_C2HPRE_0 | RCC_EXTCFGR_C2HPRE_1 | RCC_EXTCFGR_C2HPRE_3,
        _64 = RCC_EXTCFGR_C2HPRE_2 | RCC_EXTCFGR_C2HPRE_3,
        _128 = RCC_EXTCFGR_C2HPRE_0 | RCC_EXTCFGR_C2HPRE_2 | RCC_EXTCFGR_C2HPRE_3,
        _256 = RCC_EXTCFGR_C2HPRE_1 | RCC_EXTCFGR_C2HPRE_2 | RCC_EXTCFGR_C2HPRE_3,
        _512 = RCC_EXTCFGR_C2HPRE_0 | RCC_EXTCFGR_C2HPRE_1 | RCC_EXTCFGR_C2HPRE_2 | RCC_EXTCFGR_C2HPRE_3
    };

    static void set(Prescaler a_prescaler);
    static bool set(Prescaler a_prescaler, Milliseconds a_timeout);

    static std::uint32_t get_frequency_Hz();
    static Prescaler get_Prescaler();
};
template<> class rcc<system::mcu<1u>>::hclk<4u> : private Non_constructible
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = 0x0u,
        _3 = RCC_EXTCFGR_SHDHPRE_0,
        _5 = RCC_EXTCFGR_SHDHPRE_1,
        _6 = RCC_EXTCFGR_SHDHPRE_0 | RCC_EXTCFGR_SHDHPRE_2,
        _10 = RCC_EXTCFGR_SHDHPRE_1 | RCC_EXTCFGR_SHDHPRE_2,
        _32 = RCC_EXTCFGR_SHDHPRE_0 | RCC_EXTCFGR_SHDHPRE_1 | RCC_EXTCFGR_SHDHPRE_2,
        _2 = RCC_EXTCFGR_SHDHPRE_3,
        _4 = RCC_EXTCFGR_SHDHPRE_0 | RCC_EXTCFGR_SHDHPRE_3,
        _8 = RCC_EXTCFGR_SHDHPRE_1 | RCC_EXTCFGR_SHDHPRE_3,
        _16 = RCC_EXTCFGR_SHDHPRE_0 | RCC_EXTCFGR_SHDHPRE_1 | RCC_EXTCFGR_SHDHPRE_3,
        _64 = RCC_EXTCFGR_SHDHPRE_2 | RCC_EXTCFGR_SHDHPRE_3,
        _128 = RCC_EXTCFGR_SHDHPRE_0 | RCC_EXTCFGR_SHDHPRE_2 | RCC_EXTCFGR_SHDHPRE_3,
        _256 = RCC_EXTCFGR_SHDHPRE_1 | RCC_EXTCFGR_SHDHPRE_2 | RCC_EXTCFGR_SHDHPRE_3,
        _512 = RCC_EXTCFGR_SHDHPRE_0 | RCC_EXTCFGR_SHDHPRE_1 | RCC_EXTCFGR_SHDHPRE_2 | RCC_EXTCFGR_SHDHPRE_3
    };

    static void set(Prescaler a_prescaler);
    static bool set(Prescaler a_prescaler, Milliseconds a_timeout);

    static Prescaler get_Prescaler();
    static std::uint32_t get_frequency_Hz();
};
template<> class rcc<system::mcu<1u>>::hclk<5u> : private Non_constructible
{
public:
    template<typename Source_t> static void set() = delete;
    template<typename Source_t, sources::hse::Prescaler> static void set() = delete;

    static std::uint32_t get_frequency_Hz()
    {
        return 16_MHz;
    }
};

template<> class rcc<system::mcu<1u>>::pclk<1u> : private Non_constructible
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = 0x0u,
        _2 = RCC_CFGR_PPRE1_2,
        _4 = RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_2,
        _8 = RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE1_2,
        _16 = RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE1_2,
    };

    static void set(Prescaler a_prescaler);
    static bool set(Prescaler a_prescaler, Milliseconds a_timeout);

    static Prescaler get_Prescaler();
    static std::uint32_t get_frequency_Hz();
    static std::uint32_t get_Tim_frequency_Hz();
};
template<> class rcc<system::mcu<1u>>::pclk<2u> : private Non_constructible
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = 0x0u,
        _2 = RCC_CFGR_PPRE2_2,
        _4 = RCC_CFGR_PPRE2_0 | RCC_CFGR_PPRE2_2,
        _8 = RCC_CFGR_PPRE2_1 | RCC_CFGR_PPRE2_2,
        _16 = RCC_CFGR_PPRE2_0 | RCC_CFGR_PPRE2_1 | RCC_CFGR_PPRE2_2,
    };

    static void set(Prescaler a_prescaler);
    static bool set(Prescaler a_prescaler, Milliseconds a_timeout);

    static Prescaler get_Prescaler();
    static std::uint32_t get_frequency_Hz();
    static std::uint32_t get_Tim_frequency_Hz();
};

template<> class rcc<system::mcu<2u>> : private Non_constructible
{
public:
    template<typename Source_t> static void set_wakeup_clock_source() = delete;
};

template<> void rcc<system::mcu<1u>>::hclk<5u>::set<sources::hsi16>();
template<> void rcc<system::mcu<1u>>::hclk<5u>::set<sources::hse, sources::hse::Prescaler::_2>();

template<> void rcc<system::mcu<1u>>::clk48::set<sources::hsi48>();
template<> bool rcc<system::mcu<1u>>::clk48::set<sources::hsi48>(Milliseconds a_timeout);
template<> bool rcc<system::mcu<1u>>::clk48::is_source<sources::hsi48>();
template<> void rcc<system::mcu<1u>>::clk48::set<sources::msi>();
template<> bool rcc<system::mcu<1u>>::clk48::set<sources::msi>(Milliseconds a_timeout);
template<> bool rcc<system::mcu<1u>>::clk48::is_source<sources::msi>();
template<> void rcc<system::mcu<1u>>::clk48::set<sources::pll::q>();
template<> bool rcc<system::mcu<1u>>::clk48::set<sources::pll::q>(Milliseconds a_timeout);
template<> bool rcc<system::mcu<1u>>::clk48::is_source<sources::pll::q>();
template<> void rcc<system::mcu<1u>>::clk48::set<sources::pll::sai1::q>();
template<> bool rcc<system::mcu<1u>>::clk48::set<sources::pll::sai1::q>(Milliseconds a_timeout);
template<> bool rcc<system::mcu<1u>>::clk48::is_source<sources::pll::sai1::q>();

template<> void rcc<system::mcu<1u>>::set_system_clock_source<sources::msi>();
template<> bool rcc<system::mcu<1u>>::set_system_clock_source<sources::msi>(Milliseconds a_timeout);
template<> bool rcc<system::mcu<1>>::is_system_clock_source<sources::msi>();

template<> void rcc<system::mcu<1u>>::set_system_clock_source<sources::hsi16>();
template<> bool rcc<system::mcu<1u>>::set_system_clock_source<sources::hsi16>(Milliseconds a_timeout);
template<> bool rcc<system::mcu<1>>::is_system_clock_source<sources::hsi16>();

template<> void rcc<system::mcu<1u>>::set_system_clock_source<sources::pll>();
template<> bool rcc<system::mcu<1u>>::set_system_clock_source<sources::pll>(Milliseconds a_timeout);
template<> bool rcc<system::mcu<1>>::is_system_clock_source<sources::pll>();

template<> void rcc<system::mcu<1u>>::set_system_clock_source<sources::hse, sources::hse::Prescaler::_1>();
template<> void rcc<system::mcu<1u>>::set_system_clock_source<sources::hse, sources::hse::Prescaler::_2>();

template<> bool rcc<system::mcu<1u>>::set_system_clock_source<sources::hse, sources::hse::Prescaler::_1>(
    Milliseconds a_timeout);
template<> bool rcc<system::mcu<1u>>::set_system_clock_source<sources::hse, sources::hse::Prescaler::_2>(
    Milliseconds a_timeout);

template<> bool rcc<system::mcu<1>>::is_system_clock_source<sources::hse>();

template<> void rcc<system::mcu<1u>>::set_wakeup_clock_source<sources::msi>();
template<> void rcc<system::mcu<1u>>::set_wakeup_clock_source<sources::hsi16>();

template<> void rcc<system::mcu<2u>>::set_wakeup_clock_source<sources::lse>();
template<> void rcc<system::mcu<2u>>::set_wakeup_clock_source<sources::hse>();
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu
