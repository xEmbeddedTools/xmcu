#pragma once

/**/

// std
#include <cstdint>

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/non_constructible.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/various.hpp>
#include <xmcu/soc/Scoped_guard.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
class nvic : private xmcu::non_constructible
{
public:
    enum class Mode : std::uint32_t
    {
        enabled,
        disabled,
    };

    struct Config
    {
        enum class Grouping : std::uint32_t
        {
            _0 = 0x7,
            _1 = 0x6,
            _2 = 0x5,
            _3 = 0x4,
            _4 = 0x3,
        };

        Grouping grouping = various::get_enum_incorrect_value<Grouping>();
        std::uint32_t lowest_avaliable_priority = 0x0u;
    };

public:
    static void set_config(const Config& a_config)
    {
        hkm_assert(various::get_enum_incorrect_value<Config::Grouping>() != a_config.grouping);

        NVIC_SetPriorityGrouping(static_cast<std::uint32_t>(a_config.grouping));
        __set_BASEPRI(a_config.lowest_avaliable_priority << (0x8u - __NVIC_PRIO_BITS));
    }

    // TODO: rename to `void set_interrupt_handling_enabled(bool)` to be more descriptive?
    static void set_mode(Mode a_mode)
    {
        switch (a_mode)
        {
            case Mode::enabled: {
                __enable_irq();
            }
            break;

            case Mode::disabled: {
                __disable_irq();
            }
            break;
        }
    }

    static void system_reset()
    {
        NVIC_SystemReset();
    }

    static Config get_config()
    {
        return { static_cast<Config::Grouping>(NVIC_GetPriorityGrouping()), __get_BASEPRI() };
    }

    // TODO: rename to `bool is_interrupt_handling_enabled()`?
    static Mode get_mode()
    {
        return static_cast<Mode>(__get_PRIMASK());
    }
};
} // namespace m4
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
template<> class Scoped_guard<m4::nvic> : private xmcu::Non_copyable
{
public:
    Scoped_guard()
        : mode(m4::nvic::get_mode())
    {
        m4::nvic::set_mode(m4::nvic::Mode::disabled);
    }

    ~Scoped_guard()
    {
        m4::nvic::set_mode(this->mode);
    }

private:
    m4::nvic::Mode mode;
};
} // namespace soc
} // namespace xmcu
