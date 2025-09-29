#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// externals
#include <core_cm0plus.h>

// xmcu
#include <xmcu/Non_copyable.hpp>
#include <xmcu/non_constructible.hpp>
#include <soc/Scoped_guard.hpp>
#include <xmcu/various.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace xmcu::soc::st::arm::m0 {
class nvic : private xmcu::non_constructible
{
public:
    enum class Mode : uint32_t
    {
        enabled,
        disabled
    };

    struct Config
    {
        enum class Grouping : std::uint32_t
        {
            _0 = 7,
            _1 = 6,
            _2 = 5,
        };

        Grouping grouping = various::get_enum_incorrect_value<Grouping>();
    };

    static void set_config(const Config& a_config)
    {
        hkm_assert(various::get_enum_incorrect_value<Config::Grouping>() != a_config.grouping);

        NVIC_SetPriorityGrouping(static_cast<std::uint32_t>(a_config.grouping));
    }

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
        return { .grouping = static_cast<Config::Grouping>(NVIC_GetPriorityGrouping()) };
    }

    static Mode get_mode()
    {
        return static_cast<Mode>(__get_PRIMASK());
    }
};

} // namespace xmcu::soc::st::arm::m0

namespace xmcu::soc {
template<> class Scoped_guard<st::arm::m0::nvic> : private xmcu::Non_copyable
{
public:
    Scoped_guard()
        : mode(st::arm::m0::nvic::get_mode())
    {
        st::arm::m0::nvic::set_mode(st::arm::m0::nvic::Mode::disabled);
    }

    ~Scoped_guard()
    {
        st::arm::m0::nvic::set_mode(this->mode);
    }

private:
    st::arm::m0::nvic::Mode mode;
};
} // namespace xmcu::soc
