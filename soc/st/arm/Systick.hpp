#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>
#include <limits>
#include <utility>

// xmcu
#include <soc/st/arm/IRQ_config.hpp>
#include <soc/peripheral.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/config.hpp>

namespace xmcu {
namespace soc {

class Systick : private Non_copyable
{
public:
    enum class Prescaler : std::uint32_t
    {
        _1 = SysTick_CTRL_CLKSOURCE_Msk,
        _8 = 0
    };

    class Polling : Non_copyable
    {
    public:
        std::uint32_t get_value() const
        {
            return static_cast<SysTick_Type*>(*(this->p_systick))->VAL;
        }

    private:
        Systick* p_systick;
        friend Systick;
    };
    class Interrupt : private Non_copyable
    {
    public:
        struct Callback
        {
            using Function = void (*)(void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        ~Interrupt()
        {
            if (true == this->is_enabled())
            {
                this->disable();
            }
        }

        void enable(const IRQ_config& a_irq_config);
        void disable();

        void register_callback(const Callback& a_callback);
        void unregister_callback();

        bool is_enabled() const
        {
            return 0 != NVIC_GetEnableIRQ(IRQn_Type::SysTick_IRQn);
        }

    private:
        Systick* p_systick;
        friend Systick;
    };

    Systick(Systick&&) = default;
    Systick& operator=(Systick&&) = default;

    constexpr Systick()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
    {
        this->polling.p_systick = nullptr;
        this->interrupt.p_systick = nullptr;
    }
    ~Systick();

    void enable(std::uint32_t a_start_value, Prescaler a_prescaler);
    void disable();

    void start();
    void stop();

    bool is_enabled()
    {
        return bit::flag::is(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
    }

    operator SysTick_Type*()
    {
        return SysTick;
    }

    operator const SysTick_Type*() const
    {
        return SysTick;
    }

    Polling polling;
    Interrupt interrupt;

private:
    constexpr Systick(std::uint32_t a_idx)
        : idx(a_idx)
    {
        this->polling.p_systick = this;
        this->interrupt.p_systick = this;
    }

    std::uint32_t idx;
    Interrupt::Callback callback;

    template<typename Periph_t, std::uint32_t periph_id> friend class soc::peripheral;
    friend void systick_interrupt_handler();
};

void systick_interrupt_handler();

} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
template<std::uint32_t id> class peripheral<Systick, id> : private non_constructible
{
public:
    static constexpr Systick create()
    {
        return Systick(0);
    }
};
} // namespace soc
} // namespace xmcu
