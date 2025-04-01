/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <xmcu/config.hpp>
#include <soc/st/arm/Systick.hpp>

// xmcu
#include <soc/Scoped_guard.hpp>
#if defined(XMCU_SOC_CORE_FAMILY_M0)
#include <soc/st/arm/m0/nvic.hpp>
#elif defined(XMCU_SOC_CORE_FAMILY_M4)
#include <soc/st/arm/m4/nvic.hpp>
#endif

// debug
#include <xmcu/assertion.hpp>

namespace {
using namespace xmcu::soc;

Systick* irq_context[1] = { nullptr };
} // namespace

extern "C" {
using namespace xmcu::soc;

void SysTick_Handler()
{
    systick_interrupt_handler();
}
}

namespace xmcu::soc {
using namespace xmcu::debug;

#if defined(XMCU_SOC_CORE_FAMILY_M0)
using nvic = st::arm::m0::nvic;
#elif defined(XMCU_SOC_CORE_FAMILY_M4)
using nvic = st::arm::m4::nvic;
#endif

void systick_interrupt_handler()
{
    hkm_assert(nullptr != irq_context[0]);

    if (nullptr != irq_context[0]->callback.function)
    {
        irq_context[0]->callback.function(irq_context[0]->callback.p_user_data);
    }
}

Systick::~Systick()
{
    if (true == this->is_enabled())
    {
        this->disable();
    }
}

void Systick::enable(std::uint32_t a_start_value, Prescaler a_prescaler)
{
    hkm_assert(a_start_value > 0);

    SysTick->CTRL = 0x0u;
    SysTick->LOAD = a_start_value;
    SysTick->VAL = 0x0u;
    SysTick->CTRL = static_cast<std::uint32_t>(a_prescaler) | SysTick_CTRL_TICKINT_Msk;
}

void Systick::disable()
{
    if (true == this->is_enabled())
    {
        this->interrupt.disable();
    }

    SysTick->LOAD = 0x0u;
    SysTick->VAL = 0x0u;
    SysTick->CTRL = 0x0u;
}

void Systick::start()
{
    bit::flag::set(&(SysTick->CTRL), SysTick_CTRL_ENABLE_Msk);
}

void Systick::stop()
{
    bit::flag::clear(&(SysTick->CTRL), SysTick_CTRL_ENABLE_Msk);
}

void Systick::Interrupt::enable(const IRQ_config& a_irq_config)
{
    hkm_assert(nullptr != this->p_systick);
    hkm_assert(nullptr == irq_context[0]);

    irq_context[0] = this->p_systick;

    NVIC_SetPriority(
        SysTick_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(SysTick_IRQn);
}

void Systick::Interrupt::disable()
{
    hkm_assert(nullptr != this->p_systick);

    NVIC_DisableIRQ(SysTick_IRQn);

    irq_context[0] = nullptr;
}

void Systick::Interrupt::register_callback(const Callback& a_callback)
{
    Scoped_guard<nvic> guard;
    this->p_systick->callback = a_callback;
}

void Systick::Interrupt::unregister_callback()
{
    Scoped_guard<nvic> guard;
    this->p_systick->callback = { nullptr, nullptr };
}
} // namespace xmcu::soc