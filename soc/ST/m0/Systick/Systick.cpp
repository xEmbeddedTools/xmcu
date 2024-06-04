/**/

// this
#include <xmcu/soc/ST/m0/Systick/Systick.hpp>

// xmcu
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m0/nvic.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace {
using namespace xmcu::soc::m0;

Systick* irq_context[1] = { nullptr };
} // namespace

extern "C" {
using namespace xmcu::soc::m0;

void SysTick_Handler()
{
    systick_interrupt_handler();
}
}

namespace xmcu {
namespace soc {
namespace m0 {
using namespace xmcu;
using namespace xmcu::debug;

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
    SysTick->VAL  = 0x0u;
    SysTick->CTRL = static_cast<std::uint32_t>(a_prescaler) | SysTick_CTRL_TICKINT_Msk;
}

void Systick::disable()
{
    if (true == this->is_enabled())
    {
        this->interrupt.disable();
    }

    SysTick->LOAD = 0x0u;
    SysTick->VAL  = 0x0u;
    SysTick->CTRL = 0x0u;
}

void Systick::start()
{
    bit_flag::set(&(SysTick->CTRL), SysTick_CTRL_ENABLE_Msk);
}

void Systick::stop()
{
    bit_flag::clear(&(SysTick->CTRL), SysTick_CTRL_ENABLE_Msk);
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

} // namespace m0
} // namespace soc
} // namespace xmcu
