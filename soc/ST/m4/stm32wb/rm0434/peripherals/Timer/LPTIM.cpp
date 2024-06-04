/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/Timer/LPTIM.hpp>

// hkm
#include <xmcu/various.hpp>
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m4/nvic.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace {
using namespace xmcu::soc::m4::stm32wb::peripherals;

LPTIM* LPTIM_irq_context[2] = { nullptr, nullptr };
} // namespace

extern "C" {
void LPTIM1_IRQHandler()
{
    hkm_assert(nullptr != LPTIM_irq_context[0]);

    LPTIM_interrupt_handler(LPTIM_irq_context[0]);
}
void LPTIM2_IRQHandler()
{
    hkm_assert(nullptr != LPTIM_irq_context[1]);

    LPTIM_interrupt_handler(LPTIM_irq_context[1]);
}
}

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::utils;
using namespace utils;

void LPTIM_interrupt_handler(LPTIM* a_p_this)
{
    std::uint32_t ier = a_p_this->p_registers->IER;
    std::uint32_t isr = a_p_this->p_registers->ISR;

    if (true == bit_flag::is(ier, LPTIM_IER_ARRMIE) && true == bit_flag::is(isr, LPTIM_ISR_ARRM))
    {
        if (nullptr != a_p_this->tick_counter_callback.function)
        {
            a_p_this->tick_counter_callback.function(a_p_this->tick_counter_callback.p_user_data);
        }

        bit_flag::set(&(a_p_this->p_registers->ICR), LPTIM_ICR_ARRMCF);
    }
}

void LPTIM::disable()
{
    this->p_registers->CNT = 0x0u;
    this->p_registers->ARR = 0x0u;

    this->p_registers->CR = 0;
}

void LPTIM::Tick_counter::enable(const Prescaler a_prescaler)
{
    hkm_assert(false == bit_flag::is(this->p_LPTIM->p_registers->CR, LPTIM_CR_ENABLE));

    this->p_LPTIM->p_registers->ICR = LPTIM_ICR_CMPMCF | LPTIM_ICR_ARRMCF | LPTIM_ICR_EXTTRIGCF | LPTIM_ICR_CMPOKCF |
                                      LPTIM_ICR_ARROKCF | LPTIM_ICR_UPCF | LPTIM_ICR_DOWNCF;
    this->p_LPTIM->p_registers->CNT  = 0x0u;
    this->p_LPTIM->p_registers->CFGR = static_cast<std::uint32_t>(a_prescaler);
}

void LPTIM::Tick_counter::start(Mode a_mode, std::uint16_t a_auto_reload)
{
    bit_flag::set(&(this->p_LPTIM->p_registers->ICR), LPTIM_ICR_ARROKCF);
    bit_flag::set(&(this->p_LPTIM->p_registers->CR), LPTIM_CR_ENABLE);
    bit_flag::set(&(this->p_LPTIM->p_registers->CR), static_cast<std::uint32_t>(a_mode));

    this->p_LPTIM->p_registers->ARR = a_auto_reload;

    wait_until::all_bits_are_set(this->p_LPTIM->p_registers->ISR, LPTIM_ISR_ARROK);
}

bool LPTIM::Tick_counter::start(Mode a_mode, std::uint16_t a_auto_reload, Milliseconds a_timeout)
{
    bit_flag::set(&(this->p_LPTIM->p_registers->ICR), LPTIM_ICR_ARROKCF);
    bit_flag::set(&(this->p_LPTIM->p_registers->CR), LPTIM_CR_ENABLE);
    bit_flag::set(&(this->p_LPTIM->p_registers->CR), static_cast<std::uint32_t>(a_mode));

    NVIC_ClearPendingIRQ(this->p_LPTIM->irqn);

    this->p_LPTIM->p_registers->ARR = a_auto_reload;

    return wait_until::all_bits_are_set(this->p_LPTIM->p_registers->ISR, LPTIM_ISR_ARROK, a_timeout);
}

void LPTIM::Tick_counter::stop()
{
    bit_flag::clear(&(this->p_LPTIM->p_registers->CR), LPTIM_CR_ENABLE);
}

bool LPTIM::Tick_counter::Polling::is_overload() const
{
    bool r = bit_flag::is(this->p_LPTIM->p_registers->ISR, LPTIM_ISR_ARRM);

    if (true == r)
    {
        bit_flag::set(&(this->p_LPTIM->p_registers->ICR), LPTIM_ICR_ARRMCF);
    }

    return r;
}

void LPTIM::Tick_counter::Interrupt::enable(const IRQ_config& a_config)
{
    hkm_assert(nullptr == LPTIM_irq_context[this->p_LPTIM->idx]);

    LPTIM_irq_context[this->p_LPTIM->idx] = this->p_LPTIM;

    NVIC_SetPriority(this->p_LPTIM->irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_config.preempt_priority, a_config.sub_priority));
    NVIC_EnableIRQ(this->p_LPTIM->irqn);
}
void LPTIM::Tick_counter::Interrupt::disable()
{
    hkm_assert(nullptr != LPTIM_irq_context[this->p_LPTIM->idx]);

    this->unregister_callback();

    NVIC_DisableIRQ(this->p_LPTIM->irqn);

    LPTIM_irq_context[this->p_LPTIM->idx] = nullptr;
}

void LPTIM::Tick_counter::Interrupt::register_callback(const Callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;

    this->p_LPTIM->tick_counter_callback = a_callback;

    bit_flag::set(&(this->p_LPTIM->p_registers->IER), LPTIM_IER_ARRMIE);
}
void LPTIM::Tick_counter::Interrupt::unregister_callback()
{
    Scoped_guard<nvic> guard;

    bit_flag::clear(&(this->p_LPTIM->p_registers->IER), LPTIM_IER_ARRMIE);

    this->p_LPTIM->tick_counter_callback = { nullptr, nullptr };
}
} // namespace peripherals
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
using namespace xmcu::soc::m4::stm32wb::peripherals;
using namespace xmcu::soc::m4::stm32wb::sources;
using namespace xmcu::soc::m4::stm32wb::system;

template<> template<> void rcc<LPTIM, 1>::enable<rcc<mcu<1u>>::pclk<1u>>(bool a_enable_in_lp)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_LPTIM1SEL);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_LPTIM1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_LPTIM1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_LPTIM1SMEN);
    }
}
template<> template<> void rcc<peripherals::LPTIM, 1>::enable<lsi>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_LPTIM1SEL, RCC_CCIPR_LPTIM1SEL_0);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_LPTIM1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_LPTIM1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_LPTIM1SMEN);
    }
}
template<> template<> void rcc<peripherals::LPTIM, 1>::enable<hsi16>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_LPTIM1SEL, RCC_CCIPR_LPTIM1SEL_1);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_LPTIM1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_LPTIM1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_LPTIM1SMEN);
    }
}
template<> template<> void rcc<peripherals::LPTIM, 1>::enable<lse>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_LPTIM1SEL);
    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_LPTIM1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_LPTIM1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_LPTIM1SMEN);
    }
}

template<> template<> void rcc<peripherals::LPTIM, 2>::enable<rcc<mcu<1u>>::pclk<1u>>(bool a_enable_in_lp)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_LPTIM2SEL);
    bit_flag::set(&(RCC->APB1ENR2), RCC_APB1ENR2_LPTIM2EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPTIM2SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPTIM2SMEN);
    }
}
template<> template<> void rcc<peripherals::LPTIM, 2>::enable<lsi>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_LPTIM2SEL, RCC_CCIPR_LPTIM2SEL_0);
    bit_flag::set(&(RCC->APB1ENR2), RCC_APB1ENR2_LPTIM2EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPTIM2SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPTIM2SMEN);
    }
}
template<> template<> void rcc<peripherals::LPTIM, 2>::enable<hsi16>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_LPTIM2SEL, RCC_CCIPR_LPTIM2SEL_1);
    bit_flag::set(&(RCC->APB1ENR2), RCC_APB1ENR2_LPTIM2EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPTIM2SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPTIM2SMEN);
    }
}
template<> template<> void rcc<peripherals::LPTIM, 2>::enable<lse>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_LPTIM2SEL);
    bit_flag::set(&(RCC->APB1ENR2), RCC_APB1ENR2_LPTIM2EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPTIM2SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPTIM2SMEN);
    }
}
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif