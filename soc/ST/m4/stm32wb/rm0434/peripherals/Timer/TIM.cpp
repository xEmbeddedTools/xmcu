/**/

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/Timer/TIM.hpp>

// xmcu
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m4/nvic.hpp>

// Tim_counter & derived
namespace xmcu::soc::m4::stm32wb::peripherals::timer {
using namespace xmcu;

bool Polling::is_overload() const
{
    if (bit_flag::is(p_registers->SR, TIM_SR_UIF))
    {
        p_registers->SR = ~TIM_SR_UIF; // without or-assign (|=) - rc_w0
        return true;
    }
    return false;
}

Tim_counter::Tim_counter(TIM_TypeDef* a_p_driver)
    : p_registers { a_p_driver }
    , polling { a_p_driver }
{
}

Tim_counter::~Tim_counter()
{
    if (true == this->is_enabled())
    {
        this->disable();
    }
}

void Tim_counter::start(Mode a_mode) const
{
    bit_flag::set(&this->p_registers->CR1, TIM_CR1_CEN);
    this->p_registers->EGR = TIM_EGR_UG; // update event - transfer to shadow register and reset
    bit_flag::set(&this->p_registers->CR1, static_cast<std::uint32_t>(a_mode));
}

void Tim_counter::stop() const
{
    bit_flag::clear(&this->p_registers->CR1, TIM_CR1_OPM | TIM_CR1_CEN);
}

void Tim_counter::enable(Tmr_divider a_div, Count_Mode a_mode, std::uint16_t a_psc) const
{
    this->p_registers->CR1 = TIM_CR1_URS | // prevent set flags in SR (that implies dma and interrupts) when set
                                           // TIM_EGR_UG or by external trigger.
                                           // only overflow / underflow causes
                             static_cast<std::uint32_t>(a_mode) | static_cast<std::uint32_t>(a_div);
    this->p_registers->CR2 = 0;
    this->p_registers->SMCR = 0;
    this->p_registers->PSC = a_psc - 1; // The counter clock frequency (CK_CNT) is equal to fCK_PSC / (PSC[15:0] + 1).
}

void Tim_counter::disable() const
{
    this->p_registers->CR1 = 0; // first step - disable
    this->p_registers->CR2 = 0;
    this->p_registers->CNT = 0x0u;
    this->p_registers->ARR = 0x0u;
    this->p_registers->SMCR = 0;
}

void Tim_advanced::start(Mode a_mode, Counter_word_t a_arr) const
{
    this->stop();
    this->p_registers->ARR = a_arr;
    bit_flag::set(&this->p_registers->CR1, TIM_CR1_CEN);
    // MOE (Main output enable) is set automatically when update (TIM_EGR_UG)
    bit_flag::set(&this->p_registers->BDTR, TIM_BDTR_AOE);
    // generate update event - transfer to shadow register and reset
    this->p_registers->EGR = TIM_EGR_UG;
    // one pulse mode will stop counter after EGR_UG, so is set after (EGR_UG)
    bit_flag::set(&this->p_registers->CR1, static_cast<std::uint32_t>(a_mode));
}

void Tim_advanced::stop() const
{
    bit_flag::clear(&this->p_registers->BDTR, TIM_BDTR_MOE | TIM_BDTR_AOE);
    Tim_counter::stop();
}
void Tim_advanced::enable(Tmr_divider a_div, Count_Mode a_mode, std::uint16_t a_psc) const
{
    bit_flag::set(&this->p_registers->BDTR, TIM_BDTR_OSSI | TIM_BDTR_OSSR);
    Tim_counter::enable(a_div, a_mode, a_psc);
}
} // namespace xmcu::soc::m4::stm32wb::peripherals::timer

namespace xmcu::soc::m4::stm32wb::peripherals {
using namespace xmcu;

void TIM_ADV::config_output_idle_state(GPIO::Level a_lvl)
{
    std::uint32_t all_bits = TIM_CR2_OIS1 | TIM_CR2_OIS1N | TIM_CR2_OIS2 | TIM_CR2_OIS2N | TIM_CR2_OIS3 |
                             TIM_CR2_OIS3N | TIM_CR2_OIS4 | TIM_CR2_OIS5 | TIM_CR2_OIS6;
    switch (a_lvl)
    {
        case GPIO::Level::low:
            bit_flag::clear(&this->tick_counter.p_registers->CR2, all_bits);
            break;
        case GPIO::Level::high:
            bit_flag::set(&this->tick_counter.p_registers->CR2, all_bits);
            break;
    }
}
} // namespace xmcu::soc::m4::stm32wb::peripherals

// interrupts
namespace xmcu::soc::m4::stm32wb::peripherals::timer {

struct Interrupt_constants
{
    IRQn_Type irq;
    TIM_TypeDef* driver;
    std::uint32_t dier;
    std::uint32_t sr;
};

// TIM_DIER_CCxIE enable TIM_SR_CCxIF | TIM_SR_CCxOF ... but I dont know who enable: TIM_SR_CC5IF | TIM_SR_CC6IF;
// TIM_DIER_BIE enable: TIM_SR_BIF | TIM_SR_B2IF | TIM_SR_SBIF
static constexpr std::uint32_t flags_sr_capture_compare = TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF |
                                                          TIM_SR_CC1OF | TIM_SR_CC2OF | TIM_SR_CC3OF | TIM_SR_CC4OF |
                                                          TIM_SR_CC5IF | TIM_SR_CC6IF;
static constexpr std::uint32_t flags_dier_capture_compare =
    TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;
static constexpr std::uint32_t flags_sr_all =
    flags_sr_capture_compare | TIM_SR_UIF | TIM_SR_COMIF | TIM_SR_TIF | TIM_SR_BIF | TIM_SR_B2IF | TIM_SR_SBIF;
static constexpr std::uint32_t flags_dier_all =
    flags_dier_capture_compare | TIM_DIER_UIE | TIM_DIER_COMIE | TIM_DIER_TIE | TIM_DIER_BIE;
Interrupt* handlers[static_cast<std::uint32_t>(TIM_irq::count)] { nullptr };

static constexpr Interrupt_constants irq_constants[static_cast<std::uint32_t>(TIM_irq::count)] {
    { /* TIM1_BRK    */
      .irq = TIM1_BRK_IRQn,
      .driver = TIM1,
      .dier = TIM_DIER_BIE,
      .sr = TIM_SR_BIF | TIM_SR_B2IF | TIM_SR_SBIF },
    { /* TIM1_UP     */
      .irq = TIM1_UP_TIM16_IRQn,
      .driver = TIM1,
      .dier = TIM_DIER_UIE,
      .sr = TIM_SR_UIF },
    { /* TIM16_IRQ   */
      .irq = TIM1_UP_TIM16_IRQn,
      .driver = TIM16,
      .dier = flags_dier_all,
      .sr = flags_sr_all },
    { /* TIM1_TRG    */
      .irq = TIM1_TRG_COM_TIM17_IRQn,
      .driver = TIM1,
      .dier = TIM_DIER_TIE | TIM_DIER_COMIE,
      .sr = TIM_SR_TIF | TIM_SR_COMIF },
    { /* TIM17_IRQ   */
      .irq = TIM1_TRG_COM_TIM17_IRQn,
      .driver = TIM17,
      .dier = flags_dier_all,
      .sr = flags_sr_all },
    { /* TIM1_CC     */
      .irq = TIM1_CC_IRQn,
      .driver = TIM1,
      .dier = flags_dier_capture_compare,
      .sr = flags_sr_capture_compare },
    { /* TIM2_IRQ    */
      .irq = TIM2_IRQn,
      .driver = TIM2,
      .dier = flags_dier_all,
      .sr = flags_sr_all },
};

constexpr const Interrupt_constants& get_irq(TIM_irq a_irq)
{
    return irq_constants[static_cast<std::uint32_t>(a_irq)];
}

std::uint32_t read_pending(TIM_TypeDef* a_p_dirver, std::uint32_t a_mask)
{
    std::uint32_t result = bit_flag::get(a_p_dirver->SR, a_mask);
    a_p_dirver->SR = a_p_dirver->SR & ~result; // w0 clear
    return result;
}

extern "C" {
// TIM1 Break Interrupt
void TIM1_BRK_IRQHandler(void)
{
    constexpr std::uint32_t irqn = static_cast<std::uint32_t>(TIM_irq::TIM1_BRK);
    if (rcc<TIM_ADV, 1u>::is_enabled())
        if (std::uint32_t sr = read_pending(TIM1, irq_constants[irqn].sr))
        {
            Interrupt* h = handlers[irqn];
            hkm_assert(nullptr != h);
            h->handle(TIM1, sr);
        }
}
// TIM1 Update and TIM16 global Interrupts
void TIM1_UP_TIM16_IRQHandler(void)
{
    // section TIM1
    {
        constexpr std::uint32_t irqn = static_cast<std::uint32_t>(TIM_irq::TIM1_UP);
        if (rcc<TIM_ADV, 1u>::is_enabled())
            if (std::uint32_t sr = read_pending(TIM1, irq_constants[irqn].sr))
            {
                Interrupt* h = handlers[irqn];
                hkm_assert(nullptr != h);
                h->handle(TIM1, sr);
            }
    }
    // section TIM16
    {
        constexpr std::uint32_t irqn = static_cast<std::uint32_t>(TIM_irq::TIM16_IRQ);
        if (rcc<TIM_G16, 16u>::is_enabled())
            if (std::uint32_t sr = read_pending(TIM16, irq_constants[irqn].sr))
            {
                Interrupt* h = handlers[irqn];
                hkm_assert(nullptr != h);
                h->handle(TIM16, sr);
            }
    }
}

// TIM1 Trigger and Communication and TIM17 global Interrupts
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
    // section TIM1
    {
        constexpr std::uint32_t irqn = static_cast<std::uint32_t>(TIM_irq::TIM1_TRG);
        if (rcc<TIM_ADV, 1u>::is_enabled())
            if (std::uint32_t sr = read_pending(TIM1, irq_constants[irqn].sr))
            {
                Interrupt* h = handlers[irqn];
                hkm_assert(nullptr != h);
                h->handle(TIM1, sr);
            }
    }
    // section TIM17
    {
        constexpr std::uint32_t irqn = static_cast<std::uint32_t>(TIM_irq::TIM17_IRQ);
        if (rcc<TIM_G16, 17u>::is_enabled())
            if (std::uint32_t sr = read_pending(TIM17, irq_constants[irqn].sr))
            {
                Interrupt* h = handlers[irqn];
                hkm_assert(nullptr != h);
                h->handle(TIM17, sr);
            }
    }
}
// TIM1 Capture Compare Interrupt
void TIM1_CC_IRQHandler(void)
{
    constexpr std::uint32_t irqn = static_cast<std::uint32_t>(TIM_irq::TIM1_CC);
    if (rcc<TIM_ADV, 1u>::is_enabled())
        if (std::uint32_t sr = read_pending(TIM1, irq_constants[irqn].sr))
        {
            Interrupt* h = handlers[irqn];
            hkm_assert(nullptr != h);
            h->handle(TIM1, sr);
        }
}
// TIM2 Global Interrupt
void TIM2_IRQHandler(void)
{
    // using T                      = peripheral<TIM<>, 2u>;
    // constexpr std::uint32_t irqn = static_cast<std::uint32_t>(TIM_irq::TIM2_IRQ);
    // if (T::rcc::is_enabled())
    //     if (std::uint32_t sr = read_pending(TIM2, irq_constants[irqn].sr))
    //     {
    //         Interrupt* h = handlers[irqn];
    //         hkm_assert(nullptr != h);
    //         h->handle(TIM2, sr);
    //     }
}
} // extern "C"

} // namespace xmcu::soc::m4::stm32wb::peripherals::timer

// class Interrupt
namespace xmcu::soc::m4::stm32wb::peripherals::timer {

void Interrupt::enable(const IRQ_config& a_config)
{
    hkm_assert(nullptr == handlers[this->idx]);

    handlers[this->idx] = this;

    uint32_t prior = NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_config.preempt_priority, a_config.sub_priority);
    NVIC_SetPriority(irq_constants[this->idx].irq, prior);
    NVIC_EnableIRQ(irq_constants[this->idx].irq);
    // TODO: problem when re-enable with other config?
    // put part with enable/disable to some static in TIM<> but prart with callback stay here?
    // or unify calbacks at api level
}
void Interrupt::disable()
{
    hkm_assert(nullptr != handlers[this->idx]);
    this->unregister_callback();
    Scoped_guard<nvic> nvic_guard;
    handlers[this->idx] = nullptr;
    // use naive loop for check OR add field "next shared irq" to type: decltype(irq_constants)
    for (std::size_t i = 0; i < static_cast<std::uint32_t>(TIM_irq::count); ++i)
    {
        if ((irq_constants[i].irq == irq_constants[this->idx].irq) && (nullptr != handlers[i]))
            return; // don't disable if alternate source of shared interrupt is used
    }
    NVIC_DisableIRQ(irq_constants[this->idx].irq);
}

void Interrupt::register_callback(const Callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;

    this->callback = a_callback;

    const Interrupt_constants& irq = irq_constants[this->idx];
    bit_flag::set(&irq.driver->DIER, irq.dier);
}
void Interrupt::unregister_callback()
{
    Scoped_guard<nvic> guard;

    const Interrupt_constants& irq = irq_constants[this->idx];
    bit_flag::clear(&irq.driver->DIER, irq.dier);

    this->callback = { nullptr, nullptr };
}

} // namespace xmcu::soc::m4::stm32wb::peripherals::timer
