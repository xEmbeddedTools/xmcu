/**/

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/GPIO/GPIO.hpp>

// xmcu
#include <xmcu/bit.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m4/IRQ_config.hpp>
#include <xmcu/soc/ST/m4/nvic.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace {
using namespace xmcu::soc::m4::stm32wb::peripherals;

GPIO::Interrupt::Callback callbacks[7u];
} // namespace

extern "C" {
using namespace xmcu;

static bool interrupt_handler(std::uint32_t a_pr1, std::uint32_t a_bit_index, std::uint32_t a_handler_index)
{
    if (true == bit::is(a_pr1, a_bit_index))
    {
        callbacks[a_handler_index].function(a_bit_index, callbacks[a_handler_index].p_user_data);
        return true;
    }

    return false;
}

void EXTI0_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 0u, 0u))
    {
        bit::set(&(EXTI->PR1), 0u);
    }
}

void EXTI1_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 1u, 1u))
    {
        bit::set(&(EXTI->PR1), 1u);
    }
}

void EXTI2_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 2u, 2u))
    {
        bit::set(&(EXTI->PR1), 2u);
    }
}

void EXTI3_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 3u, 3u))
    {
        bit::set(&(EXTI->PR1), 3u);
    }
}

void EXTI4_IRQHandler()
{
    if (true == interrupt_handler(EXTI->PR1, 4u, 4u))
    {
        bit::set(&(EXTI->PR1), 4u);
    }
}

void EXTI9_5_IRQHandler()
{
    for (std::uint32_t i = 5u; i <= 9u; i++)
    {
        if (true == interrupt_handler(EXTI->PR1, i, 5u))
        {
            bit::set(&(EXTI->PR1), i);
        }
    }
}

void EXTI15_10_IRQHandler()
{
    for (std::uint32_t i = 10u; i <= 15u; i++)
    {
        if (true == interrupt_handler(EXTI->PR1, i, 6u))
        {
            bit::set(&(EXTI->PR1), i);
        }
    }
}
}

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::sources;
using namespace xmcu::soc::m4::stm32wb::system;

void GPIO::In::Pin::set_pull(Pull a_pull)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2),
                  static_cast<std::uint32_t>(a_pull) << (this->id * 2));
}

GPIO::Level GPIO::In::Pin::get_level() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Level>(bit::is(static_cast<GPIO_TypeDef*>(*(this->p_port))->IDR, this->id));
}

GPIO::Pull GPIO::In::Pin::get_pull() const
{
    hkm_assert(nullptr != this->p_port && 0xFF != this->id);

    return static_cast<Pull>(
        (bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR, static_cast<std::uint32_t>(0x3u << this->id))
         << this->id));
}

void GPIO::Out::Pin::set_level(Level a_level)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    constexpr std::uint8_t mul[]                      = { 16u, 0u };
    static_cast<GPIO_TypeDef*>(*(this->p_port))->BSRR = 0x1u << (this->id + mul[static_cast<std::uint32_t>(a_level)]);
}

void GPIO::Out::Pin::toggle_level()
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit::toggle(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->ODR), this->id);
}

void GPIO::Out::Pin::set_type(Type a_type)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER),
                  0x1u << this->id,
                  static_cast<std::uint32_t>(a_type) << this->id);
}

void GPIO::Out::Pin::set_pull(Pull a_pull)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2),
                  static_cast<std::uint32_t>(a_pull) << (this->id * 2));
}

void GPIO::Out::Pin::set_speed(Speed a_speed)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR),
                  0x3u << (this->id * 2),
                  static_cast<std::uint32_t>(a_speed) << (this->id * 2u));
}

GPIO::Level GPIO::Out::Pin::get_level() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Level>(bit::is(static_cast<GPIO_TypeDef*>(*(this->p_port))->IDR, this->id));
}

GPIO::Type GPIO::Out::Pin::get_type() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Type>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER,
                                           static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Pull GPIO::Out::Pin::get_pull() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Pull>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                           static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Speed GPIO::Out::Pin::get_speed() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Speed>((bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR,
                                             static_cast<std::uint32_t>(0x1u << this->id) << this->id)));
}

void GPIO::Analog::Pin::set_pull(Pull a_pull)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2u),
                  static_cast<std::uint32_t>(a_pull) << (this->id * 2));
}

GPIO::Pull GPIO::Analog::Pin::get_pull() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Pull>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                           static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

void GPIO::Alternate_function::Pin::set_type(Type a_type)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER),
                  0x1u << this->id,
                  static_cast<std::uint32_t>(a_type) << this->id);
}

void GPIO::Alternate_function::Pin::set_pull(Pull a_pull)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR),
                  0x3u << (this->id * 2u),
                  static_cast<std::uint32_t>(a_pull) << (this->id * 2u));
}

void GPIO::Alternate_function::Pin::set_speed(Speed a_speed)
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    bit_flag::set(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR),
                  0x3u << (this->id * 2u),
                  static_cast<std::uint32_t>(a_speed) << (this->id * 2u));
}

GPIO::Type GPIO::Alternate_function::Pin::get_type() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Type>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OTYPER,
                                           static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Pull GPIO::Alternate_function::Pin::get_pull() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Pull>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR,
                                           static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

GPIO::Speed GPIO::Alternate_function::Pin::get_speed() const
{
    hkm_assert(nullptr != this->p_port && 0xFFu != this->id);

    return static_cast<Speed>(bit_flag::get(static_cast<GPIO_TypeDef*>(*(this->p_port))->OSPEEDR,
                                            static_cast<std::uint32_t>(0x1u << this->id) << this->id));
}

void GPIO::In::enable(Limited<std::uint32_t, 0, 15> a_id, Pull a_pull, Pin* a_p_pin)
{
    hkm_assert(a_id < 16u);

    hkm_assert(false == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>((*(this->p_port)));

    bit_flag::set(&(p_port->PUPDR), 0x3u << (a_id * 2u), static_cast<std::uint32_t>(a_pull) << (a_id * 2u));
    bit_flag::clear(&(p_port->MODER), 0x3u << (a_id * 2u));

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id     = a_id;
        a_p_pin->p_port = this->p_port;
    }
}

void GPIO::In::disable(Limited<std::uint32_t, 0, 15> a_id)
{
    hkm_assert(a_id < 16u);

    hkm_assert(false == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    const std::uint32_t flag = (0x3u << (a_id * 2u));

    bit_flag::set(&(p_port->MODER), flag);
    bit_flag::clear(&(p_port->PUPDR), flag);

    this->p_port->give_pin(a_id);
}
void GPIO::In::disable(Pin* p_pin)
{
    this->disable(p_pin->get_id());

    p_pin->p_port = nullptr;
    p_pin->id     = 0xFFu;
}

void GPIO::Out::enable(Limited<std::uint32_t, 0, 15> a_id, const Enable_config& a_config, Pin* a_p_pin)
{
    hkm_assert(various::get_enum_incorrect_value<Pull>() != a_config.pull);
    hkm_assert(various::get_enum_incorrect_value<Speed>() != a_config.speed);
    hkm_assert(various::get_enum_incorrect_value<Type>() != a_config.type);

    hkm_assert(false == this->p_port->is_pin_taken(a_id));

    const std::uint32_t clear_flag_2bit = 0x3u << (a_id * 2);
    GPIO_TypeDef* p_port                = static_cast<GPIO_TypeDef*>(*(this->p_port));

    bit_flag::set(&(p_port->OSPEEDR), clear_flag_2bit, static_cast<std::uint32_t>(a_config.speed) << (a_id * 2u));
    bit_flag::set(&(p_port->PUPDR), clear_flag_2bit, static_cast<std::uint32_t>(a_config.pull) << (a_id * 2u));
    bit_flag::set(&(p_port->MODER), clear_flag_2bit, 0x1u << (a_id * 2u));
    bit_flag::set(&(p_port->OTYPER), 0x1u << a_id, static_cast<std::uint32_t>(a_config.type) << a_id);

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id     = a_id;
        a_p_pin->p_port = this->p_port;
    }
}

void GPIO::Out::disable(Limited<std::uint32_t, 0, 15> a_id)
{
    hkm_assert(true == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    const std::uint32_t flag = (0x3u << (a_id * 2u));

    bit_flag::set(&(p_port->MODER), flag);
    bit_flag::clear(&(p_port->OSPEEDR), flag);
    bit_flag::clear(&(p_port->PUPDR), flag);

    this->p_port->give_pin(a_id);
}
void GPIO::Out::disable(Pin* p_pin)
{
    this->disable(p_pin->get_id());

    p_pin->p_port = nullptr;
    p_pin->id     = 0xFFu;
}

void GPIO::Analog::enable(Limited<std::uint32_t, 0, 15> a_id, Pull a_pull, Pin* a_p_out_pin)
{
    hkm_assert(false == this->p_port->is_pin_taken(a_id));

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    bit_flag::set(&(p_port->PUPDR), 0x3u << (a_id * 2u), static_cast<std::uint32_t>(a_pull) << (a_id * 2u));
    bit_flag::set(&(p_port->MODER), 0x3u << (a_id * 2u), 0x3u << (a_id * 2u));

    this->p_port->take_pin(a_id);

    if (nullptr != a_p_out_pin)
    {
        a_p_out_pin->id     = a_id;
        a_p_out_pin->p_port = this->p_port;
    }
}

void GPIO::Analog::disable(Limited<std::uint32_t, 0, 15> a_id)
{
    bit_flag::clear(&(static_cast<GPIO_TypeDef*>(*(this->p_port))->PUPDR), (0x3u << (a_id * 2u)));

    this->p_port->give_pin(a_id);
}
void GPIO::Analog::disable(Pin* p_pin)
{
    this->disable(p_pin->get_id());

    p_pin->p_port = nullptr;
    p_pin->id     = 0xFFu;
}

void GPIO::Alternate_function::disable(Limited<std::uint32_t, 0, 15> a_id)
{
    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    const std::uint32_t flag = (0x3u << (a_id * 2u));

    bit_flag::set(&(p_port->MODER), flag);
    bit_flag::clear(&(p_port->OSPEEDR), flag);
    bit_flag::clear(&(p_port->PUPDR), flag);

    this->p_port->give_pin(a_id);
}
void GPIO::Alternate_function::disable(Pin* p_pin)
{
    this->disable(p_pin->get_id());

    p_pin->p_port = nullptr;
    p_pin->id     = 0xFFu;
}

void GPIO::Alternate_function::enable(Limited<std::uint32_t, 0, 15> a_id,
                                      const Enable_config& a_config,
                                      std::uint32_t a_function,
                                      Pin* a_p_pin)
{
    hkm_assert(false == this->p_port->is_pin_taken(a_id));

    hkm_assert(various::get_enum_incorrect_value<Pull>() != a_config.pull);
    hkm_assert(various::get_enum_incorrect_value<Speed>() != a_config.speed);
    hkm_assert(various::get_enum_incorrect_value<Type>() != a_config.type);

    const std::uint32_t clear_flag_2bit = 0x3u << (a_id * 2);

    GPIO_TypeDef* p_port = static_cast<GPIO_TypeDef*>(*(this->p_port));

    bit_flag::set(&(p_port->OSPEEDR), clear_flag_2bit, static_cast<std::uint32_t>(a_config.speed) << (a_id * 2u));
    bit_flag::set(&(p_port->PUPDR), clear_flag_2bit, static_cast<std::uint32_t>(a_config.pull) << (a_id * 2u));
    bit_flag::set(&(p_port->OTYPER), 0x1u << a_id, static_cast<std::uint32_t>(a_config.type) << a_id);

    std::uint32_t af_register_index = a_id >> 3u;
    std::uint32_t af_register       = p_port->AFR[af_register_index];

    af_register &= ~(0xFu << ((a_id - (af_register_index * 8u)) * 4u));
    af_register |= a_function << ((a_id - (af_register_index * 8u)) * 4u);

    p_port->AFR[af_register_index] = af_register;

    // setting MODER should be the last operation, 
    // because other operations may change the state of the pin unexpectedly
    bit_flag::set(&(p_port->MODER), clear_flag_2bit, 0x2u << (a_id * 2u));
    this->p_port->take_pin(a_id);

    if (nullptr != a_p_pin)
    {
        a_p_pin->id       = a_id;
        a_p_pin->p_port   = this->p_port;
        a_p_pin->function = a_function;
    }
}

void GPIO::Interrupt::enable(const Callback& a_callback, const IRQ_config& a_irq_config)
{
    hkm_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    NVIC_SetPriority(
        this->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->irqn);

    callbacks[this->idx] = a_callback;
}

void GPIO::Interrupt::disable()
{
    hkm_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    NVIC_DisableIRQ(this->irqn);

    callbacks[this->idx] = { nullptr, nullptr };
}

void GPIO::Interrupt::attach(const GPIO& a_port, std::uint32_t a_pin, Trigger_flag a_trigger, Type a_type)
{
    hkm_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    volatile std::uint32_t* p_register = &(SYSCFG->EXTICR[a_pin / 4u]);
    std::uint32_t pos                  = ((static_cast<std::uint32_t>(a_pin) % 4u) * 4u);

#if defined(HKM_ASSERT_ENABLED)
    const bool f = bit_flag::is(*p_register, (a_port.idx) << pos);
    hkm_assert((0u == a_port.idx && true == f) || (0u != a_port.idx && false == f));
    hkm_assert((0u == this->idx && 0u == a_pin) || (1u == this->idx && 1u == a_pin) ||
               (2u == this->idx && 2u == a_pin) || (3u == this->idx && 3u == a_pin) ||
               (4u == this->idx && 4u == a_pin) || (5u == this->idx && (a_pin >= 5u && a_pin <= 9u)) ||
               (6u == this->idx && (a_pin >= 10u && a_pin <= 15u)));
#endif

    Scoped_guard<nvic> guard;

    bit_flag::set(p_register, 0x3u << pos, a_port.idx << pos);

    bit::clear(&(EXTI->RTSR1), a_pin);
    bit::clear(&(EXTI->FTSR1), a_pin);

    switch (a_type)
    {
        case Type::event: {
            bit::set(&(EXTI->EMR1), a_pin);
        }
        break;

        case Type::interrupt: {
            bit::set(&(EXTI->IMR1), a_pin);
        }
        break;
    }

    switch (a_trigger)
    {
        case Trigger_flag::rising: {
            bit::set(&(EXTI->RTSR1), a_pin);
        }
        break;

        case Trigger_flag::falling: {
            bit::set(&(EXTI->FTSR1), a_pin);
        }
        break;

        default: {
            if ((Trigger_flag::rising | Trigger_flag::falling) == a_trigger)
            {
                bit::set(&(EXTI->RTSR1), a_pin);
                bit::set(&(EXTI->FTSR1), a_pin);
            }
        }
    }
}
void GPIO::Interrupt::attach(const GPIO::In::Pin& a_pin, Trigger_flag a_trigger, Type a_type)
{
    this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_type);
}
void GPIO::Interrupt::attach(const GPIO::Out::Pin& a_pin, Trigger_flag a_trigger, Type a_type)
{
    this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_type);
}
void GPIO::Interrupt::attach(const GPIO::Alternate_function::Pin& a_pin, Trigger_flag a_trigger, Type a_type)
{
    this->attach(*(a_pin.get_port()), a_pin.get_id(), a_trigger, a_type);
}

void GPIO::Interrupt::deattach(const GPIO& a_port, std::uint32_t a_pin)
{
    hkm_assert(std::numeric_limits<decltype(this->idx)>::max() != this->idx);

    Scoped_guard<nvic> guard;

    bit::clear(&(EXTI->RTSR1), a_pin);
    bit::clear(&(EXTI->FTSR1), a_pin);

    bit::clear(&(EXTI->EMR1), a_pin);
    bit::clear(&(EXTI->IMR1), a_pin);

    bit_flag::clear(&(SYSCFG->EXTICR[a_pin / 4u]), a_port.idx << ((static_cast<std::uint32_t>(a_pin) % 4u) * 4u));

    callbacks[this->idx] = { nullptr, nullptr };
}
void GPIO::Interrupt::deattach(const GPIO::In::Pin& a_pin)
{
    this->deattach(*(a_pin.get_port()), a_pin.get_id());
}
void GPIO::Interrupt::deattach(const GPIO::Out::Pin& a_pin)
{
    this->deattach(*(a_pin.get_port()), a_pin.get_id());
}
void GPIO::Interrupt::deattach(const GPIO::Alternate_function::Pin& a_pin)
{
    this->deattach(*(a_pin.get_port()), a_pin.get_id());
}

template<> void GPIO::mco::enable<lse>(Divider a_divider)
{
    bit_flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_3);
}
template<> void GPIO::mco::enable<lsi, 1>(Divider a_divider)
{
    bit_flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_1 | RCC_CFGR_MCOSEL_2);
}
template<> void GPIO::mco::enable<lsi, 2>(Divider a_divider)
{
    bit_flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_0 | RCC_CFGR_MCOSEL_1 | RCC_CFGR_MCOSEL_2);
}
template<> void GPIO::mco::enable<hse>(Divider a_divider)
{
    bit_flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_2);
}
template<> void GPIO::mco::enable<hsi16>(Divider a_divider)
{
    bit_flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_0 | RCC_CFGR_MCOSEL_1);
}
template<> void GPIO::mco::enable<pll::r>(Divider a_divider)
{
    bit_flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_0 | RCC_CFGR_MCOSEL_2);
}
template<> void GPIO::mco::enable<rcc<mcu<1u>>>(Divider a_divider)
{
    bit_flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_0);
}
template<> void GPIO::mco::enable<msi>(Divider a_divider)
{
    bit_flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_1);
}
template<> void GPIO::mco::enable<hsi48>(Divider a_divider)
{
    bit_flag::set(&(RCC->CFGR), static_cast<std::uint32_t>(a_divider));
    bit_flag::set(&(RCC->CFGR), RCC_CFGR_MCOSEL, RCC_CFGR_MCOSEL_0 | RCC_CFGR_MCOSEL_3);
}
void GPIO::mco::disable()
{
    bit_flag::clear(&(RCC->CFGR), RCC_CFGR_MCOPRE);
    bit_flag::clear(&(RCC->CFGR), RCC_CFGR_MCOSEL);
}

template<> void GPIO::lsco::enable<lsi>()
{
    bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSCOSEL);
    bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSCOEN);
}
template<> void GPIO::lsco::enable<lse>()
{
    bit_flag::clear(&(RCC->BDCR), RCC_BDCR_LSCOSEL);
    bit_flag::set(&(RCC->BDCR), RCC_BDCR_LSCOEN);
}
void GPIO::lsco::disable()
{
    bit_flag::clear(&(RCC->BDCR), RCC_BDCR_LSCOEN);
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
using namespace xmcu::soc::m4::stm32wb::system;

#if defined(GPIOA_PIN_MASK)
template<> void rcc<GPIO, 1>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOAEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOASMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOASMEN);
    }
}
template<> void rcc<GPIO, 1>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOAEN);
}
#endif

#if defined(GPIOB_PIN_MASK)
template<> void rcc<GPIO, 2>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOBEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOBSMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOBSMEN);
    }
}
template<> void rcc<GPIO, 2>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOBEN);
}
#endif

#if defined(GPIOC_PIN_MASK)
template<> void rcc<GPIO, 3>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOCEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOCSMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOCSMEN);
    }
}
template<> void rcc<GPIO, 3>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOCEN);
}
#endif

#if defined(GPIOD_PIN_MASK)
template<> void rcc<GPIO, 4>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIODEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIODSMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIODSMEN);
    }
}
template<> void rcc<GPIO, 4>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIODEN);
}
#endif

#if defined(GPIOE_PIN_MASK)
template<> void rcc<GPIO, 5>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOEEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOESMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOESMEN);
    }
}
template<> void rcc<GPIO, 5>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOEEN);
}
#endif

#if defined(GPIOH_PIN_MASK)
template<> void rcc<GPIO, 8>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOHEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOHSMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->AHB2SMENR), RCC_AHB2SMENR_GPIOHSMEN);
    }
}
template<> inline void rcc<GPIO, 8>::disable()
{
    bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_GPIOHEN);
}
#endif

template<> void GPIO::Alternate_function::enable<GPIO::mco>(Limited<std::uint32_t, 0, 15> a_id,
                                                            const Enable_config& a_config,
                                                            Pin* a_p_pin)
{
    hkm_assert((0u == this->p_port->idx && 8u == a_id) || (1u == this->p_port->idx && 6u == a_id));

    this->enable(a_id, a_config, 0x0u, a_p_pin);
}
template<> void GPIO::Alternate_function::enable<GPIO::lsco>(Limited<std::uint32_t, 0, 15> a_id,
                                                             const Enable_config& a_config,
                                                             Pin* a_p_pin)
{
    hkm_assert((1 == this->p_port->idx && 2 == a_id) || (8 == this->p_port->idx && 3 == a_id));

    this->enable(a_id, a_config, 0x0u, a_p_pin);
}
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu
