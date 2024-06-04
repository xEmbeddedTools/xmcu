/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/ADC/ADC.hpp>

// xmcu
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m4/nvic.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/delay.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::peripherals;
using namespace xmcu::soc::m4::stm32wb::system;
using namespace xmcu::soc::m4::stm32wb::utils;

ADC* irq_context[] = { nullptr };

bool is_channel(ADC::Channel::Id a_type, const ADC::Channel* a_p_channels, std::size_t a_channels_count)
{
    bool found = false;

    for (std::uint32_t i = 0; i < a_channels_count && false == found; i++)
    {
        found = a_type == a_p_channels[i].id;
    }

    return found;
}

void polling_read(ADC_TypeDef* a_p_registers,
                  ADC::Mode a_mode,
                  std::uint16_t* a_p_buffer,
                  std::size_t a_buffer_capacity,
                  std::size_t a_group_size)
{
    hkm_assert(a_buffer_capacity > 0);
    hkm_assert((ADC::Mode::discontinuous == a_mode && a_group_size > 0 && a_group_size <= 8) ||
               ADC::Mode::discontinuous != a_mode);

    bit_flag::set(
        &(a_p_registers->CFGR), ADC_CFGR_CONT | ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM_Msk, static_cast<uint32_t>(a_mode));

    if (ADC::Mode::discontinuous == a_mode)
    {
        bit_flag::set(&(a_p_registers->CFGR), (a_group_size - 1) << ADC_CFGR_DISCNUM_Pos);
    }

    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTART);

    std::uint32_t i = 0;
    while (i < a_buffer_capacity)
    {
        if (true == bit_flag::is(a_p_registers->ISR, ADC_ISR_EOC))
        {
            a_p_buffer[i++] = static_cast<std::uint16_t>(a_p_registers->DR);
        }
    }

    wait_until::all_bits_are_set(a_p_registers->ISR, ADC_ISR_EOS);
    bit_flag::set(&(a_p_registers->ISR), ADC_ISR_EOS);

    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(a_p_registers->CR), ADC_CR_ADSTART);
}

bool polling_read(ADC_TypeDef* a_p_registers,
                  ADC::Mode a_mode,
                  std::uint16_t* a_p_buffer,
                  std::size_t a_buffer_capacity,
                  std::size_t a_group_size,
                  Milliseconds a_timeout)
{
    hkm_assert(a_buffer_capacity > 0);
    hkm_assert((ADC::Mode::discontinuous == a_mode && a_group_size > 0 && a_group_size <= 8) ||
               ADC::Mode::discontinuous != a_mode);
    hkm_assert(a_timeout > 0_ms);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    std::size_t i = 0;

    bit_flag::set(
        &(a_p_registers->CFGR), ADC_CFGR_CONT | ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM_Msk, static_cast<uint32_t>(a_mode));

    if (ADC::Mode::discontinuous == a_mode)
    {
        bit_flag::set(&(a_p_registers->CFGR), (a_group_size - 1) << ADC_CFGR_DISCNUM_Pos);
    }

    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTART);

    while (tick_counter<Milliseconds>::get() < start + a_timeout.get() && i < a_buffer_capacity)
    {
        if (true == bit_flag::is(a_p_registers->ISR, ADC_ISR_EOC))
        {
            a_p_buffer[i++] = static_cast<std::uint16_t>(a_p_registers->DR);
        }
    }

    bool ret = wait_until::all_bits_are_set(
        a_p_registers->ISR, ADC_ISR_EOS, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == ret)
    {
        bit_flag::set(&(a_p_registers->ISR), ADC_ISR_EOS);
    }

    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTP);
    bit_flag::clear(&(a_p_registers->CR), ADC_CR_ADSTART);

    return ret && i == a_buffer_capacity;
}

} // namespace

extern "C" {
using namespace xmcu::soc::m4::stm32wb::peripherals;

void ADC1_IRQHandler()
{
    hkm_assert(nullptr != irq_context[0]);
    ADC_interrupt_handler(irq_context[0]);
}
}

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {
using namespace xmcu;
using namespace utils;

void ADC_interrupt_handler(ADC* a_p_this)
{
    hkm_assert(nullptr != a_p_this);

    ADC_TypeDef* p_registers = static_cast<ADC_TypeDef*>(*(a_p_this));

    const std::uint32_t isr = p_registers->ISR;

    if (true == bit_flag::is(isr, ADC_ISR_EOC))
    {
        const bool series_end = bit_flag::is(isr, ADC_ISR_EOS);

        a_p_this->callback.function(a_p_this, p_registers->DR, series_end, a_p_this->callback.p_user_data);

        if (true == series_end)
        {
            bit_flag::set(&(p_registers->ISR), ADC_ISR_EOS);
        }
    }
}

void ADC::enable(Resolution a_resolution, const Channel* a_p_channels, std::size_t a_channels_count)
{
    hkm_assert(true == this->is_created());
    hkm_assert(0x0u == bit_flag::get(this->p_registers->CR, ADC_CR_ADSTART | ADC_CR_JADSTART));

    bit_flag::clear(&(this->p_registers->CR), ADC_CR_DEEPPWD);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADVREGEN);
    delay::wait(21_us);

    bit_flag::clear(&(this->p_registers->CR), ADC_CR_ADCALDIF);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADCAL);

    wait_until::all_bits_are_cleared(this->p_registers->CR, ADC_CR_ADCAL);

    bit_flag::set(&(this->p_registers->CFGR), ADC_CFGR_RES_Msk, static_cast<std::uint32_t>(a_resolution));
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADEN);

    wait_until::all_bits_are_set(this->p_registers->ISR, ADC_ISR_ADRDY);

    bit_flag::set(&(this->p_registers->ISR), ADC_ISR_ADRDY);

    this->p_registers->SQR1 = a_channels_count - 1;

    volatile uint32_t* p_SQRs  = &(this->p_registers->SQR1);
    volatile uint32_t* p_SMPRs = &(this->p_registers->SMPR1);

    for (std::size_t i = 0; i < a_channels_count && i < 4; i++)
    {
        hkm_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
        bit_flag::set(&(p_SQRs[0]), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
    }

    for (std::size_t i = 4; i < a_channels_count; i++)
    {
        hkm_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
        bit_flag::set(&(p_SQRs[(i + 1) / 5]), static_cast<uint32_t>(a_p_channels[i].id) << 6 * ((i + 1) % 5));
    }

    for (std::uint32_t i = 0; i < a_channels_count; i++)
    {
        hkm_assert(various::get_enum_incorrect_value<Channel::Sampling_time>() != a_p_channels[i].sampling_time);

        const std::uint32_t channel_id        = static_cast<std::uint32_t>(a_p_channels[i].id);
        const std::uint32_t sampling_time_val = static_cast<std::uint32_t>(a_p_channels[i].sampling_time);
        const std::uint32_t register_index    = channel_id / 10;

        bit_flag::set(&(p_SMPRs[register_index]), sampling_time_val << ((channel_id - (register_index * 10)) * 3));
    }

    bool enable_temperature_sensor = is_channel(Channel::Id::temperature_sensor, a_p_channels, a_channels_count);
    bool enable_voltage_reference  = is_channel(Channel::Id::voltage_reference, a_p_channels, a_channels_count);
    bool enable_battery_voltage    = is_channel(Channel::Id::battery_voltage, a_p_channels, a_channels_count);

    if (true == enable_temperature_sensor)
    {
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_TSEN);
        delay::wait(120_us);
    }

    if (true == enable_voltage_reference)
    {
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_VREFEN);
    }

    if (true == enable_battery_voltage)
    {
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_VBATEN);
    }
}
bool ADC::enable(Resolution a_resolution,
                 const Channel* a_p_channels,
                 std::size_t a_channels_count,
                 Milliseconds a_timeout)
{
    hkm_assert(true == this->is_created());
    hkm_assert(0x0u == bit_flag::get(this->p_registers->CR, ADC_CR_ADSTART | ADC_CR_JADSTART));
    hkm_assert(0x0u == a_channels_count);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::clear(&(this->p_registers->CR), ADC_CR_DEEPPWD);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADVREGEN);
    delay::wait(21_us);

    bit_flag::clear(&(this->p_registers->CR), ADC_CR_ADCALDIF);
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADCAL);

    bool ret = wait_until::all_bits_are_cleared(
        this->p_registers->CR, ADC_CR_ADCAL, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == ret)
    {
        bit_flag::set(&(this->p_registers->CFGR), ADC_CFGR_RES_Msk, static_cast<std::uint32_t>(a_resolution));
        bit_flag::set(&(this->p_registers->CR), ADC_CR_ADEN);

        ret = wait_until::all_bits_are_set(
            this->p_registers->ISR, ADC_ISR_ADRDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
    }

    if (true == ret)
    {
        bit_flag::set(&(this->p_registers->ISR), ADC_ISR_ADRDY);

        this->p_registers->SQR1 = a_channels_count - 1;

        volatile uint32_t* p_SQRs  = &(this->p_registers->SQR1);
        volatile uint32_t* p_SMPRs = &(this->p_registers->SMPR1);

        for (std::size_t i = 0; i < a_channels_count && i < 4; i++)
        {
            hkm_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
            bit_flag::set(&(p_SQRs[0]), static_cast<uint32_t>(a_p_channels[i].id) << 6 * (i + 1));
        }

        for (std::size_t i = 4; i < a_channels_count; i++)
        {
            hkm_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i].id);
            bit_flag::set(&(p_SQRs[(i + 1) / 5]), static_cast<uint32_t>(a_p_channels[i].id) << 6 * ((i + 1) % 5));
        }

        for (std::uint32_t i = 0; i < a_channels_count; i++)
        {
            hkm_assert(various::get_enum_incorrect_value<Channel::Sampling_time>() != a_p_channels[i].sampling_time);

            const std::uint32_t channel_id        = static_cast<std::uint32_t>(a_p_channels[i].id);
            const std::uint32_t sampling_time_val = static_cast<std::uint32_t>(a_p_channels[i].sampling_time);
            const std::uint32_t register_index    = channel_id / 10u;

            bit_flag::set(&(p_SMPRs[register_index]), sampling_time_val << ((channel_id - (register_index * 10)) * 3));
        }

        bool enable_temperature_sensor = is_channel(Channel::Id::temperature_sensor, a_p_channels, a_channels_count);
        bool enable_voltage_reference  = is_channel(Channel::Id::voltage_reference, a_p_channels, a_channels_count);
        bool enable_battery_voltage    = is_channel(Channel::Id::battery_voltage, a_p_channels, a_channels_count);

        if (true == enable_temperature_sensor)
        {
            bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_TSEN);
            delay::wait(120_us);
        }

        if (true == enable_voltage_reference)
        {
            bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_VREFEN);
        }

        if (true == enable_battery_voltage)
        {
            bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_VBATEN);
        }
    }

    return ret;
}

void ADC::disable()
{
    hkm_assert(true == this->is_created());
    hkm_assert(0x0u == bit_flag::get(this->p_registers->CR, ADC_CR_ADSTART | ADC_CR_JADSTART));

    this->p_registers->SQR1 = 0;

    this->p_registers->SQR2 = 0;
    this->p_registers->SQR3 = 0;
    this->p_registers->SQR4 = 0;

    this->p_registers->SMPR1 = 0;
    this->p_registers->SMPR2 = 0;

    bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CCR_TSEN | ADC_CCR_VREFEN | ADC_CCR_VBATEN);

    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADDIS);
    wait_until::all_bits_are_cleared(this->p_registers->CR, ADC_CR_ADDIS);

    bit_flag::set(&(this->p_registers->CR), ADC_CR_DEEPPWD);
}

ADC::Resolution ADC::get_resolution() const
{
    hkm_assert(true == this->is_created());

    return static_cast<Resolution>(bit_flag::get(this->p_registers->CFGR, ADC_CFGR_RES_Msk));
}

std::array<std::pair<ADC::Channel::Id, bool>, ADC::s::max_channels_count> ADC::get_enabled_channels()
{
    std::uint32_t enabled_channels_count = bit_flag::get(this->p_registers->SQR1, ADC_SQR1_L) + 1;

    volatile const uint32_t* p_SQRs  = &(this->p_registers->SQR1);

    std::array<std::pair<ADC::Channel::Id, bool>, ADC::s::max_channels_count> ret;
    for (std::uint32_t i = 0; i < ADC::s::max_channels_count; i++)
    {
        ret[i] = {static_cast<Channel::Id>(i), false };
    }

    for (std::uint32_t i = 0; i < enabled_channels_count && i < 4; i++)
    {
        const std::uint32_t offset = (6 * (i + 1));
        const std::uint32_t index = bit_flag::get(p_SQRs[0], 0x1Fu << offset) >> offset;
        ret[index].second = true;
    }

    for (std::uint32_t i = 4; i < enabled_channels_count; i++)
    {
        const std::uint32_t offset = 6 * ((i + 1) % 5);
        const std::uint32_t index = bit_flag::get(p_SQRs[(i + 1u) / 5u], 0x1Fu << offset) >> offset;
        ret[index].second = true;
    }

    return ret;
}

template<> void ADC::Polling::read<ADC::Mode::single>(Not_null<uint16_t*> a_p_buffer, std::size_t a_buffer_capacity)
{
    polling_read(this->p_ADC->p_registers, ADC::Mode::single, a_p_buffer, a_buffer_capacity, 0u);
}
template<> void ADC::Polling::read<ADC::Mode::continuous>(Not_null<uint16_t*> a_p_buffer, std::size_t a_buffer_capacity)
{
    polling_read(this->p_ADC->p_registers, ADC::Mode::continuous, a_p_buffer, a_buffer_capacity, 0u);
}
template<> void ADC::Polling::read<ADC::Mode::discontinuous>(Not_null<uint16_t*> a_p_buffer,
                                                             std::size_t a_buffer_capacity,
                                                             std::size_t a_group_size)
{
    polling_read(this->p_ADC->p_registers, ADC::Mode::discontinuous, a_p_buffer, a_buffer_capacity, a_group_size);
}

template<> bool ADC::Polling::read<ADC::Mode::single>(Not_null<uint16_t*> a_p_buffer,
                                                      std::size_t a_buffer_capacity,
                                                      Milliseconds a_timeout)
{
    return polling_read(this->p_ADC->p_registers, ADC::Mode::single, a_p_buffer, a_buffer_capacity, 0u, a_timeout);
}
template<> bool ADC::Polling::read<ADC::Mode::continuous>(Not_null<uint16_t*> a_p_buffer,
                                                          std::size_t a_buffer_capacity,
                                                          Milliseconds a_timeout)
{
    return polling_read(this->p_ADC->p_registers, ADC::Mode::continuous, a_p_buffer, a_buffer_capacity, 0u, a_timeout);
}
template<> bool ADC::Polling::read<ADC::Mode::discontinuous>(Not_null<uint16_t*> a_p_buffer,
                                                             std::size_t a_buffer_capacity,
                                                             std::size_t a_group_size,
                                                             Milliseconds a_timeout)
{
    return polling_read(
        this->p_ADC->p_registers, ADC::Mode::discontinuous, a_p_buffer, a_buffer_capacity, a_group_size, a_timeout);
}

void ADC::Interrupt::enable(const IRQ_config& a_irq_config)
{
    hkm_assert(nullptr == irq_context[0]);

    irq_context[0] = this->p_ADC;

    NVIC_SetPriority(
        this->p_ADC->irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->p_ADC->irqn);
}

void ADC::Interrupt::disable()
{
    this->read_stop();

    NVIC_DisableIRQ(this->p_ADC->irqn);

    irq_context[0] = nullptr;
}

template<> void ADC::Interrupt::read_start<ADC::Mode::single>(const Callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;

    this->p_ADC->callback = a_callback;

    bit_flag::clear(&(this->p_ADC->p_registers->CFGR), ADC_CFGR_CONT);
    bit_flag::set(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);
}
template<> void ADC::Interrupt::read_start<ADC::Mode::continuous>(const Callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;

    this->p_ADC->callback = a_callback;

    bit_flag::set(&(this->p_ADC->p_registers->CFGR), ADC_CFGR_CONT);
    bit_flag::set(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);
}
template<>
void ADC::Interrupt::read_start<ADC::Mode::discontinuous>(const Callback& a_callback, std::size_t a_group_size)
{
    hkm_assert(nullptr != a_callback.function);
    hkm_assert(a_group_size > 0u);

    Scoped_guard<nvic> guard;

    this->p_ADC->callback = a_callback;

    bit_flag::set(
        &(this->p_ADC->p_registers->CFGR), ADC_CFGR_CONT | ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM_Msk, ADC_CFGR_DISCEN);
    bit_flag::set(&(this->p_ADC->p_registers->CFGR), (a_group_size - 1) << ADC_CFGR_DISCNUM_Pos);

    bit_flag::set(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);
}

void ADC::Interrupt::read_stop()
{
    Scoped_guard<nvic> guard;

    bit_flag::clear(&(this->p_ADC->p_registers->CFGR), ADC_CFGR_CONT | ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM_Msk);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTP);

    bit_flag::clear(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::clear(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);

    this->p_ADC->callback = { nullptr, nullptr };
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
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::peripherals;
using namespace xmcu::soc::m4::stm32wb::sources;
using namespace xmcu::soc::m4::stm32wb::system;

template<> void rcc<ADC>::async::enable<rcc<mcu<1u>>>(Prescaler a_prescaler, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_ADCSEL_Msk);
    bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE_Msk | ADC_CCR_PRESC_Msk, static_cast<std::uint32_t>(a_prescaler));
}
template<> void rcc<ADC>::async::enable<pll::sai1::r>(Prescaler a_prescaler, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_ADCSEL_Msk, RCC_CCIPR_ADCSEL_0);
    bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE_Msk | ADC_CCR_PRESC_Msk, static_cast<std::uint32_t>(a_prescaler));
}
template<> void rcc<ADC>::async::enable<pll::p>(Prescaler a_prescaler, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_ADCSEL_Msk, RCC_CCIPR_ADCSEL_1);
    bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE_Msk | ADC_CCR_PRESC_Msk, static_cast<std::uint32_t>(a_prescaler));
}
template<> void rcc<ADC>::sync::enable<rcc<mcu<1u>>::hclk<1u>>(Prescaler a_prescaler, bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_ADCSEL_Msk);
    bit_flag::set(&(ADC1_COMMON->CCR), static_cast<std::uint32_t>(a_prescaler));
}
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif