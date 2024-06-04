/**/

// Differences from L4/WB:
// - No channel injecting
// - No deep power down mode
// - No differential inputs
// - No battery/temperature check
// - No custom order channel sequences
// - Only global sampling rate setting

// xmcu
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/peripherals/ADC/ADC.hpp>
#include <xmcu/soc/ST/m0/nvic.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/delay.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/wait_until.hpp>
#include <xmcu/soc/Scoped_guard.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc;
using namespace xmcu::soc::m0;
using namespace xmcu::soc::m0::stm32l0::rm0451::peripherals;
using namespace xmcu::soc::m0::stm32l0::rm0451::system;
using namespace xmcu::soc::m0::stm32l0::rm0451::utils;

ADC* irq_context[] = { nullptr };

bool is_channel(ADC::Channel::Id a_type, const ADC::Channel::Id* a_p_channels, std::size_t a_channels_count)
{
    bool found = false;

    for (std::uint32_t i = 0; i < a_channels_count && false == found; i++)
    {
        found = a_type == a_p_channels[i];
    }

    return found;
}

// This was part of polling_read but was moved away to separate function in order to speed up davin sample stream.
void polling_read_setup(ADC_TypeDef* a_p_registers, ADC::Mode a_mode, std::size_t a_group_size)
{
    hkm_assert((ADC::Mode::discontinuous == a_mode && 0 < a_group_size && a_group_size <= 1) ||
               ADC::Mode::discontinuous != a_mode);

    // Errata ES0483 2.4.2 - disable ADC while configuring CFGR1 to prefent RES being cleared
    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADDIS);
    wait_until::all_bits_are_cleared(a_p_registers->CR, ADC_CR_ADEN);
    bit_flag::set(&(a_p_registers->CFGR1), ADC_CFGR1_CONT | ADC_CFGR1_DISCEN, static_cast<uint32_t>(a_mode));
    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADEN);
}

void polling_read_unsafe(ADC_TypeDef* a_p_registers, std::uint16_t* a_p_buffer, std::size_t a_buffer_capacity)
{
    hkm_assert(a_buffer_capacity > 0);
    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTART);
    while (a_buffer_capacity--)
    {
        while (false == bit_flag::is(a_p_registers->ISR, ADC_ISR_EOC)) continue;
        *(a_p_buffer++) = static_cast<std::uint16_t>(a_p_registers->DR);
    }

    // We've read all samples we needed - It's faster to stop conversion than wait for its end
    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTP);
    bit_flag::set(&(a_p_registers->ISR), ADC_ISR_EOS | ADC_ISR_EOC);
}

void polling_read(ADC_TypeDef* a_p_registers, std::uint16_t* a_p_buffer, std::size_t a_buffer_capacity)
{
    Scoped_guard<nvic> interrupt_guard;
    polling_read_unsafe(a_p_registers, a_p_buffer, a_buffer_capacity);
}

bool polling_read(ADC_TypeDef* a_p_registers,
                  std::uint16_t* a_p_buffer,
                  std::size_t a_buffer_capacity,
                  Milliseconds a_timeout)
{
    hkm_assert(a_buffer_capacity > 0);
    hkm_assert(a_timeout > 0_ms);

    Scoped_guard<nvic> interrupt_guard;
    const std::uint64_t start = tick_counter<Milliseconds>::get();
    std::size_t i             = 0;

    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTART);
    while (tick_counter<Milliseconds>::get() < start + a_timeout.get() && i < a_buffer_capacity)
    {
        if (true == bit_flag::is(a_p_registers->ISR, ADC_ISR_EOC))
        {
            a_p_buffer[i++] = static_cast<std::uint16_t>(a_p_registers->DR);
        }
    }

    // We've read all samples we needed - It's faster to stop conversion than wait for its end
    bit_flag::set(&(a_p_registers->CR), ADC_CR_ADSTP);
    bit_flag::set(&(a_p_registers->ISR), ADC_ISR_EOS | ADC_ISR_EOC);
    return i == a_buffer_capacity;
}

} // namespace

extern "C" {
using namespace xmcu::soc::m0::stm32l0::rm0451::peripherals;

void ADC1_IRQHandler()
{
    hkm_assert(nullptr != irq_context[0]);
    ADC_interrupt_handler(irq_context[0]);
}
}

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
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

void ADC::enable(Resolution a_resolution,
                 const Channel::Id* a_p_channels,
                 std::size_t a_channels_count,
                 Channel::Sampling_time a_sampling_time)
{
    hkm_assert(true == this->is_created());
    hkm_assert(0x0u == bit_flag::get(this->p_registers->CR, ADC_CR_ADSTART));

    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADVREGEN);
    delay::wait(1_ms);

    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADCAL);

    wait_until::all_bits_are_cleared(this->p_registers->CR, ADC_CR_ADCAL);

    bit_flag::set(&(this->p_registers->CFGR1), ADC_CFGR1_RES_Msk, static_cast<std::uint32_t>(a_resolution));
    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADEN);

    wait_until::all_bits_are_set(this->p_registers->ISR, ADC_ISR_ADRDY);

    bit_flag::set(&(this->p_registers->ISR), ADC_ISR_ADRDY);

    this->p_registers->CHSELR = 0;
    for (std::size_t i = 0; i < a_channels_count; i++)
    {
        hkm_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i]);
        bit_flag::set(&this->p_registers->CHSELR, 1 << static_cast<uint32_t>(a_p_channels[i]));
    }

    hkm_assert(various::get_enum_incorrect_value<Channel::Sampling_time>() != a_sampling_time);
    bit_flag::set(&this->p_registers->SMPR, static_cast<std::uint32_t>(a_sampling_time));

    bool enable_voltage_reference = is_channel(Channel::Id::voltage_reference, a_p_channels, a_channels_count);

    if (true == enable_voltage_reference)
    {
        bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_VREFEN);
    }
}
bool ADC::enable(Resolution a_resolution,
                 const Channel::Id* a_p_channels,
                 std::size_t a_channels_count,
                 Channel::Sampling_time a_sampling_time,
                 Milliseconds a_timeout)
{
    hkm_assert(true == this->is_created());
    hkm_assert(0x0u == bit_flag::get(this->p_registers->CR, ADC_CR_ADSTART));
    hkm_assert(0x0u == a_channels_count);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADVREGEN);
    delay::wait(21_us);

    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADCAL);

    bool ret = wait_until::all_bits_are_cleared(
        this->p_registers->CR, ADC_CR_ADCAL, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == ret)
    {
        bit_flag::set(&(this->p_registers->CFGR1), ADC_CFGR1_RES_Msk, static_cast<std::uint32_t>(a_resolution));
        bit_flag::set(&(this->p_registers->CR), ADC_CR_ADEN);

        ret = wait_until::all_bits_are_set(
            this->p_registers->ISR, ADC_ISR_ADRDY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
    }

    if (true == ret)
    {
        bit_flag::set(&(this->p_registers->ISR), ADC_ISR_ADRDY);

        this->p_registers->CHSELR = 0;
        for (std::size_t i = 0; i < a_channels_count; i++)
        {
            hkm_assert(various::get_enum_incorrect_value<Channel::Id>() != a_p_channels[i]);
            bit_flag::set(&this->p_registers->CHSELR, 1 << static_cast<uint32_t>(a_p_channels[i]));
        }

        hkm_assert(various::get_enum_incorrect_value<Channel::Sampling_time>() != a_sampling_time);
        bit_flag::set(&this->p_registers->SMPR, static_cast<std::uint32_t>(a_sampling_time));

        bool enable_voltage_reference = is_channel(Channel::Id::voltage_reference, a_p_channels, a_channels_count);

        if (true == enable_voltage_reference)
        {
            bit_flag::set(&(ADC1_COMMON->CCR), ADC_CCR_VREFEN);
        }
    }

    return ret;
}

void ADC::disable()
{
    hkm_assert(true == this->is_created());
    hkm_assert(0x0u == bit_flag::get(this->p_registers->CR, ADC_CR_ADSTART));

    this->p_registers->CHSELR = 0;
    this->p_registers->SMPR   = 0;

    bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CCR_VREFEN);

    bit_flag::set(&(this->p_registers->CR), ADC_CR_ADDIS);
    wait_until::all_bits_are_cleared(this->p_registers->CR, ADC_CR_ADDIS);
}

ADC::Resolution ADC::get_resolution() const
{
    hkm_assert(true == this->is_created());

    return static_cast<Resolution>(bit_flag::get(this->p_registers->CFGR1, ADC_CFGR1_RES_Msk));
}

std::array<std::pair<ADC::Channel::Id, bool>, ADC::s::max_channels_count> ADC::get_enabled_channels()
{
    std::array<std::pair<Channel::Id, bool>, ADC::s::max_channels_count> channels;

    for (std::uint32_t i = 0; i < ADC::s::max_channels_count; i++)
    {
        channels[i] = {static_cast<Channel::Id>(i), bit_flag::get(this->p_registers->CHSELR, 1 << i) };
    }

    return channels;
}

ADC::Channel::Sampling_time ADC::get_sampling_time()
{
    Channel::Sampling_time sampling_time = static_cast<Channel::Sampling_time>(this->p_registers->SMPR);
    return sampling_time;
}

template<> void ADC::Polling::read_setup<ADC::Mode::single>(std::size_t a_group_size)
{
    polling_read_setup(this->p_ADC->p_registers, ADC::Mode::single, a_group_size);
}
template<> void ADC::Polling::read_setup<ADC::Mode::continuous>(std::size_t a_group_size)
{
    polling_read_setup(this->p_ADC->p_registers, ADC::Mode::continuous, a_group_size);
}
template<> void ADC::Polling::read_setup<ADC::Mode::discontinuous>(std::size_t a_group_size)
{
    polling_read_setup(this->p_ADC->p_registers, ADC::Mode::discontinuous, a_group_size);
}

void ADC::Polling::read_unsafe(Not_null<uint16_t*> a_p_buffer, std::size_t a_buffer_capacity)
{
    polling_read_unsafe(this->p_ADC->p_registers, a_p_buffer, a_buffer_capacity);
}

void ADC::Polling::read(Not_null<uint16_t*> a_p_buffer, std::size_t a_buffer_capacity)
{
    polling_read(this->p_ADC->p_registers, a_p_buffer, a_buffer_capacity);
}

bool ADC::Polling::read(Not_null<uint16_t*> a_p_buffer, std::size_t a_buffer_capacity, Milliseconds a_timeout)
{
    return polling_read(this->p_ADC->p_registers, a_p_buffer, a_buffer_capacity, a_timeout);
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

    // Errata ES0483 2.4.2 - disable ADC while configuring CFGR1 to prefent RES being cleared
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADDIS);
    wait_until::all_bits_are_cleared(this->p_ADC->p_registers->CR, ADC_CR_ADEN);
    bit_flag::clear(&(this->p_ADC->p_registers->CFGR1), ADC_CFGR1_CONT);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADEN);
    bit_flag::set(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);
}
template<> void ADC::Interrupt::read_start<ADC::Mode::continuous>(const Callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;

    this->p_ADC->callback = a_callback;

    // Errata ES0483 2.4.2 - disable ADC while configuring CFGR1 to prefent RES being cleared
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADDIS);
    wait_until::all_bits_are_cleared(this->p_ADC->p_registers->CR, ADC_CR_ADEN);
    bit_flag::set(&(this->p_ADC->p_registers->CFGR1), ADC_CFGR1_CONT);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADEN);
    bit_flag::set(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);
}
template<>
void ADC::Interrupt::read_start<ADC::Mode::discontinuous>(const Callback& a_callback, std::size_t a_group_size)
{
    hkm_assert(nullptr != a_callback.function);
    hkm_assert(0u < a_group_size && a_group_size <= 1);

    Scoped_guard<nvic> guard;

    this->p_ADC->callback = a_callback;

    // Errata ES0483 2.4.2 - disable ADC while configuring CFGR1 to prefent RES being cleared
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADDIS);
    wait_until::all_bits_are_cleared(this->p_ADC->p_registers->CR, ADC_CR_ADEN);
    bit_flag::set(&(this->p_ADC->p_registers->CFGR1), ADC_CFGR1_CONT | ADC_CFGR1_DISCEN, ADC_CFGR1_DISCEN);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADEN);
    bit_flag::set(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);
}

void ADC::Interrupt::read_stop()
{
    Scoped_guard<nvic> guard;

    // Errata ES0483 2.4.2 - disable ADC while configuring CFGR1 to prefent RES being cleared
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADDIS);
    wait_until::all_bits_are_cleared(this->p_ADC->p_registers->CR, ADC_CR_ADEN);
    bit_flag::clear(&(this->p_ADC->p_registers->CFGR1), ADC_CFGR1_CONT | ADC_CFGR1_DISCEN);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADEN);
    bit_flag::set(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTP);

    bit_flag::clear(&(this->p_ADC->p_registers->IER), ADC_IER_EOCIE | ADC_IER_EOSIE);
    bit_flag::clear(&(this->p_ADC->p_registers->CR), ADC_CR_ADSTART);

    this->p_ADC->callback = { nullptr, nullptr };
}
} // namespace peripherals
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
using namespace xmcu::soc::m0::stm32l0::rm0451::peripherals;
using namespace xmcu::soc::m0::stm32l0::rm0451::sources;
using namespace xmcu::soc::m0::stm32l0::rm0451::system;

template<> void rcc<ADC>::async::enable<rcc<mcu<1u>>>(Prescaler a_prescaler, bool a_enable_in_lp)
{
    bit_flag::set(&RCC->APB2ENR, RCC_APB2ENR_ADCEN);
    bit_flag::clear(&ADC1->CFGR2, ADC_CFGR2_CKMODE_Msk);
    bit_flag::set(&ADC1_COMMON->CCR, ADC_CCR_PRESC_Msk, static_cast<std::uint32_t>(a_prescaler));
}

template<> void rcc<ADC>::sync::enable<rcc<mcu<1u>>::hclk<1u>>(Prescaler a_prescaler, bool a_enable_in_lp)
{
    bit_flag::set(&RCC->APB2ENR, RCC_APB2ENR_ADCEN);
    bit_flag::clear(&ADC1_COMMON->CCR, ADC_CCR_PRESC_Msk);
    bit_flag::set(&ADC1->CFGR2, ADC_CFGR2_CKMODE_Msk, static_cast<std::uint32_t>(a_prescaler));
}

} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
