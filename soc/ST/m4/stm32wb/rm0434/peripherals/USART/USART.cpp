#include "USART.hpp"
/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/USART/LPUART.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/USART/USART.hpp>

// xmcu
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m4/nvic.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

// debug
#include <xmcu/assertion.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::m4;
using namespace xmcu::soc::m4::stm32wb::peripherals;
using namespace xmcu::soc::m4::stm32wb::utils;

constexpr std::uint32_t clock_prescaler_lut[] = { 1u, 2u, 4u, 6u, 8u, 10u, 12u, 16u, 32u, 64u, 128u, 256u };

bool is_rx_error(USART_TypeDef* a_p_registers)
{
    return bit::is_any(a_p_registers->ISR, USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
}

bool is_tx_error(USART_TypeDef* a_p_registers)
{
    return bit::is_any(a_p_registers->ISR, USART_ISR_PE | USART_ISR_NE);
}

USART::Event_flag get_Event_flag_and_clear(volatile std::uint32_t* a_p_icr, std::uint32_t a_isr)
{
    USART::Event_flag pending_events = USART::Event_flag::none;
    std::uint32_t clear_mask = 0;

    if (true == bit_flag::is(a_isr, USART_ISR_IDLE))
    {
        clear_mask |= USART_ICR_IDLECF;
        pending_events |= USART::Event_flag::idle;
    }
    if (true == bit_flag::is(a_isr, USART_ISR_TC))
    {
        clear_mask |= USART_ICR_TCCF;
        pending_events |= USART::Event_flag::transfer_complete;
    }

    if (true == bit_flag::is(a_isr, USART_ISR_WUF))
    {
        clear_mask |= USART_ICR_WUCF;
        pending_events |= USART::Event_flag::wakeup_from_stop;
    }
    if (true == bit_flag::is(a_isr, USART_ISR_CMF))
    {
        clear_mask |= USART_ICR_CMCF;
        pending_events |= USART::Event_flag::character_matched;
    }

    if (true == bit::is_any(a_isr, USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE))
    {
        if (true == bit_flag::is(a_isr, USART_ISR_PE))
        {
            clear_mask |= USART_ICR_PECF;
            pending_events |= USART::Event_flag::parity_error;
        }
        if (true == bit_flag::is(a_isr, USART_ISR_FE))
        {
            clear_mask |= USART_ICR_FECF;
            pending_events |= USART::Event_flag::framing_error;
        }
        if (true == bit_flag::is(a_isr, USART_ISR_ORE))
        {
            clear_mask |= USART_ICR_ORECF;
            pending_events |= USART::Event_flag::overrun;
        }
        if (true == bit_flag::is(a_isr, USART_ISR_NE))
        {
            clear_mask |= USART_ICR_NECF;
            pending_events |= USART::Event_flag::noise_detected;
        }
    }

    bit_flag::set(a_p_icr, clear_mask);
    return pending_events;
}

USART::Event_flag get_pending_events(std::uint32_t a_isr)
{
    USART::Event_flag pending_events = USART::Event_flag::none;

    if (true == bit_flag::is(a_isr, USART_ISR_IDLE))
    {
        pending_events |= USART::Event_flag::idle;
    }
    if (true == bit_flag::is(a_isr, USART_ISR_TC))
    {
        pending_events |= USART::Event_flag::transfer_complete;
    }

    if (true == bit_flag::is(a_isr, USART_ISR_WUF))
    {
        pending_events |= USART::Event_flag::wakeup_from_stop;
    }
    if (true == bit_flag::is(a_isr, USART_ISR_CMF))
    {
        pending_events |= USART::Event_flag::character_matched;
    }

    if (true == bit::is_any(a_isr, USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE))
    {
        if (true == bit_flag::is(a_isr, USART_ISR_PE))
        {
            pending_events |= USART::Event_flag::parity_error;
        }
        if (true == bit_flag::is(a_isr, USART_ISR_FE))
        {
            pending_events |= USART::Event_flag::framing_error;
        }
        if (true == bit_flag::is(a_isr, USART_ISR_ORE))
        {
            pending_events |= USART::Event_flag::overrun;
        }
        if (true == bit_flag::is(a_isr, USART_ISR_NE))
        {
            pending_events |= USART::Event_flag::noise_detected;
        }
    }

    return pending_events;
}

void clear_events(volatile std::uint32_t* a_p_icr, USART::Event_flag a_event_mask)
{
    std::uint32_t clear_mask = 0;

    if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::idle))
    {
        clear_mask |= USART_ICR_IDLECF;
    }
    if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::transfer_complete))
    {
        clear_mask |= USART_ICR_TCCF;
    }

    if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::wakeup_from_stop))
    {
        clear_mask |= USART_ICR_WUCF;
    }
    if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::character_matched))
    {
        clear_mask |= USART_ICR_CMCF;
    }

    if (USART::Event_flag::none != (a_event_mask & (USART::Event_flag::parity_error | USART::Event_flag::framing_error |
                                                    USART::Event_flag::overrun | USART::Event_flag::noise_detected)))
    {
        if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::parity_error))
        {
            clear_mask |= USART_ICR_PECF;
        }
        if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::framing_error))
        {
            clear_mask |= USART_ICR_FECF;
        }
        if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::overrun))
        {
            clear_mask |= USART_ICR_ORECF;
        }
        if (USART::Event_flag::none != (a_event_mask & USART::Event_flag::noise_detected))
        {
            clear_mask |= USART_ICR_NECF;
        }
    }

    bit_flag::set(a_p_icr, clear_mask);
}

template<typename Periph_t> typename Periph_t::Frame_format::Word_length get_Word_length(std::uint32_t a_CR1_register)
{
    hkm_assert(false == bit_flag::is(a_CR1_register, USART_CR1_M0) ||
               false == bit_flag::is(a_CR1_register, USART_CR1_M1));

    if (false == bit_flag::is(a_CR1_register, USART_CR1_M0) && false == bit_flag::is(a_CR1_register, USART_CR1_M1))
    {
        return USART::Frame_format::Word_length::_8_bit;
    }

    if (false == bit_flag::is(a_CR1_register, USART_CR1_M0) && true == bit_flag::is(a_CR1_register, USART_CR1_M1))
    {
        return USART::Frame_format::Word_length::_7_bit;
    }

    return USART::Frame_format::Word_length::_9_bit;
}

void enable(USART_TypeDef* a_p_registers,
            const LPUART::Clock_config& a_clock_config,
            const LPUART::Transceiving_config& a_transceiving_config,
            const LPUART::Frame_format& a_frame_format,
            LPUART::Low_power_wakeup_method a_low_power_wakeup)
{
    a_p_registers->CR1 = 0x0u;
    a_p_registers->CR2 = 0x0u;
    a_p_registers->CR3 = 0x0u;
    a_p_registers->PRESC = static_cast<std::uint32_t>(a_clock_config.prescaler);

#if defined(HKM_ASSERT_ENABLED)
    constexpr std::uint32_t BRR_min = 0x300u;
    constexpr std::uint32_t BRR_max = 0xFFFFFu;

    std::uint32_t clk_freq =
        a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)];

    hkm_assert(clk_freq >= 3u * a_transceiving_config.baud_rate && clk_freq <= a_transceiving_config.baud_rate * 4096u);

    std::uint32_t brr = (clk_freq * 256u + (a_transceiving_config.baud_rate / 2)) / a_transceiving_config.baud_rate;
    hkm_assert(brr >= BRR_min && brr <= BRR_max);
    a_p_registers->BRR = brr;
#endif

#if !defined(HKM_ASSERT_ENABLED)
    a_p_registers->BRR =
        ((a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) * 256u +
         (a_transceiving_config.baud_rate / 2u)) /
        a_transceiving_config.baud_rate;
#endif

    if (LPUART::Transceiving_config::Flow_control_flag::RS232 ==
        (a_transceiving_config.flow_control & LPUART::Transceiving_config::Flow_control_flag::RS232))
    {
        a_p_registers->CR3 = bit_flag::get(static_cast<std::uint32_t>(a_transceiving_config.flow_control),
                                           USART_CR3_RTSE & USART_CR3_CTSE);
    }

    if (LPUART::Transceiving_config::Flow_control_flag::RS485 ==
        (a_transceiving_config.flow_control & LPUART::Transceiving_config::Flow_control_flag::RS485))
    {
        a_p_registers->CR3 = USART_CR3_DEM;
        bit_flag::set(&(a_p_registers->CR1),
                      bit_flag::get(static_cast<std::uint32_t>(a_transceiving_config.flow_control),
                                    USART_CR1_DEAT_Msk & USART_CR1_DEDT_Msk));
    }

    if (LPUART::Transceiving_config::Mute_method::character_matched ==
        static_cast<LPUART::Transceiving_config::Mute_method>(
            static_cast<std::uint32_t>(a_transceiving_config.mute_method) & 0x200u))
    {
        bit_flag::set(&(a_p_registers->CR2),
                      ((bit_flag::get(static_cast<std::uint8_t>(a_transceiving_config.mute_method), 0xFFu))
                       << USART_CR2_ADD_Pos) |
                          USART_CR2_ADDM7);
        bit_flag::set(&(a_p_registers->CR1), USART_CR1_MME | USART_CR1_WAKE);
        a_p_registers->RQR = USART_RQR_MMRQ;
    }
    else if (LPUART::Transceiving_config::Mute_method::idle_line == a_transceiving_config.mute_method)
    {
        bit_flag::set(&(a_p_registers->CR1), USART_CR1_MME);
        a_p_registers->RQR = USART_RQR_MMRQ;
    }

    if (USART::Low_power_wakeup_method::none != a_low_power_wakeup)
    {
        bit_flag::set(&(a_p_registers->CR1), USART_CR1_UESM);

        bit_flag::clear(&(a_p_registers->CR1), USART_CR1_UE);
        bit_flag::set(&(a_p_registers->CR3), USART_CR3_WUS_Msk, static_cast<std::uint32_t>(a_low_power_wakeup));
    }

    bit_flag::set(&(a_p_registers->CR2), static_cast<std::uint32_t>(a_transceiving_config.stop_bits));
    bit_flag::set(&(a_p_registers->CR1),
                  static_cast<std::uint32_t>(a_transceiving_config.mode) |
                      static_cast<std::uint32_t>(a_frame_format.parity) |
                      static_cast<std::uint32_t>(a_frame_format.word_length) | USART_CR1_UE | USART_CR1_FIFOEN);
}

void enable(USART_TypeDef* a_p_registers,
            const USART::Clock_config& a_clock_config,
            const USART::Transceiving_config& a_transceiving_config,
            const USART::Frame_format& a_frame_format,
            USART::Low_power_wakeup_method a_low_power_wakeup)
{
#if defined(HKM_ASSERT_ENABLED)
    constexpr std::uint32_t BRR_min = 0x10u;
    constexpr std::uint32_t BRR_max = 0xFFFFu;
#endif

    a_p_registers->CR1 = 0x0u;
    a_p_registers->CR2 = 0x0u;
    a_p_registers->CR3 = 0x0u;
    a_p_registers->PRESC = static_cast<std::uint32_t>(a_clock_config.prescaler);

    switch (a_transceiving_config.oversampling)
    {
        case USART::Transceiving_config::Oversampling::_16: {
#if defined(HKM_ASSERT_ENABLED)
            std::uint32_t brr =
                (a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) /
                a_transceiving_config.baud_rate;
            hkm_assert(brr >= BRR_min && brr <= BRR_max);

            a_p_registers->BRR = brr;
#endif

#if !defined(HKM_ASSERT_ENABLED)
            a_p_registers->BRR =
                (a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) /
                a_transceiving_config.baud_rate;
#endif

            bit_flag::clear(&(a_p_registers->CR1), USART_CR1_OVER8);
        }
        break;

        case USART::Transceiving_config::Oversampling::_8: {
#if defined(HKM_ASSERT_ENABLED)
            const std::uint32_t usartdiv =
                (2 *
                 (a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)])) /
                a_transceiving_config.baud_rate;
            std::uint32_t brr = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
            hkm_assert(brr >= BRR_min && brr <= BRR_max);

            a_p_registers->BRR = brr;
#endif

#if !defined(HKM_ASSERT_ENABLED)
            const std::uint32_t usartdiv =
                (2 *
                 (a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)])) /
                a_transceiving_config.baud_rate;
            a_p_registers->BRR = ((usartdiv & 0xFFF0u) | ((usartdiv & 0xFu) >> 1)) & 0xFFFF;
#endif

            bit_flag::set(&(a_p_registers->CR1), USART_CR1_OVER8);
        }
        break;
    }

    if (USART::Transceiving_config::Flow_control_flag::RS232 ==
        (a_transceiving_config.flow_control & USART::Transceiving_config::Flow_control_flag::RS232))
    {
        a_p_registers->CR3 = bit_flag::get(static_cast<std::uint32_t>(a_transceiving_config.flow_control),
                                           USART_CR3_RTSE & USART_CR3_CTSE);
    }

    if (USART::Transceiving_config::Flow_control_flag::RS485 ==
        (a_transceiving_config.flow_control & USART::Transceiving_config::Flow_control_flag::RS485))
    {
        a_p_registers->CR3 = USART_CR3_DEM;
        bit_flag::set(&(a_p_registers->CR1),
                      bit_flag::get(static_cast<std::uint32_t>(a_transceiving_config.flow_control),
                                    USART_CR1_DEAT_Msk & USART_CR1_DEDT_Msk));
    }

    if (USART::Transceiving_config::Mute_method::character_matched ==
        static_cast<USART::Transceiving_config::Mute_method>(
            static_cast<std::uint32_t>(a_transceiving_config.mute_method) & 0x200u))
    {
        bit_flag::set(&(a_p_registers->CR2),
                      ((bit_flag::get(static_cast<std::uint8_t>(a_transceiving_config.mute_method), 0xFFu))
                       << USART_CR2_ADD_Pos) |
                          USART_CR2_ADDM7);
        bit_flag::set(&(a_p_registers->CR1), USART_CR1_MME | USART_CR1_WAKE);
        a_p_registers->RQR = USART_RQR_MMRQ;
    }
    else if (USART::Transceiving_config::Mute_method::idle_line == a_transceiving_config.mute_method)
    {
        bit_flag::set(&(a_p_registers->CR1), USART_CR1_MME);
    }

    if (USART::Low_power_wakeup_method::none != a_low_power_wakeup)
    {
        bit_flag::set(&(a_p_registers->CR1), USART_CR1_UESM);

        bit_flag::clear(&(a_p_registers->CR1), USART_CR1_UE);
        bit_flag::set(&(a_p_registers->CR3), USART_CR3_WUS_Msk, static_cast<std::uint32_t>(a_low_power_wakeup));
    }

    bit_flag::set(&(a_p_registers->CR2), static_cast<std::uint32_t>(a_transceiving_config.stop_bits));
    bit_flag::set(&(a_p_registers->CR1),
                  static_cast<std::uint32_t>(a_transceiving_config.mode) |
                      static_cast<std::uint32_t>(a_frame_format.parity) |
                      static_cast<std::uint32_t>(a_frame_format.word_length) | USART_CR1_UE);
}

template<typename t_Data>
USART::Polling::Result transmit(USART_TypeDef* a_p_registers, const t_Data* a_p_data, std::size_t a_data_size_in_words)
{
    hkm_assert(a_data_size_in_words > 0);

    bit_flag::set(&(a_p_registers->ICR), USART_ICR_TCCF);

    std::size_t words = 0;
    USART::Event_flag events = USART::Event_flag::none;
    do
    {
        if (true == bit_flag::is(a_p_registers->ISR, USART_ISR_TXE))
        {
            a_p_registers->TDR = a_p_data[words++];
        }
    } while (words < a_data_size_in_words && false == is_tx_error(a_p_registers));

    if (false == is_tx_error(a_p_registers))
    {
        wait_until::all_bits_are_set(a_p_registers->ISR, USART_ISR_TC);
        events |= USART::Event_flag::transfer_complete;
    }

    if (true == bit::is_any(a_p_registers->ISR, USART_ISR_PE | USART_ISR_FE | USART_ISR_NE))
    {
        if (true == bit_flag::is(a_p_registers->ISR, USART_ISR_PE))
        {
            bit_flag::set(&(a_p_registers->ICR), USART_ICR_PECF);
            events |= USART::Event_flag::parity_error;
        }
        if (true == bit_flag::is(a_p_registers->ISR, USART_ISR_NE))
        {
            bit_flag::set(&(a_p_registers->ICR), USART_ICR_NECF);
            events |= USART::Event_flag::noise_detected;
        }
    }

    return { events, words };
}
template<typename t_Data> USART::Polling::Result
transmit(USART_TypeDef* a_p_registers, const t_Data* a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    hkm_assert(a_data_size_in_words > 0);
    hkm_assert(a_timeout > 0_ms);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(a_p_registers->ICR), USART_ICR_TCCF);

    std::size_t words = 0;
    USART::Event_flag events = USART::Event_flag::none;
    do
    {
        if (true == bit_flag::is(a_p_registers->ISR, USART_ISR_TXE))
        {
            a_p_registers->TDR = a_p_data[words++];
        }
    } while (words < a_data_size_in_words && false == is_tx_error(a_p_registers) &&
             a_timeout.get() > tick_counter<Milliseconds>::get() - start);

    if (false == is_tx_error(a_p_registers))
    {
        if (true == wait_until::all_bits_are_set(a_p_registers->ISR,
                                                 USART_ISR_TC,
                                                 a_timeout.get() - (tick_counter<Milliseconds>::get() - start)))
        {
            events |= USART::Event_flag::transfer_complete;
        }
    }

    if (true == bit::is_any(a_p_registers->ISR, USART_ISR_PE | USART_ISR_FE | USART_ISR_NE))
    {
        if (true == bit_flag::is(a_p_registers->ISR, USART_ISR_PE))
        {
            bit_flag::set(&(a_p_registers->ICR), USART_ICR_PECF);
            events |= USART::Event_flag::parity_error;
        }
        if (true == bit_flag::is(a_p_registers->ISR, USART_ISR_NE))
        {
            bit_flag::set(&(a_p_registers->ICR), USART_ICR_NECF);
            events |= USART::Event_flag::noise_detected;
        }
    }

    return { events, words };
}

template<typename t_Data>
USART::Polling::Result receive(USART_TypeDef* a_p_registers, t_Data* a_p_data, std::size_t a_data_size_in_words)
{
    hkm_assert(a_data_size_in_words > 0);

    bit_flag::set(&(a_p_registers->ICR), USART_ICR_IDLECF);

    std::size_t words = 0;

    while (false == bit_flag::is(a_p_registers->ISR, USART_ISR_IDLE) && false == is_rx_error(a_p_registers))
    {
        if (true == bit_flag::is(a_p_registers->ISR, USART_ISR_RXNE))
        {
            if (true == bit_flag::is(a_p_registers->ISR, USART_ISR_CMF) &&
                true == bit_flag::is(a_p_registers->CR1, USART_CR1_MME | USART_CR1_WAKE))
            {
                bit_flag::set(&(a_p_registers->ICR), USART_ICR_CMCF);
            }

            if (words < a_data_size_in_words)
            {
                a_p_data[words++] = static_cast<t_Data>(a_p_registers->RDR);
            }
            else
            {
                if (true == bit_flag::is(a_p_registers->ISR, USART_ISR_RXNE))
                {
                    bit_flag::set(&(a_p_registers->RQR), USART_RQR_RXFRQ);
                }
            }
        }
    }

    return { get_Event_flag_and_clear(&(a_p_registers->ICR), a_p_registers->ISR), words };
}

template<typename t_Data> USART::Polling::Result
receive(USART_TypeDef* a_p_registers, t_Data* a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    hkm_assert(a_data_size_in_words > 0);
    hkm_assert(a_timeout > 0_ms);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    bit_flag::set(&(a_p_registers->ICR), USART_ICR_IDLECF);

    std::size_t words = 0;

    while (false == bit_flag::is(a_p_registers->ISR, USART_ISR_IDLE) && false == is_rx_error(a_p_registers) &&
           a_timeout.get() > tick_counter<Milliseconds>::get() - start)
    {
        if (true == bit_flag::is(a_p_registers->ISR, USART_ISR_RXNE))
        {
            if (words < a_data_size_in_words)
            {
                a_p_data[words++] = static_cast<t_Data>(a_p_registers->RDR);
            }
            else
            {
                bit_flag::set(&(a_p_registers->RQR), USART_RQR_RXFRQ);
                words++;
            }
        }
    }

    if (true == bit_flag::is(a_p_registers->ISR, USART_ISR_CMF))
    {
        bit_flag::set(&(a_p_registers->ICR), USART_ICR_CMCF);
    }

    return { get_Event_flag_and_clear(&(a_p_registers->ICR), a_p_registers->ISR), words };
}

void transmit_start(USART_TypeDef* a_p_registers)
{
    hkm_assert(true == bit_flag::is(a_p_registers->CR1, USART_CR1_TE));

    bit_flag::set(&(a_p_registers->ICR), USART_ICR_TCCF);
    bit_flag::set(&(a_p_registers->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);
}

void receive_start(USART_TypeDef* a_p_registers)
{
    hkm_assert(true == bit_flag::is(a_p_registers->CR1, USART_CR1_RE));

    bit_flag::set(&(a_p_registers->CR1), USART_CR1_RXNEIE);
}

void event_listening_start(USART_TypeDef* a_p_registers, USART::Event_flag events)
{
    // FIXME: if not listening to all errors enabled by USART_CR3_EIE, it will cause interrupt loop due to error bits
    // not being cleared! Solutions: allow only listening to all errors at once? Clear all error bits in event handler?
    clear_events(&a_p_registers->ICR, events);
    if (USART::Event_flag::none != (events & USART::Event_flag::parity_error))
    {
        bit_flag::set(&(a_p_registers->CR1), USART_CR1_PEIE);
    }
    if (USART::Event_flag::none != (events & USART::Event_flag::idle))
    {
        bit_flag::set(&(a_p_registers->CR1), USART_CR1_IDLEIE);
    }
    if (USART::Event_flag::none != (events & USART::Event_flag::transfer_complete))
    {
        bit_flag::set(&(a_p_registers->CR1), USART_CR1_TCIE);
    }
    if (USART::Event_flag::none != (events & USART::Event_flag::character_matched))
    {
        bit_flag::set(&(a_p_registers->CR1), USART_CR1_CMIE);
    }
    bit_flag::set(&(a_p_registers->CR3), USART_CR3_EIE);
}

void USART_interrupt_handler(USART_TypeDef* a_p_registers,
                             USART::Interrupt::Transmit_callback* a_p_transmit_callback,
                             USART::Interrupt::Receive_callback* a_p_receive_callback,
                             USART::Interrupt::Event_callback* a_p_event_callback)
{
    hkm_assert(nullptr != a_p_registers);
    hkm_assert(nullptr != a_p_transmit_callback);
    hkm_assert(nullptr != a_p_receive_callback);
    hkm_assert(nullptr != a_p_event_callback);

    const std::uint32_t isr = a_p_registers->ISR;

    if (nullptr != a_p_event_callback->function)
    {
        USART::Event_flag pending_events = get_pending_events(isr) & a_p_event_callback->events;
        if (USART::Event_flag::none != pending_events)
        {
            a_p_event_callback->function(pending_events, a_p_event_callback->p_user_data);
            clear_events(&a_p_registers->ICR, pending_events);
        }
    }

    if (nullptr != a_p_transmit_callback->function && true == bit_flag::is(isr, USART_ISR_TXE))
    {
        a_p_transmit_callback->function(&(a_p_registers->TDR), a_p_transmit_callback->p_user_data);
    }

    if (nullptr != a_p_receive_callback->function && true == bit_flag::is(isr, USART_ISR_RXNE))
    {
        a_p_receive_callback->function(a_p_registers->RDR, a_p_receive_callback->p_user_data);
    }
}

USART* USART_irq_context[1] = { nullptr };
LPUART* LPUART_irq_context[1] = { nullptr };
} // namespace

extern "C" {
void USART1_IRQHandler()
{
    hkm_assert(nullptr != USART_irq_context[0]);

    USART_interrupt_handler(USART_irq_context[0]);
}
void LPUART1_IRQHandler()
{
    hkm_assert(nullptr != LPUART_irq_context[0]);

    LPUART_interrupt_handler(LPUART_irq_context[0]);
}
}

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {
using namespace xmcu;
using namespace utils;

void USART_interrupt_handler(USART* a_p_this)
{
    hkm_assert(nullptr != a_p_this);

    ::USART_interrupt_handler(a_p_this->p_registers,
                              &(a_p_this->transmit_callback),
                              &(a_p_this->receive_callback),
                              &(a_p_this->event_callback));
}

void LPUART_interrupt_handler(LPUART* a_p_this)
{
    hkm_assert(nullptr != a_p_this);

    ::USART_interrupt_handler(a_p_this->p_registers,
                              &(a_p_this->transmit_callback),
                              &(a_p_this->receive_callback),
                              &(a_p_this->event_callback));
}

void USART::enable(const Clock_config& a_clock_config,
                   const Transceiving_config& a_transceiving_config,
                   const Frame_format& a_frame_format,
                   Low_power_wakeup_method a_low_power_wakeup)
{
    hkm_assert(0 != a_clock_config.freq_Hz);
    hkm_assert(various::get_enum_incorrect_value<Clock_config::Prescaler>() != a_clock_config.prescaler);

    hkm_assert(0 != a_transceiving_config.baud_rate);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Oversampling>() !=
               a_transceiving_config.oversampling);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Stop_bits>() != a_transceiving_config.stop_bits);

    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Flow_control_flag>() !=
               a_transceiving_config.flow_control);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Sampling_method>() !=
               a_transceiving_config.sampling_method);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mode_flag>() != a_transceiving_config.mode);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mute_method>() !=
               a_transceiving_config.mute_method);

    hkm_assert(various::get_enum_incorrect_value<Frame_format::Parity>() != a_frame_format.parity);
    hkm_assert(various::get_enum_incorrect_value<Frame_format::Word_length>() != a_frame_format.word_length);

    ::enable(this->p_registers, a_clock_config, a_transceiving_config, a_frame_format, a_low_power_wakeup);

    wait_until::all_bits_are_set(
        this->p_registers->ISR,
        (true == bit_flag::is(this->p_registers->CR1, USART_CR1_RE) ? (USART_ISR_REACK | USART_ISR_IDLE) : 0) |
            (true == bit_flag::is(this->p_registers->CR1, USART_CR1_TE) ? USART_ISR_TEACK : 0) |
            (a_transceiving_config.mute_method == Transceiving_config::Mute_method::idle_line ||
             true == bit_flag::is(static_cast<std::uint32_t>(a_transceiving_config.mute_method),
                                  static_cast<std::uint32_t>(Transceiving_config::Mute_method::character_matched) ?
                                      USART_ISR_RWU :
                                      0x0u)));

    if (true == bit_flag::is(this->p_registers->ISR, USART_ISR_IDLE))
    {
        bit_flag::set(&(this->p_registers->ICR), USART_ICR_IDLECF);
    }
}
bool USART::enable(const Clock_config& a_clock_config,
                   const Transceiving_config& a_transceiving_config,
                   const Frame_format& a_frame_format,
                   Low_power_wakeup_method a_low_power_wakeup,
                   Milliseconds a_timeout)
{
    hkm_assert(0 != a_clock_config.freq_Hz);
    hkm_assert(various::get_enum_incorrect_value<Clock_config::Prescaler>() != a_clock_config.prescaler);

    hkm_assert(0 != a_transceiving_config.baud_rate);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Oversampling>() !=
               a_transceiving_config.oversampling);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Stop_bits>() != a_transceiving_config.stop_bits);

    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Flow_control_flag>() !=
               a_transceiving_config.flow_control);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Sampling_method>() !=
               a_transceiving_config.sampling_method);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mode_flag>() != a_transceiving_config.mode);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mute_method>() !=
               a_transceiving_config.mute_method);

    hkm_assert(various::get_enum_incorrect_value<Frame_format::Parity>() != a_frame_format.parity);
    hkm_assert(various::get_enum_incorrect_value<Frame_format::Word_length>() != a_frame_format.word_length);

    hkm_assert(a_timeout > 0_ms);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    ::enable(this->p_registers, a_clock_config, a_transceiving_config, a_frame_format, a_low_power_wakeup);

    bool ret = wait_until::all_bits_are_set(
        this->p_registers->ISR,
        (true == bit_flag::is(this->p_registers->CR1, USART_CR1_RE) ? (USART_ISR_REACK | USART_ISR_IDLE) : 0) |
            (true == bit_flag::is(this->p_registers->CR1, USART_CR1_TE) ? USART_ISR_TEACK : 0) |
            (a_transceiving_config.mute_method == Transceiving_config::Mute_method::idle_line ||
             true == bit_flag::is(static_cast<std::uint32_t>(a_transceiving_config.mute_method),
                                  static_cast<std::uint32_t>(Transceiving_config::Mute_method::character_matched) ?
                                      USART_ISR_RWU :
                                      0x0u)),
        a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == bit_flag::is(this->p_registers->ISR, USART_ISR_IDLE))
    {
        bit_flag::set(&(this->p_registers->ICR), USART_ICR_IDLECF);
    }

    return ret;
}

bool USART::enable_rx(Milliseconds a_timeout)
{
    bit_flag::set(&(this->p_registers->CR1), USART_CR1_RE);
    return wait_until::all_bits_are_set(this->p_registers->ISR, USART_ISR_REACK, a_timeout);
}
bool USART::disable_rx(Milliseconds a_timeout)
{
    bit_flag::clear(&(this->p_registers->CR1), USART_CR1_RE);
    return wait_until::all_bits_are_cleared(this->p_registers->ISR, USART_ISR_REACK, a_timeout);
}

bool USART::enable_tx(Milliseconds a_timeout)
{
    bit_flag::set(&(this->p_registers->CR1), USART_CR1_TE);
    return wait_until::all_bits_are_set(this->p_registers->ISR, USART_ISR_TEACK, a_timeout);
}
bool USART::disable_tx(Milliseconds a_timeout)
{
    bit_flag::clear(&(this->p_registers->CR1), USART_CR1_TE);
    return wait_until::all_bits_are_cleared(this->p_registers->ISR, USART_ISR_TEACK, a_timeout);
}

void USART::disable()
{
    if (true == this->interrupt.is_enabled())
    {
        this->interrupt.disable();
    }

    this->p_registers->CR1 = 0;
    this->p_registers->CR2 = 0;
    this->p_registers->CR3 = 0;
}

USART::Polling::Result USART::Polling::transmit(Not_null<const std::uint8_t*> a_p_data,
                                                std::size_t a_data_size_in_words)
{
    return ::transmit<std::uint8_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words);
}
USART::Polling::Result USART::Polling::transmit(Not_null<const std::uint16_t*> a_p_data,
                                                std::size_t a_data_size_in_words)
{
    return ::transmit<std::uint16_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words);
}

USART::Polling::Result USART::Polling::transmit(Not_null<const std::uint8_t*> a_p_data,
                                                std::size_t a_data_size_in_words,
                                                Milliseconds a_timeout)
{
    return ::transmit<std::uint8_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}
USART::Polling::Result USART::Polling::transmit(Not_null<const std::uint16_t*> a_p_data,
                                                std::size_t a_data_size_in_words,
                                                Milliseconds a_timeout)
{
    return ::transmit<std::uint16_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}

USART::Polling::Result USART::Polling::receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words)
{
    return ::receive<std::uint8_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words);
}
USART::Polling::Result USART::Polling::receive(Not_null<std::uint16_t*> a_p_data, std::size_t a_data_size_in_words)
{
    return ::receive<std::uint16_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words);
}

USART::Polling::Result
USART::Polling::receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    return ::receive<std::uint8_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}
USART::Polling::Result
USART::Polling::receive(Not_null<std::uint16_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    return ::receive<std::uint16_t>(this->p_USART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}

bool USART::Polling::is_transfer_complete()
{
    Event_flag pending_events = get_pending_events(this->p_USART->p_registers->ISR);
    if (Event_flag::none != (pending_events & Event_flag::transfer_complete))
    {
        clear_events(&this->p_USART->p_registers->ICR, Event_flag::transfer_complete);
        return true;
    }
    return false;
}

void USART::Interrupt::enable(const IRQ_config& a_config)
{
    hkm_assert(nullptr == USART_irq_context[this->p_USART->idx]);

    USART_irq_context[this->p_USART->idx] = this->p_USART;

    NVIC_SetPriority(this->p_USART->irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_config.preempt_priority, a_config.sub_priority));
    NVIC_EnableIRQ(this->p_USART->irqn);
}

void USART::Interrupt::disable()
{
    hkm_assert(nullptr != USART_irq_context[this->p_USART->idx]);

    this->transmit_stop();
    this->receive_stop();
    this->event_listening_stop();

    NVIC_DisableIRQ(this->p_USART->irqn);

    bit_flag::clear(&(this->p_USART->p_registers->CR1), USART_CR1_UESM);
    bit_flag::clear(&(this->p_USART->p_registers->CR3), USART_CR3_WUS_Msk);

    USART_irq_context[this->p_USART->idx] = nullptr;
}

void USART::Interrupt::transmit_start(const Transmit_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;
    this->p_USART->transmit_callback = a_callback;

    ::transmit_start(this->p_USART->p_registers);
}

void USART::Interrupt::receive_start(const Receive_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;
    this->p_USART->receive_callback = a_callback;

    ::receive_start(this->p_USART->p_registers);
}

void USART::Interrupt::event_listening_start(const Event_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;

    this->p_USART->event_callback = a_callback;

    ::event_listening_start(this->p_USART->p_registers, a_callback.events);
}

void USART::Interrupt::transmit_stop()
{
    Scoped_guard<nvic> guard;

    bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_USART))->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);

    this->p_USART->transmit_callback = { .function = nullptr, .p_user_data = nullptr };
}

void USART::Interrupt::receive_stop()
{
    Scoped_guard<nvic> guard;

    bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_USART))->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);

    this->p_USART->receive_callback = { .function = nullptr, .p_user_data = nullptr };
}

void USART::Interrupt::event_listening_stop()
{
    Scoped_guard<nvic> guard;

    bit_flag::clear(&(this->p_USART->p_registers->CR1), USART_CR1_PEIE | USART_CR1_IDLEIE);
    bit_flag::clear(&(this->p_USART->p_registers->CR3), USART_CR3_EIE);

    this->p_USART->event_callback = { .events = USART::Event_flag::none, .function = nullptr, .p_user_data = nullptr };
}

void LPUART::enable(const Clock_config& a_clock_config,
                    const Transceiving_config& a_transceiving_config,
                    const Frame_format& a_frame_format,
                    Low_power_wakeup_method a_low_power_wakeup)
{
    hkm_assert(0 != a_clock_config.freq_Hz);
    hkm_assert(various::get_enum_incorrect_value<Clock_config::Prescaler>() != a_clock_config.prescaler);

    hkm_assert(0 != a_transceiving_config.baud_rate);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Stop_bits>() != a_transceiving_config.stop_bits);

    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Flow_control_flag>() !=
               a_transceiving_config.flow_control);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Sampling_method>() !=
               a_transceiving_config.sampling_method);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mode_flag>() != a_transceiving_config.mode);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mute_method>() !=
               a_transceiving_config.mute_method);

    hkm_assert(various::get_enum_incorrect_value<USART::Frame_format::Parity>() != a_frame_format.parity);
    hkm_assert(various::get_enum_incorrect_value<USART::Frame_format::Word_length>() != a_frame_format.word_length);

    hkm_assert(((a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) >=
                (3u * a_transceiving_config.baud_rate)) ||
               ((a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) <=
                (4096u * a_transceiving_config.baud_rate)));

    ::enable(this->p_registers, a_clock_config, a_transceiving_config, a_frame_format, a_low_power_wakeup);

    return wait_until::all_bits_are_set(
        this->p_registers->ISR,
        (true == bit_flag::is(this->p_registers->CR1, USART_CR1_RE) ? (USART_ISR_REACK | USART_ISR_IDLE) : 0) |
            (true == bit_flag::is(this->p_registers->CR1, USART_CR1_TE) ? USART_ISR_TEACK : 0) |
            (a_transceiving_config.mute_method == Transceiving_config::Mute_method::idle_line ||
             true ==
                 bit_flag::is(static_cast<std::uint32_t>(a_transceiving_config.mute_method),
                              static_cast<std::uint32_t>(USART::Transceiving_config::Mute_method::character_matched) ?
                                  USART_ISR_RWU :
                                  0x0u)));
}

bool LPUART::enable(const Clock_config& a_clock_config,
                    const Transceiving_config& a_transceiving_config,
                    const Frame_format& a_frame_format,
                    Low_power_wakeup_method a_low_power_wakeup,
                    Milliseconds a_timeout)
{
    hkm_assert(0 != a_clock_config.freq_Hz);
    hkm_assert(various::get_enum_incorrect_value<Clock_config::Prescaler>() != a_clock_config.prescaler);

    hkm_assert(0 != a_transceiving_config.baud_rate);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Stop_bits>() != a_transceiving_config.stop_bits);

    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Flow_control_flag>() !=
               a_transceiving_config.flow_control);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Sampling_method>() !=
               a_transceiving_config.sampling_method);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mode_flag>() != a_transceiving_config.mode);
    hkm_assert(various::get_enum_incorrect_value<Transceiving_config::Mute_method>() !=
               a_transceiving_config.mute_method);

    hkm_assert(various::get_enum_incorrect_value<USART::Frame_format::Parity>() != a_frame_format.parity);
    hkm_assert(various::get_enum_incorrect_value<USART::Frame_format::Word_length>() != a_frame_format.word_length);

    hkm_assert(a_timeout > 0_ms);

    hkm_assert(((a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) >=
                (3u * a_transceiving_config.baud_rate)) ||
               ((a_clock_config.freq_Hz / clock_prescaler_lut[static_cast<std::uint32_t>(a_clock_config.prescaler)]) <=
                (4096u * a_transceiving_config.baud_rate)));

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    ::enable(this->p_registers, a_clock_config, a_transceiving_config, a_frame_format, a_low_power_wakeup);

    return wait_until::all_bits_are_set(
        this->p_registers->ISR,
        (true == bit_flag::is(this->p_registers->CR1, USART_CR1_RE) ? (USART_ISR_REACK | USART_ISR_IDLE) : 0) |
            (true == bit_flag::is(this->p_registers->CR1, USART_CR1_TE) ? USART_ISR_TEACK : 0) |
            (a_transceiving_config.mute_method == Transceiving_config::Mute_method::idle_line ||
             true ==
                 bit_flag::is(static_cast<std::uint32_t>(a_transceiving_config.mute_method),
                              static_cast<std::uint32_t>(USART::Transceiving_config::Mute_method::character_matched) ?
                                  USART_ISR_RWU :
                                  0x0u)),
        a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}

void LPUART::disable()
{
    if (true == this->interrupt.is_enabled())
    {
        this->interrupt.disable();
    }

    this->p_registers->CR1 = 0;
    this->p_registers->CR2 = 0;
    this->p_registers->CR3 = 0;
}

LPUART::Polling::Result LPUART::Polling::transmit(Not_null<const std::uint8_t*> a_p_data,
                                                  std::size_t a_data_size_in_words)
{
    return ::transmit<std::uint8_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words);
}
LPUART::Polling::Result LPUART::Polling::transmit(Not_null<const std::uint16_t*> a_p_data,
                                                  std::size_t a_data_size_in_words)
{
    return ::transmit<std::uint16_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words);
}

LPUART::Polling::Result LPUART::Polling::transmit(Not_null<const std::uint8_t*> a_p_data,
                                                  std::size_t a_data_size_in_words,
                                                  Milliseconds a_timeout)
{
    return ::transmit<std::uint8_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}
LPUART::Polling::Result LPUART::Polling::transmit(Not_null<const std::uint16_t*> a_p_data,
                                                  std::size_t a_data_size_in_words,
                                                  Milliseconds a_timeout)
{
    return ::transmit<std::uint16_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}

LPUART::Polling::Result LPUART::Polling::receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words)
{
    return ::receive<std::uint8_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words);
}
LPUART::Polling::Result LPUART::Polling::receive(Not_null<std::uint16_t*> a_p_data, std::size_t a_data_size_in_words)
{
    return ::receive<std::uint16_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words);
}

LPUART::Polling::Result
LPUART::Polling::receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    return ::receive<std::uint8_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}
LPUART::Polling::Result
LPUART::Polling::receive(Not_null<std::uint16_t*> a_p_data, std::size_t a_data_size_in_words, Milliseconds a_timeout)
{
    return ::receive<std::uint16_t>(this->p_LPUART->p_registers, a_p_data, a_data_size_in_words, a_timeout);
}

void LPUART::Interrupt::enable(const IRQ_config& a_config)
{
    hkm_assert(nullptr == LPUART_irq_context[this->p_LPUART->idx]);

    LPUART_irq_context[this->p_LPUART->idx] = this->p_LPUART;

    NVIC_SetPriority(this->p_LPUART->irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_config.preempt_priority, a_config.sub_priority));
    NVIC_EnableIRQ(this->p_LPUART->irqn);
}

void LPUART::Interrupt::disable()
{
    hkm_assert(nullptr != LPUART_irq_context[this->p_LPUART->idx]);

    this->transmit_stop();
    this->receive_stop();
    this->event_listening_stop();

    NVIC_DisableIRQ(this->p_LPUART->irqn);

    LPUART_irq_context[this->p_LPUART->idx] = nullptr;
}

void LPUART::Interrupt::transmit_start(const Transmit_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;
    this->p_LPUART->transmit_callback = a_callback;

    ::transmit_start(this->p_LPUART->p_registers);
}

void LPUART::Interrupt::receive_start(const Receive_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;
    this->p_LPUART->receive_callback = a_callback;

    ::receive_start(this->p_LPUART->p_registers);
}

void LPUART::Interrupt::event_listening_start(const Event_callback& a_callback)
{
    hkm_assert(nullptr != a_callback.function);

    Scoped_guard<nvic> guard;

    this->p_LPUART->event_callback = a_callback;

    ::event_listening_start(this->p_LPUART->p_registers, a_callback.events);
}

void LPUART::Interrupt::transmit_stop()
{
    Scoped_guard<nvic> guard;

    bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_LPUART))->CR1), USART_CR1_TXEIE | USART_CR1_TCIE);

    this->p_LPUART->transmit_callback = { .function = nullptr, .p_user_data = nullptr };
}

void LPUART::Interrupt::receive_stop()
{
    Scoped_guard<nvic> guard;

    bit_flag::clear(&(static_cast<USART_TypeDef*>(*(this->p_LPUART))->CR1), USART_CR1_RXNEIE | USART_CR1_IDLEIE);

    this->p_LPUART->receive_callback = { .function = nullptr, .p_user_data = nullptr };
}

void LPUART::Interrupt::event_listening_stop()
{
    Scoped_guard<nvic> guard;

    bit_flag::clear(&(this->p_LPUART->p_registers->CR1), USART_CR1_PEIE | USART_CR1_IDLEIE);
    bit_flag::clear(&(this->p_LPUART->p_registers->CR3), USART_CR3_EIE);

    this->p_LPUART->event_callback = { .events = LPUART::Event_flag::none,
                                       .function = nullptr,
                                       .p_user_data = nullptr };
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

template<> template<> void rcc<peripherals::USART, 1u>::enable<rcc<mcu<1u>>::pclk<2u>>(bool a_enable_in_lp)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART1SEL);
    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN);
    }
}
template<> template<> void rcc<peripherals::USART, 1u>::enable<rcc<mcu<1u>>>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART1SEL, RCC_CCIPR_USART1SEL_0);
    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN);
    }
}
template<> template<> void rcc<peripherals::USART, 1u>::enable<hsi16>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART1SEL, RCC_CCIPR_USART1SEL_1);
    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN);
    }
}
template<> template<> void rcc<peripherals::USART, 1u>::enable<lse>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_USART1SEL);
    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB2SMENR), RCC_APB2SMENR_USART1SMEN);
    }
}
template<> void rcc<peripherals::USART, 1u>::disable()
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_USART1SEL);
    bit_flag::clear(&(RCC->APB2ENR), RCC_APB2ENR_USART1EN);
}

template<> template<> void rcc<peripherals::LPUART, 1u>::enable<rcc<mcu<1u>>::pclk<1u>>(bool a_enable_in_lp)
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_LPUART1SEL);
    bit_flag::set(&(RCC->APB1ENR2), RCC_APB1ENR2_LPUART1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPUART1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPUART1SMEN);
    }
}
template<> template<> void rcc<peripherals::LPUART, 1u>::enable<rcc<mcu<1u>>>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_LPUART1SEL, RCC_CCIPR_LPUART1SEL_0);
    bit_flag::set(&(RCC->APB1ENR2), RCC_APB1ENR2_LPUART1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPUART1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPUART1SMEN);
    }
}
template<> template<> void rcc<peripherals::LPUART, 1u>::enable<hsi16>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_LPUART1SEL, RCC_CCIPR_LPUART1SEL_1);
    bit_flag::set(&(RCC->APB1ENR2), RCC_APB1ENR2_LPUART1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPUART1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPUART1SMEN);
    }
}
template<> template<> void rcc<peripherals::LPUART, 1u>::enable<lse>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->CCIPR), RCC_CCIPR_LPUART1SEL_0 | RCC_CCIPR_LPUART1SEL_1);
    bit_flag::set(&(RCC->APB1ENR2), RCC_APB1ENR2_LPUART1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPUART1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPUART1SMEN);
    }
}

template<> void rcc<peripherals::LPUART, 1>::disable()
{
    bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_LPUART1SEL);
    bit_flag::clear(&(RCC->APB1ENR2), RCC_APB1ENR2_LPUART1EN);
    bit_flag::clear(&(RCC->APB1SMENR2), RCC_APB1SMENR2_LPUART1SMEN);
}
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif
