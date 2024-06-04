/**/

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/USART/DMA.hpp>

// xmcus
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m4/nvic.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb;
using namespace xmcu::soc::m4::stm32wb::utils;

void transmit(USART_TypeDef* a_p_USART_registers,
              DMA_Channel_TypeDef* a_p_channel_registers,
              std::uint32_t a_channel_flags,
              volatile const void* a_p_buffer,
              std::uint16_t a_buffer_size_in_words)
{
    hkm_assert(false == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_EN));

    a_p_channel_registers->CNDTR = a_buffer_size_in_words;
    a_p_channel_registers->CPAR = reinterpret_cast<std::uint32_t>(&(a_p_USART_registers->TDR));
    a_p_channel_registers->CMAR = reinterpret_cast<std::uint32_t>(a_p_buffer);

    bit_flag::set(&(a_p_USART_registers->ICR), USART_ICR_TCCF);
    bit_flag::set(&(a_p_channel_registers->CCR), a_channel_flags | DMA_CCR_EN);
}

void receive(USART_TypeDef* a_p_USART_registers,
             DMA_Channel_TypeDef* a_p_channel_registers,
             std::uint32_t a_channel_flags,
             volatile void* a_p_buffer,
             std::uint16_t a_buffer_size_in_words)
{
    hkm_assert(false == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_EN));

    a_p_channel_registers->CNDTR = a_buffer_size_in_words;
    a_p_channel_registers->CPAR = reinterpret_cast<std::uint32_t>(&(a_p_USART_registers->RDR));
    a_p_channel_registers->CMAR = reinterpret_cast<std::uint32_t>(a_p_buffer);

    bit_flag::set(&(a_p_channel_registers->CCR), a_channel_flags | DMA_CCR_EN);
}
DMA<>::Event_flag
get_Event_flag_and_clear(std::uint32_t a_isr, volatile std::uint32_t* a_p_icr, DMA<>::Channel a_channel)
{
    DMA<>::Event_flag ret = DMA<>::Event_flag::none;
    const std::uint32_t f = static_cast<std::uint32_t>(a_channel) * 4u;

    if (true == bit_flag::is(a_isr, 0x1u << (DMA_ISR_TCIF1_Pos + f)))
    {
        ret |= DMA<>::Event_flag::full_transfer_complete;
    }
    if (true == bit_flag::is(a_isr, 0x1u << (DMA_ISR_HTIF1_Pos + f)))
    {
        ret |= DMA<>::Event_flag::half_transfer_complete;
    }
    if (true == bit_flag::is(a_isr, 0x1u << (DMA_ISR_TEIF1_Pos + f)))
    {
        ret |= DMA<>::Event_flag::transfer_error;
    }

    bit_flag::set(a_p_icr, 0x1u << (DMA_IFCR_CGIF1_Pos + f));

    return ret;
}

std::uint32_t get_interrupt_enable_flags(DMA<>::Event_flag a_event_flag)
{
    std::uint32_t ret = 0;

    if (DMA<>::Event_flag::full_transfer_complete == (DMA<>::Event_flag::full_transfer_complete & a_event_flag))
    {
        ret |= DMA_CCR_TCIE;
    }
    if (DMA<>::Event_flag::half_transfer_complete == (DMA<>::Event_flag::half_transfer_complete & a_event_flag))
    {
        ret |= DMA_CCR_HTIE;
    }
    if (DMA<>::Event_flag::transfer_error == (DMA<>::Event_flag::transfer_error & a_event_flag))
    {
        ret |= DMA_CCR_TEIE;
    }

    return ret;
}

} // namespace

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
using namespace xmcu::soc::m4::stm32wb::peripherals;

void DMA<USART>::Receiver::enable(DMA<>::Channel a_channel)
{
    switch (this->p_DMA->idx)
    {
        case 0x0u: {
            this->p_DMA->p_rx_channel_registers = reinterpret_cast<DMA_Channel_TypeDef*>(
                DMA1_Channel1_BASE + (static_cast<std::uint32_t>(a_channel) * 0x14u));
            this->p_DMA->rx_irqn = static_cast<IRQn_Type>(DMA1_Channel1_IRQn + static_cast<std::uint32_t>(a_channel));

            hkm_assert(false == bit_flag::is(this->p_DMA->p_rx_channel_registers->CCR, DMA_CCR_EN));

            reinterpret_cast<DMAMUX_Channel_TypeDef*>(DMAMUX1_Channel0_BASE +
                                                      (static_cast<std::uint32_t>(a_channel) * 4))
                ->CCR = 0xE;
        }
        break;

        case 0x1u: {
            this->p_DMA->p_rx_channel_registers = reinterpret_cast<DMA_Channel_TypeDef*>(
                DMA2_Channel1_BASE + (static_cast<std::uint32_t>(a_channel) * 0x14u));
            this->p_DMA->rx_irqn = static_cast<IRQn_Type>(DMA2_Channel1_IRQn + static_cast<std::uint32_t>(a_channel));

            hkm_assert(false == bit_flag::is(this->p_DMA->p_rx_channel_registers->CCR, DMA_CCR_EN));

            reinterpret_cast<DMAMUX_Channel_TypeDef*>(DMAMUX1_Channel7_BASE +
                                                      (static_cast<std::uint32_t>(a_channel) * 4))
                ->CCR = 0xE;
        }
        break;
    }

    bit_flag::set(&(this->p_DMA->p_USART_registers->CR3), USART_CR3_DMAR);

    this->p_DMA->rx_channel = a_channel;
}
void DMA<USART>::Receiver::disable()
{
    if (this->interrupt.is_enabled())
    {
        this->interrupt.disable();
    }

    this->p_DMA->p_rx_channel_registers->CCR = 0x0u;
    this->p_DMA->p_rx_channel_registers->CNDTR = 0x0u;
    this->p_DMA->p_rx_channel_registers->CPAR = 0x0u;
    this->p_DMA->p_rx_channel_registers->CMAR = 0x0u;

    bit_flag::clear(
        &(reinterpret_cast<DMAMUX_Channel_TypeDef*>(
              DMAMUX1_Channel7_BASE + (static_cast<std::uint32_t>(this->p_DMA->p_rx_channel_registers->CCR) * 4))
              ->CCR),
        0x3fu);

    bit_flag::clear(&(this->p_DMA->p_USART_registers->CR3), USART_CR3_DMAR);
}

void DMA<USART>::Receiver::Polling::receive(DMA<>::Priority a_priority,
                                            DMA<>::Mode a_mode,
                                            Not_null<volatile void*> a_p_buffer,
                                            std::uint16_t a_buffer_size_in_words)
{
    ::receive(this->p_DMA->p_USART_registers,
              this->p_DMA->p_rx_channel_registers,
              static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                  (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) |
                  (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                       (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                       0x0u),
              a_p_buffer,
              a_buffer_size_in_words);
}
// DMA<>::Result DMA<USART>::Receiver::Polling::receive(DMA<>::Priority a_priority,
//                                                      DMA<>::Mode a_mode,
//                                                      Not_null<volatile void*> a_p_buffer,
//                                                      std::uint16_t a_buffer_size_in_words,
//                                                      bool wait_until_channel_disabled,
//                                                      Milliseconds a_timeout)
//{
//     std::uint64_t start = tick_counter<Milliseconds>::get();
//
//     ::receive(this->p_DMA->p_USART_registers,
//               this->p_DMA->p_rx_channel_registers,
//               static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
//                   (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) |
//                   (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
//                        (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
//                        0x0u),
//               a_p_buffer,
//               a_buffer_size_in_words);
//
//     if (true == wait_until_channel_disabled)
//     {
//         bool is_cleared = wait_until::any_bit_is_cleared(this->p_DMA->p_rx_channel_registers->CCR,
//                                                          DMA_CCR_EN,
//                                                          a_timeout.get() - (tick_counter<Milliseconds>::get() -
//                                                          start));
//         if (false == is_cleared)
//         {
//             bit_flag::clear(&(this->p_DMA->p_rx_channel_registers->CCR), DMA_CCR_EN);
//         }
//     }
//     else
//     {
//         wait_until::any_bit_is_set(
//             this->p_DMA->p_DMA_registers->ISR,
//             (0x1u << (DMA_ISR_TCIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->rx_channel) * 4u)))) |
//                 (0x1u << (DMA_ISR_TEIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->rx_channel) * 4u)))),
//             a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
//
//         bit_flag::clear(&(this->p_DMA->p_rx_channel_registers->CCR), DMA_CCR_EN);
//     }
//
//     return { get_Event_flag_and_clear(
//                  this->p_DMA->p_DMA_registers->ISR, &(this->p_DMA->p_DMA_registers->IFCR), this->p_DMA->rx_channel),
//              a_buffer_size_in_words - this->p_DMA->p_rx_channel_registers->CNDTR };
// }

void DMA<USART>::Receiver::Interrupt::enable(const IRQ_config& a_irq_config,
                                             const DMA<>::Callback& a_callback,
                                             DMA<>::Event_flag a_flag)
{
    Scoped_guard<nvic> nvic_guard;

    this->p_DMA->rx_callback = a_callback;
    this->set_context();
    bit_flag::set(&(this->p_DMA->p_rx_channel_registers->CCR), get_interrupt_enable_flags(a_flag));

    NVIC_SetPriority(
        this->p_DMA->rx_irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->p_DMA->rx_irqn);
}
void DMA<USART>::Receiver::Interrupt::disable()
{
    Scoped_guard<nvic> nvic_guard;

    bit_flag::clear(&(this->p_DMA->p_rx_channel_registers->CCR), DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
    this->clear_context();
    this->p_DMA->rx_callback = { .function = nullptr, .p_user_data = nullptr };

    NVIC_DisableIRQ(this->p_DMA->rx_irqn);
}
bool DMA<USART>::Receiver::Interrupt::is_enabled() const
{
    return 1u == NVIC_GetEnableIRQ(this->p_DMA->rx_irqn);
}
void DMA<USART>::Receiver::Interrupt::start(DMA<>::Priority a_priority,
                                            DMA<>::Mode a_mode,
                                            Not_null<volatile void*> a_p_buffer,
                                            std::uint16_t a_buffer_size_in_words)
{
    bit_flag::set(&(this->p_DMA->p_DMA_registers->IFCR),
                  0x1u << (DMA_IFCR_CGIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->rx_channel) * 4u))));

    ::receive(this->p_DMA->p_USART_registers,
              this->p_DMA->p_rx_channel_registers,
              static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                  (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) |
                  (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                       (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                       0x0u),
              a_p_buffer,
              a_buffer_size_in_words);
}
void DMA<USART>::Receiver::Interrupt::stop()
{
    bit_flag::clear(&(this->p_DMA->p_rx_channel_registers->CCR), DMA_CCR_EN);
}

void DMA<USART>::Transmitter::enable(DMA<>::Channel a_channel)
{
    switch (this->p_DMA->idx)
    {
        case 0x0u: {
            this->p_DMA->p_tx_channel_registers = reinterpret_cast<DMA_Channel_TypeDef*>(
                DMA1_Channel1_BASE + (static_cast<std::uint32_t>(a_channel) * 0x14u));
            this->p_DMA->tx_irqn = static_cast<IRQn_Type>(DMA1_Channel1_IRQn + static_cast<std::uint32_t>(a_channel));

            hkm_assert(false == bit_flag::is(this->p_DMA->p_tx_channel_registers->CCR, DMA_CCR_EN));

            reinterpret_cast<DMAMUX_Channel_TypeDef*>(DMAMUX1_Channel0_BASE +
                                                      (static_cast<std::uint32_t>(a_channel) * 4))
                ->CCR = 0xF;
        }
        break;

        case 0x1u: {
            this->p_DMA->p_tx_channel_registers = reinterpret_cast<DMA_Channel_TypeDef*>(
                DMA2_Channel1_BASE + (static_cast<std::uint32_t>(a_channel) * 0x14u));
            this->p_DMA->tx_irqn = static_cast<IRQn_Type>(DMA2_Channel1_IRQn + static_cast<std::uint32_t>(a_channel));

            hkm_assert(false == bit_flag::is(this->p_DMA->p_tx_channel_registers->CCR, DMA_CCR_EN));

            reinterpret_cast<DMAMUX_Channel_TypeDef*>(DMAMUX1_Channel7_BASE +
                                                      (static_cast<std::uint32_t>(a_channel) * 4))
                ->CCR = 0xF;
        }
        break;
    }

    bit_flag::set(&(this->p_DMA->p_USART_registers->CR3), USART_CR3_DMAT);

    this->p_DMA->tx_channel = a_channel;
}
void DMA<USART>::Transmitter::disable()
{
    this->p_DMA->p_tx_channel_registers->CCR = 0x0u;
    this->p_DMA->p_tx_channel_registers->CNDTR = 0x0u;
    this->p_DMA->p_tx_channel_registers->CPAR = 0x0u;
    this->p_DMA->p_tx_channel_registers->CMAR = 0x0u;

    bit_flag::clear(
        &(reinterpret_cast<DMAMUX_Channel_TypeDef*>(
              DMAMUX1_Channel7_BASE + (static_cast<std::uint32_t>(this->p_DMA->p_tx_channel_registers->CCR) * 4))
              ->CCR),
        0x3fu);

    bit_flag::clear(&(this->p_DMA->p_USART_registers->CR3), USART_CR3_DMAT);
}

DMA<>::Result DMA<USART>::Transmitter::Polling::transmit(DMA<>::Priority a_priority,
                                                         DMA<>::Mode a_mode,
                                                         Not_null<const void*> a_p_buffer,
                                                         std::uint16_t a_buffer_size_in_words)
{
    ::transmit(this->p_DMA->p_USART_registers,
               this->p_DMA->p_tx_channel_registers,
               static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                   (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) | DMA_CCR_DIR |
                   (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                        (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                        0x0u),
               a_p_buffer,
               a_buffer_size_in_words);

    wait_until::any_bit_is_set(
        this->p_DMA->p_DMA_registers->ISR,
        (0x1u << (DMA_ISR_TCIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->tx_channel) * 4u)))) |
            (0x1u << (DMA_ISR_TEIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->tx_channel) * 4u)))));

    bit_flag::clear(&(this->p_DMA->p_tx_channel_registers->CCR), DMA_CCR_EN);

    return { get_Event_flag_and_clear(
                 this->p_DMA->p_DMA_registers->ISR, &(this->p_DMA->p_DMA_registers->IFCR), this->p_DMA->tx_channel),
             a_buffer_size_in_words - this->p_DMA->p_tx_channel_registers->CNDTR };
}
DMA<>::Result DMA<USART>::Transmitter::Polling::transmit(DMA<>::Priority a_priority,
                                                         DMA<>::Mode a_mode,
                                                         Not_null<const void*> a_p_buffer,
                                                         std::uint16_t a_buffer_size_in_words,
                                                         Milliseconds a_timeout)
{
    std::uint64_t start = tick_counter<Milliseconds>::get();

    ::transmit(this->p_DMA->p_USART_registers,
               this->p_DMA->p_tx_channel_registers,
               static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                   (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) | DMA_CCR_DIR |
                   (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                        (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                        0x0u),
               a_p_buffer,
               a_buffer_size_in_words);

    wait_until::any_bit_is_set(
        this->p_DMA->p_DMA_registers->ISR,
        (0x1u << (DMA_ISR_TCIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->tx_channel) * 4u)))) |
            (0x1u << (DMA_ISR_TEIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->tx_channel) * 4u)))),
        a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    bit_flag::clear(&(this->p_DMA->p_tx_channel_registers->CCR), DMA_CCR_EN);

    return { .event = get_Event_flag_and_clear(
                 this->p_DMA->p_DMA_registers->ISR, &(this->p_DMA->p_DMA_registers->IFCR), this->p_DMA->tx_channel),
             .data_length_in_words = a_buffer_size_in_words - this->p_DMA->p_tx_channel_registers->CNDTR };
}

void DMA<USART>::Transmitter::Interrupt::enable(const IRQ_config& a_irq_config,
                                                const DMA<>::Callback& a_callback,
                                                DMA<>::Event_flag a_flag)
{
    Scoped_guard<nvic> nvic_guard;

    this->p_DMA->tx_callback = a_callback;
    this->set_context();
    bit_flag::set(&(this->p_DMA->p_tx_channel_registers->CCR), get_interrupt_enable_flags(a_flag));

    NVIC_SetPriority(
        this->p_DMA->tx_irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->p_DMA->tx_irqn);
}
void DMA<USART>::Transmitter::Interrupt::disable()
{
    Scoped_guard<nvic> nvic_guard;

    bit_flag::clear(&(this->p_DMA->p_tx_channel_registers->CCR), DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
    this->clear_context();
    this->p_DMA->rx_callback = { .function = nullptr, .p_user_data = nullptr };

    NVIC_DisableIRQ(this->p_DMA->tx_irqn);
}
bool DMA<USART>::Transmitter::Interrupt::is_enabled() const
{
    return 1u == NVIC_GetEnableIRQ(this->p_DMA->tx_irqn);
}
void DMA<USART>::Transmitter::Interrupt::start(DMA<>::Priority a_priority,
                                               DMA<>::Mode a_mode,
                                               Not_null<volatile const void*> a_p_buffer,
                                               std::uint16_t a_buffer_size_in_words)
{
    bit_flag::set(&(this->p_DMA->p_DMA_registers->IFCR),
                  0x1u << (DMA_IFCR_CGIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->tx_channel) * 4u))));

    ::transmit(this->p_DMA->p_USART_registers,
               this->p_DMA->p_tx_channel_registers,
               static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                   (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) | DMA_CCR_DIR |
                   (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                        (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                        0x0u),
               a_p_buffer,
               a_buffer_size_in_words);
}
void DMA<USART>::Transmitter::Interrupt::stop()
{
    bit_flag::clear(&(this->p_DMA->p_tx_channel_registers->CCR), DMA_CCR_EN);
}

void DMA<LPUART>::Receiver::enable(DMA<>::Channel a_channel)
{
    switch (this->p_DMA->idx)
    {
        case 0x0u: {
            this->p_DMA->p_rx_channel_registers = reinterpret_cast<DMA_Channel_TypeDef*>(
                DMA1_Channel1_BASE + (static_cast<std::uint32_t>(a_channel) * 0x14u));
            this->p_DMA->rx_irqn = static_cast<IRQn_Type>(DMA1_Channel1_IRQn + static_cast<std::uint32_t>(a_channel));

            hkm_assert(false == bit_flag::is(this->p_DMA->p_rx_channel_registers->CCR, DMA_CCR_EN));

            reinterpret_cast<DMAMUX_Channel_TypeDef*>(DMAMUX1_Channel0_BASE +
                                                      (static_cast<std::uint32_t>(a_channel) * 4))
                ->CCR = 0x10u;
        }
        break;

        case 0x1u: {
            this->p_DMA->p_rx_channel_registers = reinterpret_cast<DMA_Channel_TypeDef*>(
                DMA2_Channel1_BASE + (static_cast<std::uint32_t>(a_channel) * 0x14u));
            this->p_DMA->rx_irqn = static_cast<IRQn_Type>(DMA2_Channel1_IRQn + static_cast<std::uint32_t>(a_channel));

            hkm_assert(false == bit_flag::is(this->p_DMA->p_rx_channel_registers->CCR, DMA_CCR_EN));

            reinterpret_cast<DMAMUX_Channel_TypeDef*>(DMAMUX1_Channel7_BASE +
                                                      (static_cast<std::uint32_t>(a_channel) * 4))
                ->CCR = 0x10u;
        }
        break;
    }

    bit_flag::set(&(this->p_DMA->p_USART_registers->CR3), USART_CR3_DMAR);

    this->p_DMA->rx_channel = a_channel;
}
void DMA<LPUART>::Receiver::disable()
{
    if (this->interrupt.is_enabled())
    {
        this->interrupt.disable();
    }

    this->p_DMA->p_rx_channel_registers->CCR = 0x0u;
    this->p_DMA->p_rx_channel_registers->CNDTR = 0x0u;
    this->p_DMA->p_rx_channel_registers->CPAR = 0x0u;
    this->p_DMA->p_rx_channel_registers->CMAR = 0x0u;

    bit_flag::clear(
        &(reinterpret_cast<DMAMUX_Channel_TypeDef*>(
              DMAMUX1_Channel7_BASE + (static_cast<std::uint32_t>(this->p_DMA->p_rx_channel_registers->CCR) * 4))
              ->CCR),
        0x3fu);

    bit_flag::clear(&(this->p_DMA->p_USART_registers->CR3), USART_CR3_DMAR);
}

DMA<>::Result DMA<LPUART>::Receiver::Polling::receive(DMA<>::Priority a_priority,
                                                      DMA<>::Mode a_mode,
                                                      Not_null<void*> a_p_buffer,
                                                      std::uint16_t a_buffer_size_in_words)
{
    ::receive(this->p_DMA->p_USART_registers,
              this->p_DMA->p_rx_channel_registers,
              static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                  (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) |
                  (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                       (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                       0x0u),
              a_p_buffer,
              a_buffer_size_in_words);

    wait_until::any_bit_is_set(
        this->p_DMA->p_DMA_registers->ISR,
        (0x1u << (DMA_ISR_TCIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->rx_channel) * 4u)))) |
            (0x1u << (DMA_ISR_TEIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->rx_channel) * 4u)))));

    bit_flag::clear(&(this->p_DMA->p_rx_channel_registers->CCR), DMA_CCR_EN);

    return { get_Event_flag_and_clear(
                 this->p_DMA->p_DMA_registers->ISR, &(this->p_DMA->p_DMA_registers->IFCR), this->p_DMA->rx_channel),
             a_buffer_size_in_words - this->p_DMA->p_rx_channel_registers->CNDTR };
}
DMA<>::Result DMA<peripherals::USART>::Receiver::Polling::wait_for_data(bool wait_until_channel_disabled,
                                                                        std::size_t a_buffer_size_in_words,
                                                                        Milliseconds a_timeout)
{
    if (true == wait_until_channel_disabled)
    {
        bool is_cleared =
            wait_until::any_bit_is_cleared(this->p_DMA->p_rx_channel_registers->CCR, DMA_CCR_EN, a_timeout);
        if (false == is_cleared)
        {
            bit_flag::clear(&(this->p_DMA->p_rx_channel_registers->CCR), DMA_CCR_EN);
        }
    }
    else
    {
        wait_until::any_bit_is_set(
            this->p_DMA->p_DMA_registers->ISR,
            (0x1u << (DMA_ISR_TCIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->rx_channel) * 4u)))) |
                (0x1u << (DMA_ISR_TEIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->rx_channel) * 4u)))),
            a_timeout);

        bit_flag::clear(&(this->p_DMA->p_rx_channel_registers->CCR), DMA_CCR_EN);
    }

    return { get_Event_flag_and_clear(
                 this->p_DMA->p_DMA_registers->ISR, &(this->p_DMA->p_DMA_registers->IFCR), this->p_DMA->rx_channel),
             a_buffer_size_in_words - this->p_DMA->p_rx_channel_registers->CNDTR };
}
DMA<>::Result DMA<LPUART>::Receiver::Polling::receive(DMA<>::Priority a_priority,
                                                      DMA<>::Mode a_mode,
                                                      Not_null<void*> a_p_buffer,
                                                      std::uint16_t a_buffer_size_in_words,
                                                      Milliseconds a_timeout)
{
    std::uint64_t start = tick_counter<Milliseconds>::get();

    ::receive(this->p_DMA->p_USART_registers,
              this->p_DMA->p_rx_channel_registers,
              static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                  (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) |
                  (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                       (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                       0x0u),
              a_p_buffer,
              a_buffer_size_in_words);

    wait_until::any_bit_is_set(
        this->p_DMA->p_DMA_registers->ISR,
        (0x1u << (DMA_ISR_TCIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->rx_channel) * 4u)))) |
            (0x1u << (DMA_ISR_TEIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->rx_channel) * 4u)))),
        a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    bit_flag::clear(&(this->p_DMA->p_rx_channel_registers->CCR), DMA_CCR_EN);

    return { get_Event_flag_and_clear(
                 this->p_DMA->p_DMA_registers->ISR, &(this->p_DMA->p_DMA_registers->IFCR), this->p_DMA->rx_channel),
             a_buffer_size_in_words - this->p_DMA->p_rx_channel_registers->CNDTR };
}

void DMA<LPUART>::Receiver::Interrupt::enable(const IRQ_config& a_irq_config,
                                              const DMA<>::Callback& a_callback,
                                              DMA<>::Event_flag a_flag)
{
    Scoped_guard<nvic> nvic_guard;

    this->p_DMA->rx_callback = a_callback;
    this->set_context();
    bit_flag::set(&(this->p_DMA->p_rx_channel_registers->CCR), get_interrupt_enable_flags(a_flag));

    NVIC_SetPriority(
        this->p_DMA->rx_irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->p_DMA->rx_irqn);
}
void DMA<LPUART>::Receiver::Interrupt::disable()
{
    Scoped_guard<nvic> nvic_guard;

    bit_flag::clear(&(this->p_DMA->p_rx_channel_registers->CCR), DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
    this->clear_context();
    this->p_DMA->rx_callback = { .function = nullptr, .p_user_data = nullptr };

    NVIC_DisableIRQ(this->p_DMA->rx_irqn);
}
bool DMA<LPUART>::Receiver::Interrupt::is_enabled() const
{
    return 1u == NVIC_GetEnableIRQ(this->p_DMA->rx_irqn);
}
void DMA<LPUART>::Receiver::Interrupt::start(DMA<>::Priority a_priority,
                                             DMA<>::Mode a_mode,
                                             Not_null<volatile void*> a_p_buffer,
                                             std::uint16_t a_buffer_size_in_words)
{
    bit_flag::set(&(this->p_DMA->p_DMA_registers->IFCR),
                  0x1u << (DMA_IFCR_CGIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->rx_channel) * 4u))));

    ::receive(this->p_DMA->p_USART_registers,
              this->p_DMA->p_rx_channel_registers,
              static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                  (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) |
                  (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                       (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                       0x0u),
              a_p_buffer,
              a_buffer_size_in_words);
}
void DMA<LPUART>::Receiver::Interrupt::stop()
{
    bit_flag::clear(&(this->p_DMA->p_rx_channel_registers->CCR), DMA_CCR_EN);
}

void DMA<LPUART>::Transmitter::enable(DMA<>::Channel a_channel)
{
    switch (this->p_DMA->idx)
    {
        case 0x0u: {
            this->p_DMA->p_tx_channel_registers = reinterpret_cast<DMA_Channel_TypeDef*>(
                DMA1_Channel1_BASE + (static_cast<std::uint32_t>(a_channel) * 0x14u));
            this->p_DMA->tx_irqn = static_cast<IRQn_Type>(DMA1_Channel1_IRQn + static_cast<std::uint32_t>(a_channel));

            hkm_assert(false == bit_flag::is(this->p_DMA->p_tx_channel_registers->CCR, DMA_CCR_EN));

            reinterpret_cast<DMAMUX_Channel_TypeDef*>(DMAMUX1_Channel0_BASE +
                                                      (static_cast<std::uint32_t>(a_channel) * 4))
                ->CCR = 0x11;
        }
        break;

        case 0x1u: {
            this->p_DMA->p_tx_channel_registers = reinterpret_cast<DMA_Channel_TypeDef*>(
                DMA2_Channel1_BASE + (static_cast<std::uint32_t>(a_channel) * 0x14u));
            this->p_DMA->tx_irqn = static_cast<IRQn_Type>(DMA2_Channel1_IRQn + static_cast<std::uint32_t>(a_channel));

            hkm_assert(false == bit_flag::is(this->p_DMA->p_tx_channel_registers->CCR, DMA_CCR_EN));

            reinterpret_cast<DMAMUX_Channel_TypeDef*>(DMAMUX1_Channel7_BASE +
                                                      (static_cast<std::uint32_t>(a_channel) * 4))
                ->CCR = 0x11;
        }
        break;
    }

    bit_flag::set(&(this->p_DMA->p_USART_registers->CR3), USART_CR3_DMAT);

    this->p_DMA->tx_channel = a_channel;
}
void DMA<LPUART>::Transmitter::disable()
{
    this->p_DMA->p_tx_channel_registers->CCR = 0x0u;
    this->p_DMA->p_tx_channel_registers->CNDTR = 0x0u;
    this->p_DMA->p_tx_channel_registers->CPAR = 0x0u;
    this->p_DMA->p_tx_channel_registers->CMAR = 0x0u;

    bit_flag::clear(
        &(reinterpret_cast<DMAMUX_Channel_TypeDef*>(
              DMAMUX1_Channel7_BASE + (static_cast<std::uint32_t>(this->p_DMA->p_tx_channel_registers->CCR) * 4))
              ->CCR),
        0x3fu);

    bit_flag::clear(&(this->p_DMA->p_USART_registers->CR3), USART_CR3_DMAT);
}

DMA<>::Result DMA<LPUART>::Transmitter::Polling::transmit(DMA<>::Priority a_priority,
                                                          DMA<>::Mode a_mode,
                                                          Not_null<const void*> a_p_buffer,
                                                          std::uint16_t a_buffer_size_in_words)
{
    ::transmit(this->p_DMA->p_USART_registers,
               this->p_DMA->p_tx_channel_registers,
               static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                   (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) | DMA_CCR_DIR |
                   (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                        (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                        0x0u),
               a_p_buffer,
               a_buffer_size_in_words);

    wait_until::any_bit_is_set(
        this->p_DMA->p_DMA_registers->ISR,
        (0x1u << (DMA_ISR_TCIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->tx_channel) * 4u)))) |
            (0x1u << (DMA_ISR_TEIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->tx_channel) * 4u)))));

    bit_flag::clear(&(this->p_DMA->p_tx_channel_registers->CCR), DMA_CCR_EN);

    return { get_Event_flag_and_clear(
                 this->p_DMA->p_DMA_registers->ISR, &(this->p_DMA->p_DMA_registers->IFCR), this->p_DMA->tx_channel),
             a_buffer_size_in_words - this->p_DMA->p_tx_channel_registers->CNDTR };
}
DMA<>::Result DMA<LPUART>::Transmitter::Polling::transmit(DMA<>::Priority a_priority,
                                                          DMA<>::Mode a_mode,
                                                          Not_null<const void*> a_p_buffer,
                                                          std::uint16_t a_buffer_size_in_words,
                                                          Milliseconds a_timeout)
{
    std::uint64_t start = tick_counter<Milliseconds>::get();

    ::transmit(this->p_DMA->p_USART_registers,
               this->p_DMA->p_tx_channel_registers,
               static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                   (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) | DMA_CCR_DIR |
                   (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                        (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                        0x0u),
               a_p_buffer,
               a_buffer_size_in_words);

    wait_until::any_bit_is_set(this->p_DMA->p_DMA_registers->ISR,
                               (DMA_ISR_TCIF1 + static_cast<std::uint32_t>(this->p_DMA->tx_channel)) |
                                   (DMA_ISR_TEIF1 + static_cast<std::uint32_t>(this->p_DMA->tx_channel)),
                               a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    bit_flag::clear(&(this->p_DMA->p_tx_channel_registers->CCR), DMA_CCR_EN);

    return { get_Event_flag_and_clear(
                 this->p_DMA->p_DMA_registers->ISR, &(this->p_DMA->p_DMA_registers->IFCR), this->p_DMA->tx_channel),
             a_buffer_size_in_words - this->p_DMA->p_tx_channel_registers->CNDTR };
}

void DMA<LPUART>::Transmitter::Interrupt::enable(const IRQ_config& a_irq_config,
                                                 const DMA<>::Callback& a_callback,
                                                 DMA<>::Event_flag a_flag)
{
    Scoped_guard<nvic> nvic_guard;

    this->p_DMA->tx_callback = a_callback;
    this->set_context();
    bit_flag::set(&(this->p_DMA->p_tx_channel_registers->CCR), get_interrupt_enable_flags(a_flag));

    NVIC_SetPriority(
        this->p_DMA->tx_irqn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), a_irq_config.preempt_priority, a_irq_config.sub_priority));
    NVIC_EnableIRQ(this->p_DMA->tx_irqn);
}
void DMA<LPUART>::Transmitter::Interrupt::disable()
{
    Scoped_guard<nvic> nvic_guard;

    bit_flag::clear(&(this->p_DMA->p_tx_channel_registers->CCR), DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
    this->clear_context();
    this->p_DMA->rx_callback = { .function = nullptr, .p_user_data = nullptr };

    NVIC_DisableIRQ(this->p_DMA->tx_irqn);
}
bool DMA<LPUART>::Transmitter::Interrupt::is_enabled() const
{
    return 1u == NVIC_GetEnableIRQ(this->p_DMA->tx_irqn);
}
void DMA<LPUART>::Transmitter::Interrupt::start(DMA<>::Priority a_priority,
                                                DMA<>::Mode a_mode,
                                                Not_null<volatile const void*> a_p_buffer,
                                                std::uint16_t a_buffer_size_in_words)
{
    bit_flag::set(&(this->p_DMA->p_DMA_registers->IFCR),
                  0x1u << (DMA_IFCR_CGIF1_Pos + ((static_cast<std::uint32_t>(this->p_DMA->tx_channel) * 4u))));

    ::transmit(this->p_DMA->p_USART_registers,
               this->p_DMA->p_tx_channel_registers,
               static_cast<std::uint32_t>(a_priority) | static_cast<std::uint32_t>(a_mode) |
                   (a_buffer_size_in_words > 1 ? DMA_CCR_MINC : 0x0u) | DMA_CCR_DIR |
                   (USART_CR1_M0 == bit_flag::get(this->p_DMA->p_USART_registers->CR1, USART_CR1_M0 | USART_CR1_M1) ?
                        (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0) :
                        0x0u),
               a_p_buffer,
               a_buffer_size_in_words);
}
void DMA<LPUART>::Transmitter::Interrupt::stop()
{
    bit_flag::clear(&(this->p_DMA->p_tx_channel_registers->CCR), DMA_CCR_EN);
}
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu
