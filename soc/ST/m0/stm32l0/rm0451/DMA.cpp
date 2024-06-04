/**/

// xmcu
#include <xmcu/bit.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/DMA.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/peripherals/USART/DMA.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::m0::stm32l0::rm0451;

DMA<>::Callback* p_callbacks[DMA<>::channel_count] = { nullptr };

DMA<>::Event_flag get_Event_flag_and_clear(DMA_TypeDef* a_p_DMA_registers,
                                           DMA_Channel_TypeDef* a_p_channel_registers,
                                           std::size_t a_channel_idx)
{
    DMA<>::Event_flag ret        = DMA<>::Event_flag::none;
    const std::uint32_t isr      = a_p_DMA_registers->ISR;
    std::uint32_t channel_offset = a_channel_idx * 4;

    if (true == bit::is(isr, DMA_ISR_TCIF1_Pos + channel_offset))
    {
        bit::set(&(a_p_DMA_registers->IFCR), DMA_IFCR_CTCIF1_Pos + channel_offset);

        if (true == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_TCIE))
        {
            ret |= DMA<>::Event_flag::full_transfer_complete;
        }
    }

    if (true == bit::is(isr, DMA_ISR_HTIF1_Pos + channel_offset))
    {
        bit::set(&(a_p_DMA_registers->IFCR), DMA_IFCR_CHTIF1_Pos + channel_offset);

        if (true == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_HTIE))
        {
            ret |= DMA<>::Event_flag::half_transfer_complete;
        }
    }

    if (true == bit::is(isr, DMA_ISR_TEIF1_Pos + channel_offset))
    {
        bit::set(&(a_p_DMA_registers->IFCR), DMA_IFCR_CTEIF1_Pos + channel_offset);

        if (true == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_TEIE))
        {
            ret |= DMA<>::Event_flag::transfer_error;
        }
    }

    return ret;
}

void shared_dma_int_handler(std::uint32_t a_start, std::uint32_t a_end)
{
    for (std::uint32_t i = a_start; i <= a_end; ++i)
    {
        std::uint32_t channel_id                = i - 1;
        DMA_Channel_TypeDef* p_dma_channel_regs = ((DMA_Channel_TypeDef*)(DMA1_Channel1_BASE + 20 * channel_id));
        DMA<>::Event_flag event_flags           = get_Event_flag_and_clear(DMA1, p_dma_channel_regs, channel_id);
        if (event_flags != DMA<>::Event_flag::none)
        {
            hkm_assert(nullptr != p_callbacks[channel_id]);
            p_callbacks[channel_id]->function(event_flags, p_callbacks[channel_id]->p_user_data);
        }
    }
}

} // namespace

extern "C" {

void DMA1_Channel1_IRQHandler()
{
    constexpr std::uint32_t channel_id = 0;
    hkm_assert(nullptr != p_callbacks[channel_id]);
    p_callbacks[channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                      p_callbacks[channel_id]->p_user_data);
}

void DMA1_Channel2_3_IRQHandler()
{
    shared_dma_int_handler(2, 3);
}

#if STM32L010XX_DMA_CHANNEL_COUNT == 5
void DMA1_Channel4_5_IRQHandler()
{
    shared_dma_int_handler(4, 5);
}
#elif STM32L010XX_DMA_CHANNEL_COUNT == 7
void DMA1_Channel4_5_6_7_IRQHandler()
{
    shared_dma_int_handler(4, 7);
}
#else
#error "Unhandled STM32L010XX_DMA_CHANNEL_COUNT"
#endif
}

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
using namespace xmcu::soc::m0::stm32l0::rm0451::peripherals;

void DMA<USART>::Receiver::Interrupt::set_context()
{
    std::uint32_t dma_channel = static_cast<std::uint32_t>(this->p_DMA->rx_channel);
    hkm_assert(dma_channel < DMA<>::channel_count);
    p_callbacks[dma_channel] = &(this->p_DMA->rx_callback);
}

void DMA<USART>::Receiver::Interrupt::clear_context()
{
    std::uint32_t dma_channel = static_cast<std::uint32_t>(this->p_DMA->rx_channel);
    hkm_assert(dma_channel < DMA<>::channel_count);
    p_callbacks[dma_channel] = nullptr;
}

void DMA<USART>::Transmitter::Interrupt::set_context()
{
    std::uint32_t dma_channel = static_cast<std::uint32_t>(this->p_DMA->tx_channel);
    hkm_assert(dma_channel < DMA<>::channel_count);
    p_callbacks[static_cast<std::uint32_t>(this->p_DMA->tx_channel)] = &(this->p_DMA->tx_callback);
}

void DMA<USART>::Transmitter::Interrupt::clear_context()
{
    std::uint32_t dma_channel = static_cast<std::uint32_t>(this->p_DMA->tx_channel);
    hkm_assert(dma_channel < DMA<>::channel_count);
    p_callbacks[static_cast<std::uint32_t>(this->p_DMA->tx_channel)] = nullptr;
}

void DMA<LPUART>::Receiver::Interrupt::set_context()
{
    std::uint32_t dma_channel = static_cast<std::uint32_t>(this->p_DMA->rx_channel);
    hkm_assert(dma_channel < DMA<>::channel_count);
    p_callbacks[dma_channel] = &(this->p_DMA->rx_callback);
}

void DMA<LPUART>::Receiver::Interrupt::clear_context()
{
    std::uint32_t dma_channel = static_cast<std::uint32_t>(this->p_DMA->rx_channel);
    hkm_assert(dma_channel < DMA<>::channel_count);
    p_callbacks[dma_channel] = nullptr;
}

void DMA<LPUART>::Transmitter::Interrupt::set_context()
{
    std::uint32_t dma_channel = static_cast<std::uint32_t>(this->p_DMA->tx_channel);
    hkm_assert(dma_channel < DMA<>::channel_count);
    p_callbacks[static_cast<std::uint32_t>(this->p_DMA->tx_channel)] = &(this->p_DMA->tx_callback);
}

void DMA<LPUART>::Transmitter::Interrupt::clear_context()
{
    std::uint32_t dma_channel = static_cast<std::uint32_t>(this->p_DMA->tx_channel);
    hkm_assert(dma_channel < DMA<>::channel_count);
    p_callbacks[static_cast<std::uint32_t>(this->p_DMA->tx_channel)] = nullptr;
}

} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
