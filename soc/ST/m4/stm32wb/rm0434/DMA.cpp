/**/

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/USART/DMA.hpp>

// xmcu
#include <xmcu/bit.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb;

DMA<>::Callback* p_callbacks[2][7] = { nullptr };

DMA<>::Event_flag get_Event_flag_and_clear(DMA_TypeDef* a_p_DMA_registers,
                                           DMA_Channel_TypeDef* a_p_channel_registers,
                                           std::size_t a_channel_idx)
{
    DMA<>::Event_flag ret   = DMA<>::Event_flag::none;
    const std::uint32_t isr = a_p_DMA_registers->ISR;

    if (true == bit::is(isr, DMA_ISR_TCIF1_Pos + a_channel_idx))
    {
        bit::set(&(a_p_DMA_registers->IFCR), DMA_IFCR_CTCIF1_Pos + a_channel_idx);

        if (true == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_TCIE))
        {
            ret |= DMA<>::Event_flag::full_transfer_complete;
        }
    }

    if (true == bit::is(isr, DMA_ISR_HTIF1_Pos + a_channel_idx))
    {
        bit::set(&(a_p_DMA_registers->IFCR), DMA_IFCR_CHTIF1_Pos + a_channel_idx);

        if (true == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_HTIE))
        {
            ret |= DMA<>::Event_flag::half_transfer_complete;
        }
    }

    if (true == bit::is(isr, DMA_ISR_TEIF1_Pos + a_channel_idx))
    {
        bit::set(&(a_p_DMA_registers->IFCR), DMA_IFCR_CTEIF1_Pos + a_channel_idx);

        if (true == bit_flag::is(a_p_channel_registers->CCR, DMA_CCR_TEIE))
        {
            ret |= DMA<>::Event_flag::transfer_error;
        }
    }

    return ret;
}
} // namespace

extern "C" {
void DMA1_Channel1_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 0;
    constexpr std::uint32_t channel_id = 0;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA1_Channel2_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 0;
    constexpr std::uint32_t channel_id = 1;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA1_Channel3_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 0;
    constexpr std::uint32_t channel_id = 2;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA1_Channel4_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 0;
    constexpr std::uint32_t channel_id = 3;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA1_Channel5_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 0;
    constexpr std::uint32_t channel_id = 4;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA1_Channel6_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 0;
    constexpr std::uint32_t channel_id = 5;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA1_Channel7_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 0;
    constexpr std::uint32_t channel_id = 6;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}

void DMA2_Channel1_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 1;
    constexpr std::uint32_t channel_id = 0;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA2_Channel2_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 1;
    constexpr std::uint32_t channel_id = 1;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA2_Channel3_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 1;
    constexpr std::uint32_t channel_id = 2;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA2_Channel4_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 1;
    constexpr std::uint32_t channel_id = 3;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA2_Channel5_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 1;
    constexpr std::uint32_t channel_id = 4;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA2_Channel6_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 1;
    constexpr std::uint32_t channel_id = 5;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
void DMA2_Channel7_IRQHandler()
{
    constexpr std::uint32_t DMA_id     = 1;
    constexpr std::uint32_t channel_id = 6;

    hkm_assert(nullptr != p_callbacks[DMA_id][channel_id]);

    p_callbacks[DMA_id][channel_id]->function(get_Event_flag_and_clear(DMA1, DMA1_Channel1, channel_id),
                                              p_callbacks[DMA_id][channel_id]->p_user_data);
}
}

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
using namespace xmcu::soc::m4::stm32wb::peripherals;

void DMA<USART>::Receiver::Interrupt::set_context()
{
    p_callbacks[this->p_DMA->idx][static_cast<std::uint32_t>(this->p_DMA->rx_channel)] = &(this->p_DMA->rx_callback);
}
void DMA<USART>::Receiver::Interrupt::clear_context()
{
    p_callbacks[this->p_DMA->idx][static_cast<std::uint32_t>(this->p_DMA->rx_channel)] = nullptr;
}
void DMA<USART>::Transmitter::Interrupt::set_context()
{
    p_callbacks[this->p_DMA->idx][static_cast<std::uint32_t>(this->p_DMA->tx_channel)] = &(this->p_DMA->tx_callback);
}
void DMA<USART>::Transmitter::Interrupt::clear_context()
{
    p_callbacks[this->p_DMA->idx][static_cast<std::uint32_t>(this->p_DMA->tx_channel)] = nullptr;
}

void DMA<LPUART>::Receiver::Interrupt::set_context()
{
    p_callbacks[this->p_DMA->idx][static_cast<std::uint32_t>(this->p_DMA->rx_channel)] = &(this->p_DMA->rx_callback);
}
void DMA<LPUART>::Receiver::Interrupt::clear_context()
{
    p_callbacks[this->p_DMA->idx][static_cast<std::uint32_t>(this->p_DMA->rx_channel)] = nullptr;
}
void DMA<LPUART>::Transmitter::Interrupt::set_context()
{
    p_callbacks[this->p_DMA->idx][static_cast<std::uint32_t>(this->p_DMA->tx_channel)] = &(this->p_DMA->tx_callback);
}
void DMA<LPUART>::Transmitter::Interrupt::clear_context()
{
    p_callbacks[this->p_DMA->idx][static_cast<std::uint32_t>(this->p_DMA->tx_channel)] = nullptr;
}
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu
