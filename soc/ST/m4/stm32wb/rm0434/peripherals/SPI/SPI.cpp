/**/

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/SPI/SPI.hpp>

// xmcu
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::peripherals;
using namespace xmcu::soc::m4::stm32wb::utils;

void enable(SPI_TypeDef* a_p_registers)
{
    a_p_registers->CR1 = 0;
    a_p_registers->CR2 = 0;

    bit_flag::set(&a_p_registers->CR1,
                  SPI_CR1_BR | SPI_CR1_CPOL | SPI_CR1_CPHA,
                  SPI_CR1_BR_0 | SPI_CR1_BR_1);                               // f_pclk/2, SPI mode 0
    bit_flag::set(&a_p_registers->CR1, SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE, 0); // full duplex
    bit_flag::set(&a_p_registers->CR1, SPI_CR1_MSTR);                         // SPI Master mode
    bit_flag::set(&a_p_registers->CR1, SPI_CR1_LSBFIRST, 0);                  // MSBit first

    bit_flag::set(&a_p_registers->CR2, SPI_CR2_DS, SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2); // 8-bit transfers
    bit_flag::set(&a_p_registers->CR2, SPI_CR2_SSOE, 0);              // hw CS signal disabled - multi-CS or other pin
    bit_flag::set(&a_p_registers->CR2, SPI_CR2_FRXTH, SPI_CR2_FRXTH); // trigger RXNE on first byte written
    bit_flag::set(&a_p_registers->CR1, SPI_CR1_SSM | SPI_CR1_SSI); // Set to 1 when not using hardware CS to avoid MODF

    bit_flag::set(&a_p_registers->CR1, SPI_CR1_SPE);
}

void disable(SPI_TypeDef* a_p_registers)
{
    a_p_registers->CR1 = 0;
    a_p_registers->CR2 = 0;
}

template<typename data_t>
SPI::Polling::Result transmit(SPI_TypeDef* a_p_registers, const data_t* a_p_data, std::size_t a_data_size_in_words)
{
    // Prevent multibyte writes with 8-bit data becuase this way SPI FIFO can get populated with extra bytes.
    volatile data_t* p_data_reg = reinterpret_cast<volatile data_t*>(&a_p_registers->DR);

    std::uint16_t words_sent = 0;
    while (words_sent < a_data_size_in_words)
    {
        *p_data_reg = *(a_p_data++);

        wait_until::all_bits_are_set(a_p_registers->SR, SPI_SR_TXE);
        ++words_sent;
    }
    wait_until::all_bits_are_cleared(a_p_registers->SR, SPI_SR_BSY);

    return SPI::Polling::Result { .event = SPI::Event_flag::none, .data_length_in_words = words_sent };
}

template<typename data_t>
SPI::Polling::Result receive(SPI_TypeDef* a_p_registers, data_t* a_p_data, std::size_t a_data_size_in_words)
{
    // Prevent multibyte writes with 8-bit data becuase this way SPI FIFO can get populated with extra bytes.
    volatile data_t* p_data_reg = reinterpret_cast<volatile data_t*>(&a_p_registers->DR);

    // read all pending words from FIFO
    while (0 != bit_flag::get(a_p_registers->SR, SPI_SR_FRLVL))
    {
        *a_p_data = *p_data_reg;
    }

    std::uint16_t words_received = 0;
    while (words_received < a_data_size_in_words)
    {
        *p_data_reg = 0; // fill MOSI with anything to receive via MISO
        wait_until::all_bits_are_set(a_p_registers->SR, SPI_SR_RXNE);
        *(a_p_data++) = *p_data_reg;
        ++words_received;
    }
    wait_until::all_bits_are_cleared(a_p_registers->SR, SPI_SR_BSY);

    return SPI::Polling::Result { .event = SPI::Event_flag::none, .data_length_in_words = words_received };
}

} // namespace

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {

void SPI::enable()
{
    ::enable(this->p_registers);
}

void SPI::disable()
{
    ::disable(this->p_registers);
}

SPI::Polling::Result SPI::Polling::transmit(Not_null<const std::uint8_t*> a_p_data,
                                            std::size_t a_data_size_in_words)
{
    return ::transmit<std::uint8_t>(this->p_SPI->p_registers, a_p_data, a_data_size_in_words);
}

SPI::Polling::Result SPI::Polling::receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words)
{
    return ::receive<std::uint8_t>(this->p_SPI->p_registers, a_p_data, a_data_size_in_words);
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
using namespace xmcu::soc::m4::stm32wb::sources;

template<> template<> void rcc<peripherals::SPI, 1u>::enable<hsi16>(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB2SMENR), RCC_APB2SMENR_SPI1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB2SMENR), RCC_APB2SMENR_SPI1SMEN);
    }
}

template<> void rcc<peripherals::SPI, 1>::disable()
{
    bit_flag::clear(&(RCC->APB2ENR), RCC_APB2ENR_SPI1EN);
}

} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu
