/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/crc/crc.hpp>

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/bit_flag.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::peripherals;

void crc_start(std::uint32_t a_init_value,
               std::uint32_t a_polynomial_length,
               crc<>::Polynomial_flag a_polynomial_config,
               crc<>::Input_data_format a_input_data_format,
               crc<>::Output_data_format_flag a_output_data_format)
{
    CRC->INIT = a_init_value;

    bit_flag::set(&(CRC->CR),
                  CRC_CR_REV_IN_0 | CRC_CR_REV_IN_1 | CRC_CR_REV_OUT | CRC_CR_POLYSIZE_0 | CRC_CR_POLYSIZE_1,
                  static_cast<std::uint32_t>(a_input_data_format) |
                      (crc<>::Output_data_format_flag::reversed_by_bit ==
                               (crc<>::Output_data_format_flag::reversed_by_bit & a_output_data_format) ?
                           CRC_CR_REV_OUT :
                           0x0u) |
                      a_polynomial_length);

    if (crc<>::Polynomial_flag::custom == (crc<>::Polynomial_flag::custom & a_polynomial_config))
    {
        CRC->POL = static_cast<std::uint32_t>(static_cast<std::uint64_t>(a_polynomial_config) >> 32u);
    }

    CRC->IDR = static_cast<std::uint32_t>(a_output_data_format);
}

void crc_update(const std::uint8_t* a_p_data, std::size_t a_data_legth_in_bytes)
{
    std::size_t full_words = a_data_legth_in_bytes / sizeof(std::uint32_t);

    // DR register's MSByte is processed first - make sure the byte order is correct
    for (std::size_t i = 0; i < full_words; i++)
    {
        CRC->DR = (static_cast<std::uint32_t>(a_p_data[i * 4u + 3u]) << 0u) |
                  (static_cast<std::uint32_t>(a_p_data[i * 4u + 2u]) << 8u) |
                  (static_cast<std::uint32_t>(a_p_data[i * 4u + 1u]) << 16u) |
                  (static_cast<std::uint32_t>(a_p_data[i * 4u + 0u]) << 24u);
    }

    switch (a_data_legth_in_bytes & 3u)
    {
        case 1u: {
            *(reinterpret_cast<volatile std::uint8_t*>(&(CRC->DR))) = a_p_data[a_data_legth_in_bytes - 1u];
        }
        break;

        case 2u: {
            *(reinterpret_cast<volatile std::uint16_t*>(&(CRC->DR))) =
                static_cast<std::uint16_t>(a_p_data[a_data_legth_in_bytes - 1u] << 0u) |
                static_cast<std::uint16_t>(a_p_data[a_data_legth_in_bytes - 2u] << 8u);
        }
        break;

        case 3u: {
            *(reinterpret_cast<volatile std::uint16_t*>(&(CRC->DR))) =
                static_cast<std::uint16_t>(a_p_data[a_data_legth_in_bytes - 2u] << 0u) |
                static_cast<std::uint16_t>(a_p_data[a_data_legth_in_bytes - 3u] << 8u);
            *(reinterpret_cast<volatile std::uint8_t*>(&(CRC->DR))) = a_p_data[a_data_legth_in_bytes - 1u];
        }
        break;
    }
}
} // namespace

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {

void crc<7u>::start(crc<>::Input_data_format a_input_data_format,
                    crc<>::Output_data_format_flag a_output_data_format,
                    crc<>::Polynomial_flag a_polynomial,
                    std::uint8_t a_init_value)
{
    crc_start(
        a_init_value, CRC_CR_POLYSIZE_0 | CRC_CR_POLYSIZE_1, a_polynomial, a_input_data_format, a_output_data_format);
}
void crc<7u>::update(const std::uint8_t* a_p_data, std::size_t a_data_legth_in_bytes)
{
    crc_update(a_p_data, a_data_legth_in_bytes);
}
std::uint32_t crc<7u>::get()
{
    if (crc<>::Output_data_format_flag::inverted ==
        (crc<>::Output_data_format_flag::inverted & static_cast<crc<>::Output_data_format_flag>(CRC->IDR)))
    {
        return ~(CRC->DR);
    }

    return CRC->DR;
}

void crc<8u>::start(crc<>::Input_data_format a_input_data_format,
                    crc<>::Output_data_format_flag a_output_data_format,
                    crc<>::Polynomial_flag a_polynomial,
                    std::uint8_t a_init_value)
{
    crc_start(a_init_value, CRC_CR_POLYSIZE_1, a_polynomial, a_input_data_format, a_output_data_format);
}
void crc<8u>::update(const std::uint8_t* a_p_data, std::size_t a_data_legth_in_bytes)
{
    crc_update(a_p_data, a_data_legth_in_bytes);
}
std::uint32_t crc<8u>::get()
{
    if (crc<>::Output_data_format_flag::inverted ==
        (crc<>::Output_data_format_flag::inverted & static_cast<crc<>::Output_data_format_flag>(CRC->IDR)))
    {
        return static_cast<std::uint8_t>(~(CRC->DR));
    }

    return static_cast<std::uint8_t>(CRC->DR);
}

void crc<16u>::start(crc<>::Input_data_format a_input_data_format,
                     crc<>::Output_data_format_flag a_output_data_format,
                     crc<>::Polynomial_flag a_polynomial,
                     std::uint16_t a_init_value)
{
    crc_start(a_init_value, CRC_CR_POLYSIZE_0, a_polynomial, a_input_data_format, a_output_data_format);
}
void crc<16u>::update(const std::uint8_t* a_p_data, std::size_t a_data_legth_in_bytes)
{
    crc_update(a_p_data, a_data_legth_in_bytes);
}
std::uint32_t crc<16u>::get()
{
    if (crc<>::Output_data_format_flag::inverted ==
        (crc<>::Output_data_format_flag::inverted & static_cast<crc<>::Output_data_format_flag>(CRC->IDR)))
    {
        return static_cast<std::uint16_t>(~(CRC->DR));
    }

    return static_cast<std::uint16_t>(CRC->DR);
}

void crc<32u>::start(crc<>::Input_data_format a_input_data_format,
                     crc<>::Output_data_format_flag a_output_data_format,
                     crc<>::Polynomial_flag a_polynomial,
                     std::uint32_t a_init_value)
{
    crc_start(a_init_value, 0x0u, a_polynomial, a_input_data_format, a_output_data_format);
}
void crc<32u>::update(const std::uint8_t* a_p_data, std::size_t a_data_legth_in_bytes)
{
    crc_update(a_p_data, a_data_legth_in_bytes);
}
std::uint32_t crc<32u>::get()
{
    if (crc<>::Output_data_format_flag::inverted ==
        (crc<>::Output_data_format_flag::inverted & static_cast<crc<>::Output_data_format_flag>(CRC->IDR)))
    {
        return ~(CRC->DR);
    }

    return CRC->DR;
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

void rcc<crc<>>::enable(bool a_enable_in_lp)
{
    bit_flag::set(&(RCC->AHB1ENR), RCC_AHB1ENR_CRCEN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->AHB1SMENR), RCC_AHB1SMENR_CRCSMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->AHB1SMENR), RCC_AHB1SMENR_CRCSMEN);
    }
}
void rcc<crc<>>::disable()
{
    bit_flag::clear(&(RCC->AHB1ENR), RCC_AHB1ENR_CRCEN);
}
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif
