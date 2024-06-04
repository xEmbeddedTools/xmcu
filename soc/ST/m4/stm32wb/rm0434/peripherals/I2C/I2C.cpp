/**/


// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/I2C/I2C.hpp>

// xmcu
#include <xmcu/Frequency.hpp>
#include <xmcu/bit_flag.hpp>

namespace xmcu::soc::m4::stm32wb {
using namespace xmcu;

enum class Clk_Sel
{
    PCLK   = 0b00,
    SYSCLK = 0b01,
    HSI16  = 0b10
};

static std::uint32_t get_freq(Clk_Sel a_sel)
{
    switch (a_sel)
    {
        case Clk_Sel::PCLK:
            return rcc<system::mcu<1u>>::pclk<1u>::get_frequency_Hz();
        case Clk_Sel::SYSCLK:
            return rcc<system::mcu<1u>>::get_system_clock_frequency_Hz();
        case Clk_Sel::HSI16:
            return sources::hsi16::get_frequency_Hz();
        default: {
            [[maybe_unused]] bool is_valid_clock_selection = false;
            hkm_assert(is_valid_clock_selection);
            return 0;
        }
    }
}

template<std::uint32_t id> void enable_rcc_impl(Clk_Sel a_sel, bool a_enable_in_lp) = delete;

template<> void enable_rcc_impl<0x01u>(Clk_Sel a_sel, bool a_enable_in_lp)
{
    bit_flag::set(&RCC->CCIPR, RCC_CCIPR_I2C1SEL, static_cast<std::uint32_t>(a_sel) << RCC_CCIPR_I2C1SEL_Pos);
    bit_flag::set(&RCC->APB1ENR1, RCC_APB1ENR1_I2C1EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C1SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C1SMEN);
    }
}

template<> void enable_rcc_impl<0x03u>(Clk_Sel a_sel, bool a_enable_in_lp)
{
    bit_flag::set(&RCC->CCIPR, RCC_CCIPR_I2C3SEL, static_cast<std::uint32_t>(a_sel) << RCC_CCIPR_I2C3SEL_Pos);
    bit_flag::set(&RCC->APB1ENR1, RCC_APB1ENR1_I2C3EN);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C3SMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_I2C3SMEN);
    }
}

template<> template<> void rcc<peripherals::I2C, 1u>::enable<rcc<system::mcu<1u>>::pclk<1u>>(bool a_enable_in_lp)
{
    return enable_rcc_impl<1u>(Clk_Sel::PCLK, a_enable_in_lp);
}
template<> template<> void rcc<peripherals::I2C, 1u>::enable<rcc<system::mcu<1u>>>(bool a_enable_in_lp)
{
    return enable_rcc_impl<1u>(Clk_Sel::SYSCLK, a_enable_in_lp);
}
template<> template<> void rcc<peripherals::I2C, 1u>::enable<sources::hsi16>(bool a_enable_in_lp)
{
    return enable_rcc_impl<1u>(Clk_Sel::HSI16, a_enable_in_lp);
}
template<> void rcc<peripherals::I2C, 1u>::disable()
{
    bit_flag::clear(&RCC->CCIPR, RCC_CCIPR_I2C1SEL);
    bit_flag::clear(&RCC->APB1ENR1, RCC_APB1ENR1_I2C1EN);

    bit_flag::clear(&RCC->APB1SMENR1, RCC_APB1SMENR1_I2C1SMEN);
}
template<> std::uint32_t rcc<peripherals::I2C, 1u>::get_frequency_Hz()
{
    std::uint32_t reg_flags = bit_flag::get(RCC->CCIPR, RCC_CCIPR_I2C1SEL) >> RCC_CCIPR_I2C1SEL_Pos;
    return get_freq(static_cast<Clk_Sel>(reg_flags));
}

template<> template<> void rcc<peripherals::I2C, 3u>::enable<rcc<system::mcu<1u>>::pclk<1u>>(bool a_enable_in_lp)
{
    return enable_rcc_impl<3u>(Clk_Sel::PCLK, a_enable_in_lp);
}
template<> template<> void rcc<peripherals::I2C, 3u>::enable<rcc<system::mcu<1u>>>(bool a_enable_in_lp)
{
    return enable_rcc_impl<3u>(Clk_Sel::SYSCLK, a_enable_in_lp);
}
template<> template<> void rcc<peripherals::I2C, 3u>::enable<sources::hsi16>(bool a_enable_in_lp)
{
    return enable_rcc_impl<3u>(Clk_Sel::HSI16, a_enable_in_lp);
}
template<> void rcc<peripherals::I2C, 3u>::disable()
{
    bit_flag::clear(&RCC->CCIPR, RCC_CCIPR_I2C3SEL);
    bit_flag::clear(&RCC->APB1ENR1, RCC_APB1ENR1_I2C3EN);

    bit_flag::clear(&RCC->APB1SMENR1, RCC_APB1SMENR1_I2C3SMEN);
}
template<> std::uint32_t rcc<peripherals::I2C, 3u>::get_frequency_Hz()
{
    std::uint32_t reg_flags = bit_flag::get(RCC->CCIPR, RCC_CCIPR_I2C3SEL) >> RCC_CCIPR_I2C3SEL_Pos;
    return get_freq(static_cast<Clk_Sel>(reg_flags));
}

} // namespace xmcu::soc::m4::stm32wb

namespace xmcu::soc::m4::stm32wb::peripherals {
class Transfer_config
{
public:
    Transfer_config(I2C_TypeDef* a_p_registers, std::size_t a_size);
    ~Transfer_config();
    std::size_t set_size(bool);
    void configure(std::uint32_t);
    void start_read();
    void start_write();

private:
    I2C_TypeDef* p_registers;
    std::size_t remaining_size;
    bool inline is_last_part() { return 256 > this->remaining_size; }
};

Transfer_config::Transfer_config(I2C_TypeDef* a_p_registers, std::size_t a_size)
    : p_registers { a_p_registers }
    , remaining_size { a_size }
{
    if (false == this->is_last_part())
    {
        bit_flag::set(&this->p_registers->CR2, I2C_CR2_RELOAD);
    }
}

Transfer_config::~Transfer_config()
{
    if (false == bit_flag::is(this->p_registers->CR2, I2C_CR2_AUTOEND))
    {
        // using re-start - dont need clear stop flag, and CR2 register.
        while (false == bit_flag::is(this->p_registers->ISR, I2C_ISR_TC)) continue;
        return;
    }
    if (bit_flag::is(this->p_registers->ISR, I2C_ISR_BUSY))
    {
        while (false == bit_flag::is(this->p_registers->ISR, I2C_ISR_STOPF)) continue;
        bit_flag::set(&this->p_registers->ICR, I2C_ICR_STOPCF);
        this->p_registers->CR2 = 0;
    }
}

void Transfer_config::configure(std::uint32_t a_adr)
{
    hkm_assert(a_adr & 0xFE); // 7 bit shitfed
    // here we have difference between 7/10 bit
    // 7 bit witten from bit 1
    // 10 bit witten from bit 0

    // Configure slave address
    bit_flag::set(&this->p_registers->CR2, I2C_CR2_SADD, a_adr << I2C_CR2_SADD_Pos);
}

std::size_t Transfer_config::set_size(bool a_end)
{
    // NBYTES = N
    std::size_t transfer_size = this->is_last_part() ? this->remaining_size : 0xFF;
    // check size and need of reload set
    bit_flag::set(&this->p_registers->CR2, I2C_CR2_NBYTES, transfer_size << I2C_CR2_NBYTES_Pos);
    if (this->is_last_part())
    {
        bit_flag::clear(&this->p_registers->CR2, I2C_CR2_RELOAD);
        // only for last part...
        bit_flag::set(&this->p_registers->CR2, I2C_CR2_AUTOEND, a_end << I2C_CR2_AUTOEND_Pos);
    }
    this->remaining_size -= transfer_size;

    return transfer_size;
}

void Transfer_config::start_read()
{
    bit_flag::set(&this->p_registers->CR2, I2C_CR2_RD_WRN);
    bit_flag::set(&this->p_registers->CR2, I2C_CR2_START);
}
void Transfer_config::start_write()
{
    bit_flag::clear(&this->p_registers->CR2, I2C_CR2_RD_WRN);
    bit_flag::set(&this->p_registers->CR2, I2C_CR2_START);
}

I2C::Polling::Polling(I2C_TypeDef* a_p_registers)
    : p_registers { a_p_registers }
{
}

bool I2C::enable()
{
    std::uint32_t timing;
    bit_flag::clear(&this->p_registers->CR1, I2C_CR1_PE);
    __ISB();
    // bit_flag::set(&this->p_registers->CR1, I2C_CR1_ANFOFF); // analog filter

    switch (get_clk())
    {
        case 32_MHz:
            // temporary from cube:
            timing = 0x2010091A;
            break;
        default:
            [[maybe_unused]] bool unsuported_clock_frequency = false;
            hkm_assert(unsuported_clock_frequency);
            return false;
    }
    this->p_registers->TIMINGR = timing;
    // i2c must be configured first...
    bit_flag::set(&this->p_registers->CR1, I2C_CR1_PE);
    return true;
}

I2C::Transfer_result
I2C::Polling::transmit(I2C::Address a_adr, Not_null<const void*> a_p_data, std::size_t a_size, bool a_is_auto_end)
{
    const std::uint8_t* p_data = reinterpret_cast<const std::uint8_t*>(a_p_data.get());
    Transfer_config transfer { this->p_registers, a_size };
    std::size_t transfer_size = transfer.set_size(a_is_auto_end);
    transfer.configure(a_adr);
    transfer.start_write();

    while (true)
    {
        std::size_t i = 0;
        while (i < transfer_size)
        {
            if (true == bit_flag::is(this->p_registers->ISR, I2C_ISR_NACKF))
            {
                return Transfer_result::nack;
            }
            if (true == bit_flag::is(this->p_registers->ISR, I2C_ISR_ARLO))
            {
                this->p_registers->ICR = I2C_ICR_ARLOCF;
                return I2C::Transfer_result::arbitration_lost;
            }
            if (false == bit_flag::is(this->p_registers->ISR, I2C_ISR_TXIS))
            {
                continue;
            }
            // write
            this->p_registers->TXDR = *(p_data++);
            ++i;
        }
        if (true == bit_flag::is(this->p_registers->ISR, I2C_ISR_TC))
        { // This flag is set by hardware when RELOAD = 0, AUTOEND = 0 and NBYTES data have been transferred
            return Transfer_result::ok;
        }
        if (true == bit_flag::is(this->p_registers->ISR, I2C_ISR_TCR))
        { // This flag is set by hardware when RELOAD = 1 and NBYTES data have been transferred
            transfer_size = transfer.set_size(a_is_auto_end);
        }
        else
        {
            return Transfer_result::ok;
        }
    }
}

I2C::Transfer_result
I2C::Polling::receive(Address a_adr, Not_null<void*> a_p_data, std::size_t a_size, bool a_is_auto_end)
{
    std::uint8_t* p_data = reinterpret_cast<std::uint8_t*>(a_p_data.get());
    Transfer_config transfer { this->p_registers, a_size };
    std::size_t transfer_size = transfer.set_size(a_is_auto_end);
    transfer.configure(a_adr);
    transfer.start_read();
    // be careful when debuging with memory preview because reading RXDR clear RXNE flag
    // so (1) when ICD read RXDR before CPU - program lose data...
    // so (2) don't stop program between start and read RXDR

    while (true)
    {
        std::size_t i = 0;
        while (i < transfer_size)
        {
            if (false == bit_flag::is(this->p_registers->ISR, I2C_ISR_RXNE))
            {
                continue;
            }
            // read
            *(p_data++) = this->p_registers->RXDR;
            ++i;
        }
        if (true == bit_flag::is(this->p_registers->ISR, I2C_ISR_TC))
        { // This flag is set by hardware when RELOAD = 0, AUTOEND = 0 and NBYTES data have been transferred
            return Transfer_result::ok;
            // next start will be repeated start so we need new chunk of data, from another call
        }
        if (true == bit_flag::is(this->p_registers->ISR, I2C_ISR_TCR))
        { // This flag is set by hardware when RELOAD = 1 and NBYTES data have been transferred
            transfer_size = transfer.set_size(a_is_auto_end);
        }
        else
        {
            return Transfer_result::ok;
        }
    }

    return Transfer_result::nack;
}

} // namespace xmcu::soc::m4::stm32wb::peripherals
