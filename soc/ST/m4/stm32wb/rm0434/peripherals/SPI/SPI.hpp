#pragma once

/**/

// std
#include <cstdint>

// external
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Non_copyable.hpp>
#include <xmcu/Not_null.hpp>
#include <xmcu/bit.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/GPIO/GPIO.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/rcc.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hsi16.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/mcu/mcu.hpp>
#include <xmcu/soc/peripheral.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {

class SPI : private xmcu::Non_copyable
{
public:
    enum class Event_flag : std::uint32_t
    {
        none = 0x0u,
    };

    class Polling
    {
    public:
        struct Result
        {
            Event_flag event = various::get_enum_incorrect_value<Event_flag>();
            std::size_t data_length_in_words = 0;
        };

        Result transmit(Not_null<const uint8_t*> a_p_data, std::size_t a_data_size_in_words);
        Result receive(Not_null<uint8_t*> a_p_data, std::size_t a_data_size_in_words);

    private:
        SPI* p_SPI = nullptr;
        friend class SPI;
    };

    SPI(SPI&&) = default;
    SPI& operator=(SPI&&) = default;

    SPI()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
        this->polling.p_SPI = nullptr;
    }
    ~SPI()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable();

    void disable();

    bool is_enabled() const
    {
        return bit::is_any(this->p_registers->CR1, SPI_CR1_SPE_Pos);
    }

    Polling polling;

private:
    SPI(std::uint32_t a_idx, SPI_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , irqn(a_irqn)
    {
        this->polling.p_SPI = this;
    }

    std::uint32_t idx;
    SPI_TypeDef* p_registers;

    IRQn_Type irqn;

    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
};

} // namespace peripherals
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {

template<std::uint32_t id> class rcc<peripherals::SPI, id> : private xmcu::Non_constructible
{
public:
    template<typename Source_t> static void enable(bool a_enable_in_lp) = delete;
    static void disable() = delete;
};
template<> template<> void rcc<peripherals::SPI, 1u>::enable<sources::hsi16>(bool a_enable_in_lp);
template<> void rcc<peripherals::SPI, 1u>::disable();

template<> inline void
peripherals::GPIO::Alternate_function::enable<peripherals::SPI, 1>(Limited<std::uint32_t, 0, 15> a_id,
                                                                   const Enable_config& a_config,
                                                                   Pin* a_p_pin)
{
    // Check if GPIO is eligible for RX/TX pin
    std::uint8_t alternate_function_index;
#if defined(STM32WB35xx)
    // DS11929 table 19
    if ((0u == this->p_port->idx && (1u == a_id || 4u == a_id || 5u == a_id || 6u == a_id || 7u == a_id ||
                                     11u == a_id || 12u == a_id || 15u == a_id)) ||
        (1u == this->p_port->idx && (2u == a_id || 3u == a_id || 4u == a_id || 5u == a_id)))
    {
        alternate_function_index = 5u;
    }
    else
    {
        hkm_assert(false);
        while (true) continue;
    }
#else
#error "Undefined pin constraints for uC - add missing model refering to its *datasheet*"
#endif
    this->enable(a_id, a_config, alternate_function_index, a_p_pin);
}

} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
template<> class peripheral<m4::stm32wb::peripherals::SPI, 1u> : private xmcu::Non_constructible
{
public:
    static m4::stm32wb::peripherals::SPI create()
    {
        return m4::stm32wb::peripherals::SPI(0u, SPI1, IRQn_Type::SPI1_IRQn);
    }
};
} // namespace soc
} // namespace xmcu
