#pragma once

/**/

// std
#include <array>
#include <cstddef>

// external
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Non_copyable.hpp>
#include <xmcu/Not_null.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/GPIO/GPIO.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/rcc.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/mcu/mcu.hpp>
#include <xmcu/soc/peripheral.hpp>

namespace xmcu::soc::m4::stm32wb::peripherals {
class I2C : private xmcu::Non_copyable
{
public:
    enum class Transfer_result
    {
        ok,
        nack,
        arbitration_lost,
    };

    using Address = std::uint32_t;
    struct Header
    {
        std::uint32_t adr;
        // const std::uint8_t *control;
        bool auto_end = true;
    };

    /// is also possible xD
    class PollingSlave; // : private xmcu::Non_copyable
    class Polling : private xmcu::Non_copyable
    {
    public:
        Transfer_result
        transmit(Address a_adr, Not_null<const void*> a_p_data, std::size_t a_size, bool a_is_auto_end);

        Transfer_result
        receive(Address a_adr, Not_null<void*> a_p_data, std::size_t a_size, bool a_is_auto_end);

        Transfer_result transmit(Address a_adr, const std::initializer_list<std::uint8_t>& a_list, bool a_is_auto_end)
        {
            return this->transmit(a_adr, a_list.begin(), a_list.size(), a_is_auto_end);
        }

        template<typename T> Transfer_result transmit(Address a_adr, const T& a_data, bool a_is_auto_end)
        {
            return this->transmit(a_adr, &a_data, sizeof(T), a_is_auto_end);
        }

        template<typename T> Transfer_result receive(Address a_adr, T* a_data, bool a_is_auto_end)
        {
            return this->receive(a_adr, a_data, sizeof(T), a_is_auto_end);
        }

        // because default argument (a_is_auto_end) is to ambiguous . . .
        template<typename T> Transfer_result transmit(Address a_adr, const T& a_data, std::size_t a_size)
        {
            return transmit(a_adr, a_data, a_size, false);
        }

        template<typename T> Transfer_result receive(Address a_adr, T* a_data, std::size_t a_size)
        {
            return receive(a_adr, a_data, a_size, false);
        }

    private:
        Polling(I2C_TypeDef* a_p_registers);
        I2C_TypeDef* p_registers;
        friend class I2C;
    };

    I2C(I2C_TypeDef* a_p_registers, std::uint32_t (*a_get_clk)())
        : polling { a_p_registers }
        , p_registers { a_p_registers }
        , get_clk { a_get_clk }
    {
    }
    bool enable();

    template<std::size_t t_data_size>
    Transfer_result transmit(const Header& a_adr, const std::array<std::uint8_t, t_data_size>& a_data)
    {
        return this->polling.transmit(a_adr.adr, a_data, a_adr.auto_end);
    }

    Polling polling;

protected:
    I2C_TypeDef* p_registers;
    std::uint32_t (*get_clk)();
};

} // namespace xmcu::soc::m4::stm32wb::peripherals

namespace xmcu::soc::m4::stm32wb {
template<std::uint32_t id> class rcc<peripherals::I2C, id> : private xmcu::Non_constructible
{
public:
    template<typename Source_t> static void enable(bool a_enable_in_lp) = delete;
    static void disable()                                               = delete;
    static std::uint32_t get_frequency_Hz()                             = delete;
};

template<> template<> void rcc<peripherals::I2C, 1u>::enable<rcc<system::mcu<1u>>::pclk<1u>>(bool a_enable_in_lp);
template<> template<> void rcc<peripherals::I2C, 1u>::enable<rcc<system::mcu<1u>>>(bool a_enable_in_lp);
template<> template<> void rcc<peripherals::I2C, 1u>::enable<sources::hsi16>(bool a_enable_in_lp);
template<> void rcc<peripherals::I2C, 1u>::disable();
template<> std::uint32_t rcc<peripherals::I2C, 1u>::get_frequency_Hz();

template<> template<> void rcc<peripherals::I2C, 3u>::enable<rcc<system::mcu<1u>>::pclk<1u>>(bool a_enable_in_lp);
template<> template<> void rcc<peripherals::I2C, 3u>::enable<rcc<system::mcu<1u>>>(bool a_enable_in_lp);
template<> template<> void rcc<peripherals::I2C, 3u>::enable<sources::hsi16>(bool a_enable_in_lp);
template<> void rcc<peripherals::I2C, 3u>::disable();
template<> std::uint32_t rcc<peripherals::I2C, 3u>::get_frequency_Hz();

} // namespace xmcu::soc::m4::stm32wb

namespace xmcu::soc::m4::stm32wb {

template<> inline void
peripherals::GPIO::Alternate_function::enable<peripherals::I2C, 1u>(Limited<std::uint32_t, 0, 15> a_id,
                                                                    const Enable_config& a_config,
                                                                    Pin* a_p_pin)
{
    hkm_assert(Type::open_drain == a_config.type);
    hkm_assert(1 == this->p_port->idx && (8u == a_id || 9u == a_id));
    this->enable(a_id, a_config, 4u, a_p_pin);
}

} // namespace xmcu::soc::m4::stm32wb

namespace xmcu {
namespace soc {
template<> class peripheral<m4::stm32wb::peripherals::I2C, 1u> : private xmcu::Non_constructible
{
public:
    static m4::stm32wb::peripherals::I2C create()
    {
        std::uint32_t (*fun)() = m4::stm32wb::rcc<m4::stm32wb::peripherals::I2C, 1u>::get_frequency_Hz;
        return m4::stm32wb::peripherals::I2C(I2C1, fun);
    }
};
} // namespace soc
} // namespace xmcu
