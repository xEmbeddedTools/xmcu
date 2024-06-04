#pragma once

/**/

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Non_copyable.hpp>
#include <xmcu/Not_null.hpp>
#include <xmcu/various.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/USART/USART.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {
class LPUART : private Non_copyable
{
public:
    using Event_flag              = USART::Event_flag;
    using Low_power_wakeup_method = USART::Low_power_wakeup_method;
    using Clock_config            = USART::Clock_config;
    using Frame_format            = USART::Frame_format;

    struct Transceiving_config
    {
        enum class Stop_bits : std::uint32_t
        {
            _0_5 = USART_CR2_STOP_0,
            _1 = 0x0u,
            _1_5 = USART_CR2_STOP_0 | USART_CR2_STOP_1,
            _2 = USART_CR2_STOP_1,
        };
        enum class Flow_control_flag : std::uint32_t
        {
            none = 0x0u,
            RS232 = 0x1u,
            RS485 = 0x2u,
        };
        enum Sampling_method : std::uint32_t
        {
            three_sample_bit = 0,
            one_sample_bit = USART_CR3_ONEBIT,
        };
        enum class Mode_flag : std::uint32_t
        {
            none = 0x0u,
            tx = USART_CR1_TE,
            rx = USART_CR1_RE,
        };
        enum class Mute_method : std::uint32_t
        {
            none = 0x0u,
            idle_line = 0x100u,
            character_matched = 0x200u,
        };
        enum class RS232_flow_control_flag : std::uint32_t
        {
            request_to_send = USART_CR3_RTSE,
            clear_to_send = USART_CR3_CTSE,
        };

        struct RS485_flow_control_config
        {
            std::uint8_t assertion_time = 0x0u;
            std::uint8_t deassertion_time = 0x0u;
        };

        std::uint32_t baud_rate = 0;
        Stop_bits stop_bits = various::get_enum_incorrect_value<Stop_bits>();
        Flow_control_flag flow_control = various::get_enum_incorrect_value<Flow_control_flag>();
        Sampling_method sampling_method = various::get_enum_incorrect_value<Sampling_method>();
        Mode_flag mode = various::get_enum_incorrect_value<Mode_flag>();
        Mute_method mute_method = various::get_enum_incorrect_value<Mute_method>();
    };

    class Polling : private Non_copyable
    {
    public:
        using Result = USART::Polling::Result;

        Result transmit(Not_null<const std::uint8_t*> a_p_data, std::size_t a_data_size_in_words);
        Result transmit(Not_null<const std::uint16_t*> a_p_data, std::size_t a_data_size_in_words);

        Result transmit(Not_null<const std::uint8_t*> a_p_data,
                        std::size_t a_data_size_in_words,
                        Milliseconds a_timeout);
        Result transmit(Not_null<const std::uint16_t*> a_p_data,
                        std::size_t a_data_size_in_words,
                        Milliseconds a_timeout);

        Result receive(Not_null<std::uint8_t*> a_p_data, std::size_t a_data_size_in_words);
        Result receive(Not_null<std::uint16_t*> a_p_data, std::size_t a_data_size_in_words);

        Result receive(Not_null<std::uint8_t*> a_p_data,
                       std::size_t a_data_size_in_words,
                       Milliseconds a_timeout);
        Result receive(Not_null<std::uint16_t*> a_p_data,
                       std::size_t a_data_size_in_words,
                       Milliseconds a_timeout);

        template<typename t_Type> std::uint32_t transmit_2(const t_Type& a_data)
        {
            const auto itr_begin = std::begin(a_data);
            const auto itr_end   = std::end(a_data);
            auto itr             = itr_begin;

            bit_flag::set(&(this->p_LPUART->p_registers->ICR),
                                  USART_ICR_TCCF | USART_ICR_PECF | USART_ICR_NECF);

            while (itr != itr_end &&
                   false == bit::is_any(this->p_LPUART->p_registers->ISR, USART_ISR_PE | USART_ISR_NE))
            {
                if (true == bit_flag::is(this->p_LPUART->p_registers->ISR, USART_ISR_TXE))
                {
                    this->p_LPUART->p_registers->TDR = *itr;
                    itr++;
                }
            }

            if (false == bit::is_any(this->p_LPUART->p_registers->ISR, USART_ISR_PE | USART_ISR_NE))
            {
                utils::wait_until::all_bits_are_set(this->p_LPUART->p_registers->ISR, USART_ISR_TC);
            }

            return static_cast<std::uint32_t>(itr - itr_begin);
        }
        template<typename t_First, typename... t_Tail>
        std::uint32_t transmit_2(const t_First& a_first, const t_Tail&... a_tail)
        {
            const auto itr_begin = std::begin(a_first);
            const auto itr_end   = std::end(a_first);
            auto itr             = itr_begin;

            bit_flag::set(&(this->p_LPUART->p_registers->ICR),
                                  USART_ICR_TCCF | USART_ICR_PECF | USART_ICR_NECF);

            while (itr != itr_end &&
                   false == bit::is_any(this->p_LPUART->p_registers->ISR, USART_ISR_PE | USART_ISR_NE))
            {
                if (true == bit_flag::is(this->p_LPUART->p_registers->ISR, USART_ISR_TXE))
                {
                    this->p_LPUART->p_registers->TDR = *itr;
                    itr++;
                }
            }

            if (false == bit::is_any(this->p_LPUART->p_registers->ISR, USART_ISR_PE | USART_ISR_NE))
            {
                return this->transmit_2(a_tail...) + static_cast<std::uint32_t>(itr - itr_begin);
            }

            return static_cast<std::uint32_t>(itr - itr_begin);
        }

        template<typename t_Type> std::uint32_t transmit_2(Milliseconds a_timeout, const t_Type& a_data)
        {
            const std::uint64_t timeout_end_timestamp =
                utils::tick_counter<Milliseconds>::get() + a_timeout.get();

            const auto itr_begin = std::begin(a_data);
            const auto itr_end   = std::end(a_data);
            auto itr             = itr_begin;

            bit_flag::set(&(this->p_LPUART->p_registers->ICR),
                                  USART_ICR_TCCF | USART_ICR_PECF | USART_ICR_NECF);

            while (itr != itr_end &&
                   false == bit::is_any(this->p_LPUART->p_registers->ISR, USART_ISR_PE | USART_ISR_NE) &&
                   utils::tick_counter<Milliseconds>::get() <= timeout_end_timestamp)
            {
                if (true == bit_flag::is(this->p_LPUART->p_registers->ISR, USART_ISR_TXE))
                {
                    this->p_LPUART->p_registers->TDR = *itr;
                    itr++;
                }
            }

            if (false == bit::is_any(this->p_LPUART->p_registers->ISR, USART_ISR_PE | USART_ISR_NE))
            {
                utils::wait_until::all_bits_are_set(this->p_LPUART->p_registers->ISR, USART_ISR_TC);
            }

            return static_cast<std::uint32_t>(itr - itr_begin);
        }
        template<typename t_First, typename... t_Tail>
        std::uint32_t transmit_2(Milliseconds a_timeout, const t_First& a_first, const t_Tail&... a_tail)
        {
            const std::uint64_t timeout_start_timestamp = utils::tick_counter<Milliseconds>::get();
            const std::uint64_t timeout_end_timestamp   = timeout_start_timestamp + a_timeout.get();

            const auto itr_begin = std::begin(a_first);
            const auto itr_end   = std::end(a_first);
            auto itr             = itr_begin;

            bit_flag::set(&(this->p_LPUART->p_registers->ICR),
                                  USART_ICR_TCCF | USART_ICR_PECF | USART_ICR_NECF);

            while (itr != itr_end &&
                   false == bit::is_any(this->p_LPUART->p_registers->ISR, USART_ISR_PE | USART_ISR_NE) &&
                   utils::tick_counter<Milliseconds>::get() <= timeout_end_timestamp)
            {
                if (true == bit_flag::is(this->p_LPUART->p_registers->ISR, USART_ISR_TXE))
                {
                    this->p_LPUART->p_registers->TDR = *itr;
                    itr++;
                }
            }

            if (false == bit::is_any(this->p_LPUART->p_registers->ISR, USART_ISR_PE | USART_ISR_NE) &&
                utils::tick_counter<Milliseconds>::get() <= timeout_end_timestamp)
            {
                const std::uint64_t new_timeout =
                    a_timeout.get() - (utils::tick_counter<Milliseconds>::get() - timeout_start_timestamp);
                return this->transmit_2(Milliseconds(new_timeout), a_tail...) +
                       static_cast<std::uint32_t>(itr - itr_begin);
            }

            return static_cast<std::uint32_t>(itr - itr_begin);
        }

    private:
        LPUART* p_LPUART = nullptr;
        friend class LPUART;
    };
    class Interrupt : private Non_copyable
    {
    public:
        using Transmit_callback = USART::Interrupt::Transmit_callback;
        using Receive_callback  = USART::Interrupt::Receive_callback;
        using Event_callback    = USART::Interrupt::Event_callback;

        ~Interrupt()
        {
            if (true == this->is_enabled())
            {
                this->disable();
            }
        }

        void enable(const IRQ_config& a_config);
        void disable();

        void transmit_start(const Transmit_callback& a_callback);
        void transmit_stop();

        void receive_start(const Receive_callback& a_callback);
        void receive_stop();

        void event_listening_start(const Event_callback& a_callback);
        void event_listening_stop();

        bool is_enabled() const
        {
            return 0 != NVIC_GetEnableIRQ(this->p_LPUART->irqn);
        }

    private:
        LPUART* p_LPUART = nullptr;
        friend class LPUART;
    };

    LPUART(LPUART&&)            = default;
    LPUART& operator=(LPUART&&) = default;

    LPUART()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
        this->polling.p_LPUART   = nullptr;
        this->interrupt.p_LPUART = nullptr;
    }
    ~LPUART()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void enable(const Clock_config& a_clock_config,
                const Transceiving_config& a_transceiving_config,
                const Frame_format& frame_format,
                Low_power_wakeup_method a_low_power_wakeup);
    bool enable(const Clock_config& a_clock_config,
                const Transceiving_config& a_transceiving_config,
                const Frame_format& frame_format,
                Low_power_wakeup_method a_low_power_wakeup,
                Milliseconds a_timeout);
    void disable();

    bool is_enabled() const
    {
        return bit::is_any(this->p_registers->ISR, USART_ISR_REACK_Pos | USART_ISR_TEACK_Pos);
    }

    Transceiving_config get_Transceiving_config() const
    {
        return {};
    }

    Frame_format get_Frame_format() const
    {
        return {};
    }

    operator USART_TypeDef*()
    {
        return this->p_registers;
    }

    operator const USART_TypeDef*() const
    {
        return this->p_registers;
    }

    Polling polling;
    Interrupt interrupt;

private:
    LPUART(std::uint32_t a_idx, USART_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , irqn(a_irqn)
    {
        this->polling.p_LPUART   = this;
        this->interrupt.p_LPUART = this;
    }

    std::uint32_t idx;
    USART_TypeDef* p_registers;

    IRQn_Type irqn;
    Interrupt::Transmit_callback transmit_callback;
    Interrupt::Receive_callback receive_callback;
    Interrupt::Event_callback event_callback;

    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
    friend void LPUART_interrupt_handler(LPUART* a_p_this);
};
void LPUART_interrupt_handler(LPUART* a_p_this);

constexpr LPUART::Transceiving_config::Flow_control_flag
operator|(LPUART::Transceiving_config::Flow_control_flag a_f1,
          LPUART::Transceiving_config::RS232_flow_control_flag a_f2)
{
    hkm_assert(LPUART::Transceiving_config::Flow_control_flag::RS232 == a_f1);

    return static_cast<LPUART::Transceiving_config::Flow_control_flag>(
        static_cast<std::uint32_t>(LPUART::Transceiving_config::Flow_control_flag::RS232) |
        static_cast<std::uint32_t>(a_f2));
}

constexpr LPUART::Transceiving_config::Flow_control_flag
operator|(LPUART::Transceiving_config::Flow_control_flag a_f1,
          const LPUART::Transceiving_config::RS485_flow_control_config& a_f2)
{
    hkm_assert(LPUART::Transceiving_config::Flow_control_flag::RS485 == a_f1);

    return static_cast<LPUART::Transceiving_config::Flow_control_flag>(
        static_cast<std::uint32_t>(LPUART::Transceiving_config::Flow_control_flag::RS485) |
        (a_f2.assertion_time << USART_CR1_DEAT_Pos) | (a_f2.deassertion_time << USART_CR1_DEDT_Pos));
}

constexpr LPUART::Transceiving_config::Flow_control_flag operator&(LPUART::Transceiving_config::Flow_control_flag a_f1,
                                                                   LPUART::Transceiving_config::Flow_control_flag a_f2)
{
    return static_cast<LPUART::Transceiving_config::Flow_control_flag>((static_cast<std::uint32_t>(a_f1) & 0x3u) &
                                                                       (static_cast<std::uint32_t>(a_f2) & 0x3u));
}

constexpr LPUART::Transceiving_config::Mode_flag operator|(LPUART::Transceiving_config::Mode_flag a_f1,
                                                           LPUART::Transceiving_config::Mode_flag a_f2)
{
    return static_cast<LPUART::Transceiving_config::Mode_flag>(static_cast<std::uint32_t>(a_f1) |
                                                               static_cast<std::uint32_t>(a_f2));
}

constexpr LPUART::Transceiving_config::Mode_flag operator&(LPUART::Transceiving_config::Mode_flag a_f1,
                                                           LPUART::Transceiving_config::Mode_flag a_f2)
{
    return static_cast<LPUART::Transceiving_config::Mode_flag>(static_cast<std::uint32_t>(a_f1) &
                                                               static_cast<std::uint32_t>(a_f2));
}

constexpr LPUART::Transceiving_config::Mode_flag operator|=(LPUART::Transceiving_config::Mode_flag& a_f1,
                                                            LPUART::Transceiving_config::Mode_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

constexpr LPUART::Transceiving_config::Mute_method operator|(LPUART::Transceiving_config::Mute_method a_f1,
                                                             std::uint8_t a_f2)
{
    hkm_assert(LPUART::Transceiving_config::Mute_method::character_matched == a_f1);
    return static_cast<LPUART::Transceiving_config::Mute_method>(static_cast<std::uint32_t>(a_f1) | a_f2);
}

constexpr LPUART::Transceiving_config::RS232_flow_control_flag
operator|(LPUART::Transceiving_config::RS232_flow_control_flag a_f1,
          LPUART::Transceiving_config::RS232_flow_control_flag a_f2)
{
    return static_cast<LPUART::Transceiving_config::RS232_flow_control_flag>(static_cast<std::uint32_t>(a_f1) |
                                                                             static_cast<std::uint32_t>(a_f2));
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
template<std::uint32_t id> class rcc<peripherals::LPUART, id> : private Non_constructible
{
public:
    template<typename Source_t> static void enable(bool a_enable_in_lp) = delete;
    static void disable()                                               = delete;
};
template<> template<> void rcc<peripherals::LPUART, 1u>::enable<rcc<system::mcu<1u>>::pclk<1u>>(bool a_enable_in_lp);
template<> template<> void rcc<peripherals::LPUART, 1u>::enable<rcc<system::mcu<1u>>>(bool a_enable_in_lp);
template<> template<> void rcc<peripherals::LPUART, 1u>::enable<sources::hsi16>(bool a_enable_in_lp);
template<> template<> void rcc<peripherals::LPUART, 1u>::enable<sources::lse>(bool a_enable_in_lp);
template<> void rcc<peripherals::LPUART, 1u>::disable();

template<> inline void
peripherals::GPIO::Alternate_function::enable<peripherals::LPUART, 1>(Limited<std::uint32_t, 0, 15> a_id,
                                                                      const Enable_config& a_config,
                                                                      Pin* a_p_pin)
{
#if defined(STM32WB35xx) || defined(STM32WB55xx)
    hkm_assert((0u == this->p_port->idx && (2u == a_id || 3u == a_id || 6u == a_id)) ||
               (1u == this->p_port->idx &&
                (1u == a_id || 5u == a_id || 10u == a_id || 11u == a_id || 12u == a_id || 13u == a_id)) ||
               (2u == this->p_port->idx && (0u == a_id || 1 == a_id)));
#endif

    this->enable(a_id, a_config, 0x8u, a_p_pin);
}
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
template<> class peripheral<m4::stm32wb::peripherals::LPUART, 1u> : private Non_constructible
{
public:
    static m4::stm32wb::peripherals::LPUART create()
    {
        return m4::stm32wb::peripherals::LPUART(0u, LPUART1, IRQn_Type::LPUART1_IRQn);
    }
};
} // namespace soc
} // namespace xmcu
