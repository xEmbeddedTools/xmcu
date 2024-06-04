#pragma once

/**/

// std
#include <limits>

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/Not_null.hpp>
#include <xmcu/various.hpp>
#include <xmcu/soc/ST/m4/IRQ_config.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/DMA.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/USART/LPUART.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/USART/USART.hpp>
#include <xmcu/soc/peripheral.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
template<> class DMA<peripherals::USART> : private Non_copyable
{
public:
    class Receiver : private Non_copyable
    {
    public:
        class Polling : private Non_copyable
        {
        public:
            void receive(DMA<>::Priority a_priority,
                         DMA<>::Mode a_mode,
                         Not_null<volatile void*> a_p_buffer,
                         std::uint16_t a_buffer_size_in_words);

            DMA<>::Result wait_for_data(bool wait_until_channel_disabled,
                                        std::size_t a_buffer_size_in_words,
                                        Milliseconds a_timeout);

            void stop()
            {
                bit_flag::clear(&(this->p_DMA->p_rx_channel_registers->CCR), DMA_CCR_EN);
            }

        private:
            Polling(DMA<peripherals::USART>* a_p_DMA)
                : p_DMA(a_p_DMA)
            {
            }

            DMA<peripherals::USART>* const p_DMA;
            template<typename Peripheral_t> friend class DMA;
        };
        class Interrupt : private Non_copyable
        {
        public:
            void enable(const IRQ_config& a_irq_config, const DMA<>::Callback& a_callback, DMA<>::Event_flag a_flag);
            void disable();
            bool is_enabled() const;

            void start(DMA<>::Priority a_priority,
                       DMA<>::Mode a_mode,
                       Not_null<volatile void*> a_p_buffer,
                       std::uint16_t a_buffer_size_in_words);
            void stop();

        private:
            Interrupt(DMA<peripherals::USART>* a_p_DMA)
                : p_DMA(a_p_DMA)
            {
            }

            void set_context();
            void clear_context();

            DMA<peripherals::USART>* const p_DMA;
            template<typename Peripheral_t> friend class DMA;
        };

        void enable(DMA<>::Channel a_channel);
        void disable();

        Polling polling;
        Interrupt interrupt;

    private:
        Receiver(DMA<peripherals::USART>* a_p_DMA)
            : polling(a_p_DMA)
            , interrupt(a_p_DMA)
            , p_DMA(a_p_DMA)
        {
        }

        DMA<peripherals::USART>* const p_DMA;
        template<typename Peripheral_t> friend class DMA;
    };
    class Transmitter : private Non_copyable
    {
    public:
        class Polling : private Non_copyable
        {
        public:
            DMA<>::Result transmit(DMA<>::Priority a_priority,
                                   DMA<>::Mode a_mode,
                                   Not_null<const void*> a_p_buffer,
                                   std::uint16_t a_buffer_size_in_words);

            DMA<>::Result transmit(DMA<>::Priority a_priority,
                                   DMA<>::Mode a_mode,
                                   Not_null<const void*> a_p_buffer,
                                   std::uint16_t a_buffer_size_in_words,
                                   Milliseconds a_timeout);

        private:
            Polling(DMA<peripherals::USART>* a_p_DMA)
                : p_DMA(a_p_DMA)
            {
            }

            DMA<peripherals::USART>* const p_DMA;
            template<typename Peripheral_t> friend class DMA;
        };
        class Interrupt : private Non_copyable
        {
        public:
            void enable(const IRQ_config& a_irq_config, const DMA<>::Callback& a_callback, DMA<>::Event_flag a_flag);
            void disable();
            bool is_enabled() const;

            void start(DMA<>::Priority a_priority,
                       DMA<>::Mode a_mode,
                       Not_null<volatile const void*> a_p_buffer,
                       std::uint16_t a_buffer_size_in_words);
            void stop();

        private:
            Interrupt(DMA<peripherals::USART>* a_p_DMA)
                : p_DMA(a_p_DMA)
            {
            }

            void set_context();
            void clear_context();

            DMA<peripherals::USART>* const p_DMA;
            template<typename Peripheral_t> friend class DMA;
        };

        void enable(DMA<>::Channel a_channel);
        void disable();

        Polling polling;
        Interrupt interrupt;

    private:
        Transmitter(DMA<peripherals::USART>* a_p_DMA)
            : polling(a_p_DMA)
            , interrupt(a_p_DMA)
            , p_DMA(a_p_DMA)
        {
        }

        DMA<peripherals::USART>* const p_DMA;
        template<typename Peripheral_t> friend class DMA;
    };

    DMA()
        : receiver(this)
        , transmitter(this)
        , idx(std::numeric_limits<std::size_t>::max())
        , p_DMA_registers(nullptr)
        , p_USART_registers(nullptr)
        , p_tx_channel_registers(nullptr)
        , rx_channel(various::get_enum_incorrect_value<DMA<>::Channel>())
        , tx_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
        , p_rx_channel_registers(nullptr)
        , tx_channel(various::get_enum_incorrect_value<DMA<>::Channel>())
        , rx_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
    }

    Receiver receiver;
    Transmitter transmitter;

private:
    DMA(std::size_t a_idx, DMA_TypeDef* a_p_DMA_registers, USART_TypeDef* a_p_USART_registers)
        : receiver(this)
        , transmitter(this)
        , idx(a_idx)
        , p_DMA_registers(a_p_DMA_registers)
        , p_USART_registers(a_p_USART_registers)
        , p_tx_channel_registers(nullptr)
        , rx_channel(various::get_enum_incorrect_value<DMA<>::Channel>())
        , tx_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
        , p_rx_channel_registers(nullptr)
        , tx_channel(various::get_enum_incorrect_value<DMA<>::Channel>())
        , rx_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
    }

    std::uint32_t idx;
    DMA_TypeDef* p_DMA_registers;

    USART_TypeDef* p_USART_registers;

    DMA_Channel_TypeDef* p_tx_channel_registers;
    DMA<>::Channel rx_channel;
    IRQn_Type tx_irqn;

    DMA_Channel_TypeDef* p_rx_channel_registers;
    DMA<>::Channel tx_channel;
    IRQn_Type rx_irqn;

    DMA<>::Callback tx_callback;
    DMA<>::Callback rx_callback;

    template<typename Periph_t, std::size_t id> friend class xmcu::soc::peripheral;
};

template<> class DMA<peripherals::LPUART> : private Non_copyable
{
public:
    class Receiver : private Non_copyable
    {
    public:
        class Polling : private Non_copyable
        {
        public:
            DMA<>::Result receive(DMA<>::Priority a_priority,
                                  DMA<>::Mode a_mode,
                                  Not_null<void*> a_p_buffer,
                                  std::uint16_t a_buffer_size_in_words);

            DMA<>::Result receive(DMA<>::Priority a_priority,
                                  DMA<>::Mode a_mode,
                                  Not_null<void*> a_p_buffer,
                                  std::uint16_t a_buffer_size_in_words,
                                  Milliseconds a_timeout);

        private:
            Polling(DMA<peripherals::LPUART>* a_p_DMA)
                : p_DMA(a_p_DMA)
            {
            }

            DMA<peripherals::LPUART>* const p_DMA;
            template<typename Peripheral_t> friend class DMA;
        };
        class Interrupt : private Non_copyable
        {
        public:
            void enable(const IRQ_config& a_irq_config, const DMA<>::Callback& a_callback, DMA<>::Event_flag a_flag);
            void disable();
            bool is_enabled() const;

            void start(DMA<>::Priority a_priority,
                       DMA<>::Mode a_mode,
                       Not_null<volatile void*> a_p_buffer,
                       std::uint16_t a_buffer_size_in_words);
            void stop();

        private:
            Interrupt(DMA<peripherals::LPUART>* a_p_DMA)
                : p_DMA(a_p_DMA)
            {
            }

            void set_context();
            void clear_context();

            DMA<peripherals::LPUART>* const p_DMA;
            template<typename Peripheral_t> friend class DMA;
        };

        void enable(DMA<>::Channel a_channel);
        void disable();

        Polling polling;
        Interrupt interrupt;

    private:
        Receiver(DMA<peripherals::LPUART>* a_p_DMA)
            : polling(a_p_DMA)
            , interrupt(a_p_DMA)
            , p_DMA(a_p_DMA)
        {
        }

        DMA<peripherals::LPUART>* const p_DMA;
        template<typename Peripheral_t> friend class DMA;
    };
    class Transmitter : private Non_copyable
    {
    public:
        class Polling : private Non_copyable
        {
        public:
            DMA<>::Result transmit(DMA<>::Priority a_priority,
                                   DMA<>::Mode a_mode,
                                   Not_null<const void*> a_p_buffer,
                                   std::uint16_t a_buffer_size_in_words);

            DMA<>::Result transmit(DMA<>::Priority a_priority,
                                   DMA<>::Mode a_mode,
                                   Not_null<const void*> a_p_buffer,
                                   std::uint16_t a_buffer_size_in_words,
                                   Milliseconds a_timeout);

        private:
            Polling(DMA<peripherals::LPUART>* a_p_DMA)
                : p_DMA(a_p_DMA)
            {
            }

            DMA<peripherals::LPUART>* const p_DMA;
            template<typename Peripheral_t> friend class DMA;
        };
        class Interrupt : private Non_copyable
        {
        public:
            void enable(const IRQ_config& a_irq_config, const DMA<>::Callback& a_callback, DMA<>::Event_flag a_flag);
            void disable();
            bool is_enabled() const;

            void start(DMA<>::Priority a_priority,
                       DMA<>::Mode a_mode,
                       Not_null<volatile const void*> a_p_buffer,
                       std::uint16_t a_buffer_size_in_words);
            void stop();

        private:
            Interrupt(DMA<peripherals::LPUART>* a_p_DMA)
                : p_DMA(a_p_DMA)
            {
            }

            void set_context();
            void clear_context();

            DMA<peripherals::LPUART>* const p_DMA;
            template<typename Peripheral_t> friend class DMA;
        };

        void enable(DMA<>::Channel a_channel);
        void disable();

        Polling polling;
        Interrupt interrupt;

    private:
        Transmitter(DMA<peripherals::LPUART>* a_p_DMA)
            : polling(a_p_DMA)
            , interrupt(a_p_DMA)
            , p_DMA(a_p_DMA)
        {
        }

        DMA<peripherals::LPUART>* const p_DMA;
        template<typename Peripheral_t> friend class DMA;
    };

    Receiver receiver;
    Transmitter transmitter;

private:
    DMA(std::size_t a_idx, DMA_TypeDef* a_p_DMA_registers, USART_TypeDef* a_p_USART_registers)
        : receiver(this)
        , transmitter(this)
        , idx(a_idx)
        , p_DMA_registers(a_p_DMA_registers)
        , p_USART_registers(a_p_USART_registers)
        , p_tx_channel_registers(nullptr)
        , rx_channel(various::get_enum_incorrect_value<DMA<>::Channel>())
        , tx_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
        , p_rx_channel_registers(nullptr)
        , tx_channel(various::get_enum_incorrect_value<DMA<>::Channel>())
        , rx_irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
    }

    std::uint32_t idx;
    DMA_TypeDef* p_DMA_registers;

    USART_TypeDef* p_USART_registers;

    DMA_Channel_TypeDef* p_tx_channel_registers;
    DMA<>::Channel rx_channel;
    IRQn_Type tx_irqn;

    DMA_Channel_TypeDef* p_rx_channel_registers;
    DMA<>::Channel tx_channel;
    IRQn_Type rx_irqn;

    DMA<>::Callback tx_callback;
    DMA<>::Callback rx_callback;

    template<typename Periph_t, std::size_t id> friend class xmcu::soc::peripheral;
};
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
template<> class peripheral<m4::stm32wb::peripherals::USART, 1u, m4::stm32wb::DMA<>, 1u>
    : private Non_constructible
{
public:
    static m4::stm32wb::DMA<m4::stm32wb::peripherals::USART> create()
    {
        return m4::stm32wb::DMA<m4::stm32wb::peripherals::USART>(0x0u, DMA1, USART1);
    }
};
template<> class peripheral<m4::stm32wb::peripherals::USART, 1u, m4::stm32wb::DMA<>, 2u>
    : private Non_constructible
{
public:
    static m4::stm32wb::DMA<m4::stm32wb::peripherals::USART> create()
    {
        return m4::stm32wb::DMA<m4::stm32wb::peripherals::USART>(0x1u, DMA2, USART1);
    }
};

template<> class peripheral<m4::stm32wb::peripherals::LPUART, 1u, m4::stm32wb::DMA<>, 1u>
    : private Non_constructible
{
public:
    static m4::stm32wb::DMA<m4::stm32wb::peripherals::LPUART> create()
    {
        return m4::stm32wb::DMA<m4::stm32wb::peripherals::LPUART>(0x0u, DMA1, LPUART1);
    }
};
template<> class peripheral<m4::stm32wb::peripherals::LPUART, 1u, m4::stm32wb::DMA<>, 2u>
    : private Non_constructible
{
public:
    static m4::stm32wb::DMA<m4::stm32wb::peripherals::LPUART> create()
    {
        return m4::stm32wb::DMA<m4::stm32wb::peripherals::LPUART>(0x1u, DMA2, LPUART1);
    }
};
} // namespace soc
} // namespace xmcu
