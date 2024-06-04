#pragma once

/**/

// externals
#include <stm32l0xx.h>

// hkm
#include <xmcu/Duration.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/various.hpp>
#include <xmcu/soc/ST/m0/IRQ_config.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/rcc.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/sources/hsi16.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/system/mcu/mcu.hpp>
#include <xmcu/soc/peripheral.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace peripherals {
class LPTIM : private xmcu::Non_copyable
{
public:
    class Tick_counter : private xmcu::Non_copyable
    {
    public:
        enum class Mode : std::uint32_t
        {
            continuous = LPTIM_CR_CNTSTRT,
            one_pulse  = LPTIM_CR_SNGSTRT
        };
        enum class Prescaler : std::uint32_t
        {
            _1   = 0x0u,
            _2   = LPTIM_CFGR_PRESC_0,
            _4   = LPTIM_CFGR_PRESC_1,
            _8   = LPTIM_CFGR_PRESC_0 | LPTIM_CFGR_PRESC_1,
            _16  = LPTIM_CFGR_PRESC_2,
            _32  = LPTIM_CFGR_PRESC_0 | LPTIM_CFGR_PRESC_2,
            _64  = LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_2,
            _128 = LPTIM_CFGR_PRESC_0 | LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_2,
        };

        class Polling : private xmcu::Non_copyable
        {
        public:
            bool is_overload() const;

            std::uint16_t get_counter() const
            {
                return static_cast<std::uint16_t>(this->p_LPTIM->p_registers->CNT);
            }

        private:
            LPTIM* p_LPTIM;
            friend LPTIM;
        };
        class Interrupt : private xmcu::Non_copyable
        {
        public:
            struct Callback
            {
                using Function = void (*)(void* a_p_user_data);

                Function function = nullptr;
                void* p_user_data = nullptr;
            };

            void enable(const IRQ_config& a_config);
            void disable();

            void register_callback(const Callback& a_callback);
            void unregister_callback();

        private:
            LPTIM* p_LPTIM;
            friend LPTIM;
        };

        void enable(Prescaler a_prescaler);

        void start(Mode a_mode, std::uint16_t a_auto_reload);
        bool start(Mode a_mode, std::uint16_t a_auto_reload, Milliseconds a_timeout);

        void stop();

        Polling polling;
        Interrupt interrupt;

    private:
        LPTIM* p_LPTIM;
        friend LPTIM;
    };

    LPTIM(LPTIM&&)            = default;
    LPTIM& operator=(LPTIM&&) = default;

    LPTIM()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
        this->tick_counter.p_LPTIM           = nullptr;
        this->tick_counter.polling.p_LPTIM   = nullptr;
        this->tick_counter.interrupt.p_LPTIM = nullptr;
    }
    ~LPTIM()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    void disable();

    bool is_enabled() const
    {
        return bit_flag::is(this->p_registers->CR, LPTIM_CR_ENABLE);
    }

    operator LPTIM_TypeDef*()
    {
        return this->p_registers;
    }

    operator const LPTIM_TypeDef*() const
    {
        return this->p_registers;
    }

    Tick_counter tick_counter;

private:
    LPTIM(std::size_t a_idx, LPTIM_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , irqn(a_irqn)
    {
        this->tick_counter.p_LPTIM           = this;
        this->tick_counter.polling.p_LPTIM   = this;
        this->tick_counter.interrupt.p_LPTIM = this;
    }

    std::uint32_t idx;
    LPTIM_TypeDef* p_registers;

    IRQn_Type irqn;
    Tick_counter::Interrupt::Callback tick_counter_callback;

    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
    friend void LPTIM_interrupt_handler(LPTIM* a_p_this);
};
void LPTIM_interrupt_handler(LPTIM* a_p_this);
} // namespace peripherals
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
template<std::uint32_t id> class rcc<peripherals::LPTIM, id> : private xmcu::Non_constructible
{
public:
    template<typename Source_t> static void enable(bool a_enable_in_lp) = delete;
    static void disable()                                               = delete;
};

template<> template<> void rcc<peripherals::LPTIM, 1>::enable<rcc<system::mcu<1u>>::pclk<1u>>(bool a_enable_in_lp);
template<> template<> void rcc<peripherals::LPTIM, 1>::enable<sources::hsi16>(bool a_enable_in_lp);

} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
template<> class peripheral<m0::stm32l0::rm0451::peripherals::LPTIM, 1u> : private Non_constructible
{
public:
    static m0::stm32l0::rm0451::peripherals::LPTIM create()
    {
        return m0::stm32l0::rm0451::peripherals::LPTIM(0u, LPTIM1, IRQn_Type::LPTIM1_IRQn);
    }
};
} // namespace soc
} // namespace xmcu
