#pragma once

/**/

// std
#include <array>
#include <cstdint>
#include <limits>
#include <utility>

// externals
#include <stm32l0xx.h>

// hkm
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/Not_null.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/various.hpp>
#include <xmcu/soc/ST/m0/IRQ_config.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/rcc.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/system/mcu/mcu.hpp>
#include <xmcu/soc/peripheral.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace peripherals {

class ADC : private Non_copyable
{
public:
    struct s : Non_constructible
    {
        static constexpr std::size_t max_channels_count = 18;
    };

    enum class Mode : std::uint32_t
    {
        single        = 0x0u,
        continuous    = ADC_CFGR1_CONT,
        discontinuous = ADC_CFGR1_DISCEN
    };
    enum class Resolution : std::uint32_t
    {
        _6_bit  = ADC_CFGR1_RES_1 | ADC_CFGR1_RES_0,
        _8_bit  = ADC_CFGR1_RES_1,
        _10_bit = ADC_CFGR1_RES_0,
        _12_bit = 0u,
    };

    struct Calibration_data
    {
        std::uint16_t internal_voltage_reference = 0u;
    };

    struct Channel
    {
        enum class Id : std::uint32_t
        {
            _0,
            _1,
            _2,
            _3,
            _4,
            _5,
            _6,
            _7,
            _8,
            _9,
            _10,
            _11,
            _12,
            _13,
            _14,
            _15,
            reserved_16,
            voltage_reference,
        };

        enum class Sampling_time : std::uint32_t
        {
            _1_5_clock_cycles = 0x0u,
            _3_5_clock_cycles = 0x1u,
            _7_5_clock_cycles = 0x2u,
            _12_5_clock_cycles = 0x3u,
            _19_5_clock_cycles = 0x4u,
            _39_5_clock_cycles = 0x5u,
            _79_5_clock_cycles = 0x6u,
            _160_5_clock_cycles = 0x7u,
        };
    };

    class Polling : private Non_copyable
    {
    public:
        template<Mode mode> void read_setup(std::size_t a_group_size) = delete;

        /**
         * @brief Reads the ADC value without turning off interrupts. Use only if it's time-critical.
         *
         * @warning You should disable NVIC yourself, or else the ADC may overrun and you'll be stuck in endless loop!
         *
         * @param a_p_buffer Destination buffer.
         * @param a_buffer_capacity Amount of samples to write to buffer.
         *
         * @see read()
         */
        void read_unsafe(Not_null<std::uint16_t*> a_p_buffer, std::size_t a_buffer_capacity);

        void read(Not_null<std::uint16_t*> a_p_buffer, std::size_t a_buffer_capacity);

        bool read(Not_null<std::uint16_t*> a_p_buffer,
                  std::size_t a_buffer_capacity,
                  Milliseconds a_timeout);

    private:
        ADC* p_ADC;
        friend ADC;
    };
    class Interrupt : private Non_copyable
    {
    public:
        struct Callback
        {
            using Function = void (*)(ADC* a_p_this, std::uint16_t a_value, bool a_series_end, void* a_p_user_data);

            Function function = nullptr;
            void* p_user_data = nullptr;
        };

        ~Interrupt()
        {
            if (true == this->is_enabled())
            {
                this->disable();
            }
        }

        void enable(const IRQ_config& a_irq_config);
        void disable();

        template<Mode mode> void read_start(const Callback& a_callback)                           = delete;
        template<Mode mode> void read_start(const Callback& a_callback, std::size_t a_group_size) = delete;
        void read_stop();

        bool is_enabled() const
        {
            return 0 != NVIC_GetEnableIRQ(this->p_ADC->irqn);
        }

    private:
        ADC* p_ADC;
        friend ADC;
    };

    ADC(ADC&&)            = default;
    ADC& operator=(ADC&&) = default;

    ADC()
        : idx(std::numeric_limits<decltype(this->idx)>::max())
        , p_registers(nullptr)
        , irqn(static_cast<IRQn_Type>(std::numeric_limits<std::uint32_t>::max()))
    {
        this->polling.p_ADC   = nullptr;
        this->interrupt.p_ADC = nullptr;
    }
    ~ADC()
    {
        if (true == this->is_enabled())
        {
            this->disable();
        }
    }

    template<std::size_t length> void
    enable(Resolution a_resolution, const std::array<Channel::Id, length>& a_channels, Channel::Sampling_time a_sampling_time)
    {
        this->enable(a_resolution, a_channels.data(), a_channels.size(), a_sampling_time);
    }
    template<std::size_t length> bool enable(Resolution a_resolution,
                                             const std::array<Channel::Id, length>& a_channels,
                                             Channel::Sampling_time a_sampling_time,
                                             Milliseconds a_timeout)
    {
        return this->enable(a_resolution, a_channels.data(), a_channels.size(), a_sampling_time, a_timeout);
    }

    void disable();

    Resolution get_resolution() const;
    std::array<std::pair<ADC::Channel::Id, bool>, ADC::s::max_channels_count> get_enabled_channels();
    Channel::Sampling_time get_sampling_time();

    constexpr Calibration_data get_calibration_data() const
    {
        return { .internal_voltage_reference = *(reinterpret_cast<const std::uint16_t*>(0x1FFF75A8)) };
    }

    bool is_enabled() const
    {
        return ADC_CR_ADEN == bit_flag::get(this->p_registers->CR, ADC_CR_ADEN | ADC_CR_ADDIS);
    }

    bool is_created() const
    {
        return std::numeric_limits<decltype(this->idx)>::max() != this->idx && nullptr != this->p_registers;
    }

    operator ADC_TypeDef*()
    {
        return this->p_registers;
    }

    operator const ADC_TypeDef*() const
    {
        return this->p_registers;
    }

    Polling polling;
    Interrupt interrupt;

private:
    ADC(std::uint32_t a_idx, ADC_TypeDef* a_p_registers, IRQn_Type a_irqn)
        : idx(a_idx)
        , p_registers(a_p_registers)
        , irqn(a_irqn)
    {
        this->polling.p_ADC   = this;
        this->interrupt.p_ADC = this;
    }

    void enable(ADC::Resolution a_resolution,
                const ADC::Channel::Id* a_p_channels,
                std::size_t a_channels_count,
                Channel::Sampling_time a_sampling_time);
    bool enable(ADC::Resolution a_resolution,
                const ADC::Channel::Id* a_p_channels,
                std::size_t a_channels_count,
                Channel::Sampling_time a_sampling_time,
                Milliseconds a_timeout);

    std::uint32_t idx;
    ADC_TypeDef* p_registers;

    IRQn_Type irqn;
    Interrupt::Callback callback;

    template<typename Periph_t, std::size_t id> friend class xmcu::soc::peripheral;
    friend void ADC_interrupt_handler(ADC* a_p_this);
};

void ADC_interrupt_handler(ADC* a_p_this);

template<> void ADC::Polling::read_setup<ADC::Mode::single>(std::size_t a_group_size);
template<> void ADC::Polling::read_setup<ADC::Mode::continuous>(std::size_t a_group_size);
template<> void ADC::Polling::read_setup<ADC::Mode::discontinuous>(std::size_t a_group_size);

template<> void ADC::Interrupt::read_start<ADC::Mode::single>(const Callback& a_callback);
template<> void ADC::Interrupt::read_start<ADC::Mode::continuous>(const Callback& a_callback);
template<>
void ADC::Interrupt::read_start<ADC::Mode::discontinuous>(const Callback& a_callback, std::size_t a_group_size);

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
template<> class rcc<peripherals::ADC> : private Non_constructible
{
public:
    struct sync : private Non_constructible
    {
        enum class Prescaler : std::uint32_t
        {
            _2 = ADC_CFGR2_CKMODE_0,
            _4 = ADC_CFGR2_CKMODE_1,
        };

        template<typename Source_t> static void enable(Prescaler a_prescaler, bool a_enable_in_lp) = delete;

        static void disable()
        {
            bit_flag::clear(&ADC1->CFGR2, ADC_CFGR2_CKMODE_Msk);
            bit_flag::clear(&ADC1_COMMON->CCR, ADC_CCR_PRESC_Msk);
            bit_flag::clear(&RCC->APB2ENR, RCC_APB2ENR_ADCEN);
        }
    };

    struct async : private Non_constructible
    {
        enum class Prescaler : std::uint32_t
        {
            _1   = 0x0u,
            _2   = ADC_CCR_PRESC_0,
            _4   = ADC_CCR_PRESC_1,
            _6   = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1,
            _8   = ADC_CCR_PRESC_2,
            _10  = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_2,
            _12  = ADC_CCR_PRESC_1 | ADC_CCR_PRESC_2,
            _16  = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_2,
            _32  = ADC_CCR_PRESC_3,
            _64  = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_3,
            _128 = ADC_CCR_PRESC_1 | ADC_CCR_PRESC_3,
            _256 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_3
        };

        template<typename Source_t> static void enable(Prescaler a_prescaler, bool a_enable_in_lp) = delete;

        static void disable()
        {
            bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CFGR2_CKMODE_Msk | ADC_CCR_PRESC_Msk);
            bit_flag::clear(&(RCC->APB2ENR), RCC_APB2ENR_ADCEN);
        }
    };
};

template<> void rcc<peripherals::ADC>::async::enable<rcc<system::mcu<1u>>>(Prescaler a_prescaler, bool a_enable_in_lp);
template<>
void rcc<peripherals::ADC>::sync::enable<rcc<system::mcu<1u>>::hclk<1u>>(Prescaler a_prescaler, bool a_enable_in_lp);
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
template<> class peripheral<m0::stm32l0::rm0451::peripherals::ADC, 1u> : private Non_constructible
{
public:
    static m0::stm32l0::rm0451::peripherals::ADC create()
    {
        return m0::stm32l0::rm0451::peripherals::ADC(0U, ADC1, IRQn_Type::ADC1_IRQn);
    }
};
} // namespace soc
} // namespace xmcu
