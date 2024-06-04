#pragma once

/**/

// std
#include <array>
#include <cstdint>
#include <limits>

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/Not_null.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/various.hpp>
#include <xmcu/soc/ST/m4/IRQ_config.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/rcc.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lse.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/pll.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/mcu/mcu.hpp>
#include <xmcu/soc/peripheral.hpp>

// common
#include <common/Vector_array.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {
class ADC : private Non_copyable
{
public:
    struct s : Non_constructible
    {
        static constexpr std::size_t max_channels_count = 17;
    };

    enum class Mode : std::uint32_t
    {
        single        = 0x0u,
        continuous    = ADC_CFGR_CONT,
        discontinuous = ADC_CFGR_DISCEN
    };
    enum class Resolution : std::uint32_t
    {
        _6_bit = ADC_CFGR_RES_1 | ADC_CFGR_RES_0,
        _8_bit = ADC_CFGR_RES_1,
        _10_bit = ADC_CFGR_RES_0,
        _12_bit = 0u,
    };

    struct Calibration_data
    {
        std::uint16_t temperature_sensor_data_1  = 0u;
        std::uint16_t temperature_sensor_data_2  = 0u;
        std::uint16_t internal_voltage_reference = 0u;
    };
    struct Channel
    {
        enum class Id : std::uint32_t
        {
            voltage_reference,
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
            _16,
            temperature_sensor,
            battery_voltage,
        };

        enum class Sampling_time : std::uint32_t
        {
            _2_5_clock_cycles = 0x0u,
            _6_5_clock_cycles = 0x1u,
            _12_5_clock_cycles = 0x2u,
            _24_5_clock_cycles = 0x3u,
            _47_5_clock_cycles = 0x4u,
            _92_5_clock_cycles = 0x5u,
            _247_5_clock_cycles = 0x6u,
            _640_5_clock_cycles = 0x7u,
        };

        Id id = various::get_enum_incorrect_value<Id>();
        Sampling_time sampling_time = various::get_enum_incorrect_value<Sampling_time>();
    };

    class Polling : private Non_copyable
    {
    public:
        // TODO: add read_setup<Mode> for uniformity with L0, remove templates here?
        template<Mode mode> void
        read(Not_null<uint16_t*> a_p_buffer, std::size_t a_buffer_capacity, std::size_t a_group_size) = delete;
        template<Mode mode> bool read(Not_null<uint16_t*> a_p_buffer,
                                      std::size_t a_buffer_capacity,
                                      std::size_t a_group_size,
                                      Milliseconds a_timeout)                                         = delete;

        template<Mode mode> void read(Not_null<uint16_t*> a_p_buffer, std::size_t a_buffer_capacity) = delete;
        template<Mode mode> bool read(Not_null<uint16_t*> a_p_buffer,
                                      std::size_t a_buffer_capacity,
                                      Milliseconds a_timeout)                                        = delete;

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

        template<Mode mode> void read_start(const Callback& a_callback) = delete;
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

    template<std::size_t length> void enable(Resolution a_resolution, const std::array<Channel, length>& a_channels)
    {
        this->enable(a_resolution, a_channels.data(), a_channels.size());
    }
    template<std::size_t length>
    bool enable(Resolution a_resolution, const std::array<Channel, length>& a_channels, Milliseconds a_timeout)
    {
        return this->enable(a_resolution, a_channels.data(), a_channels.size(), a_timeout);
    }

    void disable();

    Resolution get_resolution() const;
    std::array<std::pair<Channel::Id, bool>, s::max_channels_count> get_enabled_channels();

    constexpr Calibration_data get_calibration_data() const
    {
        return { *(reinterpret_cast<const std::uint16_t*>(0x1FFF75A8)),
                 *(reinterpret_cast<const std::uint16_t*>(0x1FFF75CA)),
                 *(reinterpret_cast<const std::uint16_t*>(0x1FFF75AA)) };
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

    void enable(ADC::Resolution a_resolution, const ADC::Channel* a_p_channels, std::size_t a_channels_count);
    bool enable(ADC::Resolution a_resolution,
                const ADC::Channel* a_p_channels,
                std::size_t a_channels_count,
                Milliseconds a_timeout);

    std::uint32_t idx;
    ADC_TypeDef* p_registers;

    IRQn_Type irqn;
    Interrupt::Callback callback;

    template<typename Periph_t, std::size_t id> friend class soc::peripheral;
    friend void ADC_interrupt_handler(ADC* a_p_this);
};
void ADC_interrupt_handler(ADC* a_p_this);

template<>
void ADC::Polling::read<ADC::Mode::single>(Not_null<uint16_t*> a_p_buffer, std::size_t a_buffer_capacity);
template<>
void ADC::Polling::read<ADC::Mode::continuous>(Not_null<uint16_t*> a_p_buffer, std::size_t a_buffer_capacity);
template<> void ADC::Polling::read<ADC::Mode::discontinuous>(Not_null<uint16_t*> a_p_buffer,
                                                             std::size_t a_buffer_capacity,
                                                             std::size_t a_group_size);

template<> bool ADC::Polling::read<ADC::Mode::single>(Not_null<uint16_t*> a_p_buffer,
                                                      std::size_t a_buffer_capacity,
                                                      Milliseconds a_timeout);
template<> bool ADC::Polling::read<ADC::Mode::continuous>(Not_null<uint16_t*> a_p_buffer,
                                                          std::size_t a_buffer_capacity,
                                                          Milliseconds a_timeout);
template<> bool ADC::Polling::read<ADC::Mode::discontinuous>(Not_null<uint16_t*> a_p_buffer,
                                                             std::size_t a_buffer_capacity,
                                                             std::size_t a_group_size,
                                                             Milliseconds a_timeout);

template<> void ADC::Interrupt::read_start<ADC::Mode::single>(const Callback& a_callback);
template<> void ADC::Interrupt::read_start<ADC::Mode::continuous>(const Callback& a_callback);
template<>
void ADC::Interrupt::read_start<ADC::Mode::discontinuous>(const Callback& a_callback, std::size_t a_group_size);
} // namespace peripherals
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
template<> class rcc<peripherals::ADC> : private Non_constructible
{
public:
    struct sync : private Non_constructible
    {
        enum class Prescaler : std::uint32_t
        {
            _1 = ADC_CCR_CKMODE_0,
            _2 = ADC_CCR_CKMODE_1,
            _4 = ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1
        };

        template<typename Source_t> static void enable(Prescaler a_prescaler, bool a_enable_in_lp) = delete;
        static void disable()
        {
            bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE_Msk | ADC_CCR_PRESC_Msk);
            bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_ADCSEL_Msk);
            bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
        }
    };

    struct async : private Non_constructible
    {
        enum class Prescaler : std::uint32_t
        {
            _1 = 0x0u,
            _2 = ADC_CCR_PRESC_0,
            _4 = ADC_CCR_PRESC_1,
            _6 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1,
            _8 = ADC_CCR_PRESC_2,
            _10 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_2,
            _12 = ADC_CCR_PRESC_1 | ADC_CCR_PRESC_2,
            _16 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_2,
            _32 = ADC_CCR_PRESC_3,
            _64 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_3,
            _128 = ADC_CCR_PRESC_1 | ADC_CCR_PRESC_3,
            _256 = ADC_CCR_PRESC_0 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_3
        };

        template<typename Source_t> static void enable(Prescaler a_prescaler, bool a_enable_in_lp) = delete;
        static void disable()
        {
            bit_flag::clear(&(ADC1_COMMON->CCR), ADC_CCR_CKMODE_Msk | ADC_CCR_PRESC_Msk);
            bit_flag::clear(&(RCC->CCIPR), RCC_CCIPR_ADCSEL_Msk);
            bit_flag::clear(&(RCC->AHB2ENR), RCC_AHB2ENR_ADCEN);
        }
    };
};

template<> void rcc<peripherals::ADC>::async::enable<rcc<system::mcu<1u>>>(Prescaler a_prescaler, bool a_enable_in_lp);
template<> void rcc<peripherals::ADC>::async::enable<sources::pll::sai1::r>(Prescaler a_prescaler, bool a_enable_in_lp);
template<> void rcc<peripherals::ADC>::async::enable<sources::pll::p>(Prescaler a_prescaler, bool a_enable_in_lp);
template<>
void rcc<peripherals::ADC>::sync::enable<rcc<system::mcu<1u>>::hclk<1u>>(Prescaler a_prescaler, bool a_enable_in_lp);
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
template<> class peripheral<m4::stm32wb::peripherals::ADC, 1u> : private Non_constructible
{
public:
    static m4::stm32wb::peripherals::ADC create()
    {
        return m4::stm32wb::peripherals::ADC(0U, ADC1, IRQn_Type::ADC1_IRQn);
    }
};
} // namespace soc
} // namespace xmcu
