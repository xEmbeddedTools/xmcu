#pragma once

/**/

// std
#include <type_traits>
#include <utility>

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/various.hpp>
#include <xmcu/soc/ST/m4/IRQ_config.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/GPIO/GPIO.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/rcc.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hsi16.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lse.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lsi.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/mcu/mcu.hpp>
#include <xmcu/soc/peripheral.hpp>

// small classes & enums
namespace xmcu::soc::m4::stm32wb::peripherals::timer {

enum class Count_Mode : std::uint32_t
{
    up = 0,
    down = TIM_CR1_DIR,
    center_on_down = TIM_CR1_CMS_0,
    center_on_up = TIM_CR1_CMS_1,
    center_on_both = TIM_CR1_CMS,
};

enum class Mode : std::uint32_t
{
    continuous = 0,
    one_pulse = TIM_CR1_OPM,
};

enum class Tmr_divider : std::uint32_t
{
    _1 = 0u,
    _2 = TIM_CR1_CKD_0,
    _4 = TIM_CR1_CKD_1,
};

enum class TIM_irq : std::uint32_t
{
    TIM1_BRK,
    TIM1_UP,
    TIM16_IRQ,
    TIM1_TRG,
    TIM17_IRQ,
    TIM1_CC,
    TIM2_IRQ,
    count
};

struct Callback
{
    using Function = void (*)(void* a_p_user_data);

    Function function = nullptr;
    void* p_user_data = nullptr;
};

class Interrupt : private xmcu::Non_copyable
{
protected:
    Callback callback;
    const std::uint32_t idx;

public:
    void enable(const IRQ_config& a_config);
    void disable();

    void register_callback(const Callback& a_callback);
    void unregister_callback();

    inline void handle(TIM_TypeDef* a_driver, std::uint32_t a_sr)
    {
        hkm_assert(nullptr != this->callback.function);
        this->callback.function(this->callback.p_user_data);
    }

private:
    Interrupt(TIM_irq a_irq)
        : idx { static_cast<std::uint32_t>(a_irq) }
    {
    }
    friend class Tim_general;
    friend class Tim_advanced;
};

class Polling : private xmcu::Non_copyable
{
protected:
    TIM_TypeDef* const p_registers;

public:
    bool is_overload() const;
    std::uint32_t get_counter() const
    {
        return this->p_registers->CNT;
    }

private:
    Polling(TIM_TypeDef* a_p_driver)
        : p_registers { a_p_driver }
    {
    }
    Polling(Polling&&) = default;
    friend class Tim_counter;
};

} // namespace xmcu::soc::m4::stm32wb::peripherals::timer

namespace xmcu::soc::m4::stm32wb::peripherals {
class TIM_ADV;
} // namespace xmcu::soc::m4::stm32wb::peripherals

namespace xmcu::soc::m4::stm32wb::peripherals::timer::helper {
static constexpr std::uint32_t convert_compare_mode(std::uint32_t a_in)
{
    return (0x7 & a_in) << TIM_CCMR1_OC1M_Pos | (0x8 & a_in ? TIM_CCMR1_OC1M_3 : 0);
}
} // namespace xmcu::soc::m4::stm32wb::peripherals::timer::helper
// Tim_counter & derived
namespace xmcu::soc::m4::stm32wb::peripherals::timer {

class Tim_counter : private xmcu::Non_copyable
{
protected:
    TIM_TypeDef* const p_registers;
    Tim_counter(TIM_TypeDef* a_p_driver);
    void start(Mode a_mode) const;
    template<typename T> void start(Mode a_mode, T a_arr) const
    {
        this->stop();
        this->p_registers->ARR = a_arr;
        this->start(a_mode);
    }
    void stop() const;

public:
    using Tmr_divider = timer::Tmr_divider;
    using Count_Mode = timer::Count_Mode;
    using Start_Mode = timer::Mode;
    const Polling polling;
    ~Tim_counter();
    void enable(Tmr_divider a_div, Count_Mode a_mode, std::uint16_t a_psc) const;
    void disable() const;
    bool is_enabled() const
    {
        return bit_flag::is(this->p_registers->CR1, TIM_CR1_CEN); // naive
    }
};

class Tim_general : public Tim_counter
{
public:
    using TIM_irq = timer::TIM_irq;

protected:
    Tim_general(TIM_TypeDef* a_p_driver, TIM_irq a_irq_n)
        : Tim_counter(a_p_driver)
        , interrupt { a_irq_n }
    {
    }
    // used by channels drivers:
    template<typename, std::size_t> friend class Channel_PWM;
    static constexpr volatile std::uint32_t* get_CCMR_ptr(std::uint32_t idx, TIM_TypeDef* a_p_registers)
    {
        switch (idx / 2)
        {
            case 0:
                return &a_p_registers->CCMR1;
            case 1:
                return &a_p_registers->CCMR2;
            default:
                hkm_assert(false);
                return 0;
        }
    }
    static constexpr volatile std::uint32_t* get_CCR_ptr(std::uint32_t idx, TIM_TypeDef* a_p_registers)
    {
        return &a_p_registers->CCR1 + idx;
    }
    Interrupt interrupt;
};

class Tim_general16 : public Tim_general
{
public:
    using Counter_word_t = std::uint16_t;
    void start(Mode a_mode, Counter_word_t a_arr) const
    {
        return Tim_counter::start(a_mode, a_arr);
    }
    Tim_general16(TIM_TypeDef* a_p_driver, TIM_irq a_irq_n)
        : Tim_general { a_p_driver, a_irq_n }
    {
    }

    friend class peripherals::TIM_ADV;
    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
};

class Tim_advanced : public Tim_counter
{
    Interrupt interrupt_up, interrupt_break, interrupt_trigger;
    // capture compare is separete from counter?
public:
    using Counter_word_t = std::uint16_t;
    using Repetiton_word_t = std::uint16_t;
    struct TIM_irq_t
    {
        m4::stm32wb::peripherals::timer::TIM_irq brk, cc, trg, up;
    };
    void start(Mode a_mode, Counter_word_t a_arr) const;
    void start(Mode a_mode, Counter_word_t a_arr, Repetiton_word_t a_rcr) const
    {
        this->p_registers->RCR = a_rcr;
        return start(a_mode, a_arr);
    }
    void stop() const;
    void enable(Tmr_divider a_div, Count_Mode a_mode, std::uint16_t a_psc) const;

private:
    Tim_advanced(TIM_TypeDef* a_p_driver, const TIM_irq_t& a_irq)
        : Tim_counter(a_p_driver)
        , interrupt_up { a_irq.up }
        , interrupt_break { a_irq.brk }
        , interrupt_trigger { a_irq.trg }
    {
    }
    friend class peripherals::TIM_ADV;
    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
    // used by channels drivers:
    template<typename, std::size_t> friend class Channel_PWM;
    static constexpr volatile std::uint32_t* get_CCMR_ptr(std::uint32_t idx, TIM_TypeDef* a_p_registers)
    {
        switch (idx / 2)
        {
            case 0:
                return &a_p_registers->CCMR1;
            case 1:
                return &a_p_registers->CCMR2;
            case 2:
                return &a_p_registers->CCMR3;
            default:
                hkm_assert(false);
                return 0;
        }
    }

    static constexpr volatile std::uint32_t* get_CCR_ptr(std::uint32_t idx, TIM_TypeDef* a_p_registers)
    {
        switch (idx)
        {
            case 0:
                return &a_p_registers->CCR1;
            case 1:
                return &a_p_registers->CCR2;
            case 2:
                return &a_p_registers->CCR3;
            case 3:
                return &a_p_registers->CCR4;
            case 4: // discontinuity in the register map
                return &a_p_registers->CCR5;
            case 5:
                return &a_p_registers->CCR6;
            default:
                hkm_assert(false);
                return 0;
        }
    }
};

/**
 * @brief class intorduced to simplify specialization of peripherals::GPIO::Alternate_function::enable
 * Additonaly internal types declared in template class isn't good idea
 * std::is_same<T<1>::U, T<2>::U> is false...
 */
class Channel
{
protected:
    static constexpr std::uint32_t output_config_mask = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NE | TIM_CCER_CC1NP;
    static constexpr std::uint32_t set_compare_mask =
        TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1FE_Msk | TIM_CCMR1_OC1PE_Msk | TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC1CE_Msk;

public:
    using Base_Channel = Channel;
    enum class Out_config
    {
        disabled,
        active_h = TIM_CCER_CC1E,
        active_l = TIM_CCER_CC1E | TIM_CCER_CC1P,
    };

    enum class Compare_mode
    {
        frozen = helper::convert_compare_mode(0b0000),                   // Frozen
        on_match_activate = helper::convert_compare_mode(0b0001),        // Set channel 1 to active level on match
        on_match_inactive = helper::convert_compare_mode(0b0010),        // Set channel 1 to inactive level on match
        toggle = helper::convert_compare_mode(0b0011),                   // Toggle
        force_inactive = helper::convert_compare_mode(0b0100),           // Force inactive level
        force_active_level = helper::convert_compare_mode(0b0101),       // Force active level
        pwm_mode_1 = helper::convert_compare_mode(0b0110),               // PWM mode 1
        pwm_mode_2 = helper::convert_compare_mode(0b0111),               // PWM mode 2
        retriggerable_opm_mode_1 = helper::convert_compare_mode(0b1000), // Retriggerable OPM mode 1
        retriggerable_opm_mode_2 = helper::convert_compare_mode(0b1001), // Retriggerable OPM mode 2
        combined_pwm_mode_1 = helper::convert_compare_mode(0b1100),      // Combined PWM mode 1
        combined_pwm_mode_2 = helper::convert_compare_mode(0b1101),      // Combined PWM mode 2
        asymmetric_pwm_mode_1 = helper::convert_compare_mode(0b1110),    // Asymmetric PWM mode 1
        asymmetric_pwm_mode_2 = helper::convert_compare_mode(0b1111),    // Asymmetric PWM mode 2
        // Reserved1             = helper::convert_compare_mode(0b1010), // Reserved,
        // Reserved2             = helper::convert_compare_mode(0b1011), // Reserved,
    };

    enum class Preload
    {
        disable = 0,
        enable = TIM_CCMR1_OC1PE
    };

    enum class Clear
    {
        disable = 0,
        enable = TIM_CCMR1_OC1CE
    };
};

class Channel_Input : public Channel
{
    // TODO: sometime or not... ;)
};

template<typename T, std::size_t N> class Channel_PWM : public Channel
{
    using Counter_base = T;
    Counter_base* const p_parent;
    TIM_TypeDef* const p_registers;
    const std::uint8_t idx;

public:
    using Channels_Limit = Limited<decltype(idx), 0, N>;
    Channel_PWM(Channels_Limit&& a_idx, Counter_base* const a_timer)
        : p_parent { a_timer }
        , p_registers { a_timer->p_registers }
        , idx { a_idx }
    {
    }

    /// @brief Configure output mode of channel
    /// @param a_cm These bits define the behavior of the output reference signal
    /// @param a_preload Defines how chanes of CCR are applied. Disable: new value is taken immediately. Enable value is
    /// loaded at update event.
    /// @param a_clear Defines if output is cleared as soon as a High level is detected on ocref_clr_int signal
    void set_compare_mode(Compare_mode a_cm, Preload a_preload = Preload::disable, Clear a_clear = Clear::disable)
    {
        std::uint32_t config =
            // TIM_CCMR1_CC1S S is 0, output
            // TIM_CCMR1_OC1FE fast enable not used
            static_cast<std::uint32_t>(a_clear) | static_cast<std::uint32_t>(a_preload) |
            static_cast<std::uint32_t>(a_cm);
        std::uint32_t shift = (this->idx & 1) * TIM_CCMR1_CC2S_Pos;
        volatile std::uint32_t* ccmr_ptr = Counter_base::get_CCMR_ptr(this->idx, this->p_registers);
        bit_flag::set(ccmr_ptr, set_compare_mask << shift, config << shift);
    }

    void set_output_configuration(Out_config a_output, Out_config a_outputN = Out_config::disabled)
    {
        std::uint32_t config = static_cast<std::uint32_t>(a_output) | static_cast<std::uint32_t>(a_outputN)
                                                                          << TIM_CCER_CC1NE_Pos;
        std::uint32_t shift = TIM_CCER_CC2E_Pos * this->idx;
        bit_flag::set(&this->p_registers->CCER, output_config_mask << shift, config << shift);
    }

    void set_CCR(typename Counter_base::Counter_word_t ccr)
    {
        *Counter_base::get_CCR_ptr(this->idx, this->p_registers) = ccr;
    }
};

} // namespace xmcu::soc::m4::stm32wb::peripherals::timer

// TIM - main class
namespace xmcu::soc::m4::stm32wb::peripherals {
class TIM_ADV
{
    static constexpr std::uint32_t CH_N = 4;

public:
    using Tick_counter = timer::Tim_advanced;
    using Channel = timer::Channel_PWM<Tick_counter, CH_N>;

    Tick_counter tick_counter;
    Channel channels[CH_N];

    // compatibility with LPTIM:
    ~TIM_ADV()
    { // disable in this->tick_counter
    }
    void disable() const
    {
        this->tick_counter.disable();
    }
    bool is_enabled() const
    {
        return this->tick_counter.is_enabled();
    }

    void config_output_idle_state(GPIO::Level);

public:
    TIM_ADV(TIM_TypeDef* a_p_driver, const Tick_counter::TIM_irq_t& a_irq)
        : TIM_ADV(a_p_driver, a_irq, std::make_index_sequence<CH_N>())
    {
    }

private:
    // https://stackoverflow.com/questions/36233587/how-to-initialize-template-member-array-from-constructor-parameter
    template<std::size_t... Is>
    TIM_ADV(TIM_TypeDef* a_p_driver, const Tick_counter::TIM_irq_t& a_irq, std::index_sequence<Is...>)
        : tick_counter { a_p_driver, std::move(a_irq) }
        , channels { Channel(0 + Is, &this->tick_counter)... }
    {
    }

    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
};

class TIM_G16
{
public:
    using Tick_counter = timer::Tim_general16;
    using Channel = timer::Channel_PWM<Tick_counter, 1>;
    Tick_counter tick_counter;
    Channel channels[1];

    // compatibility with LPTIM:
    ~TIM_G16()
    { // disable in this->tick_counter
    }
    void disable() const
    {
        this->tick_counter.disable();
    }
    void is_enabled() const
    {
        this->tick_counter.is_enabled();
    }

private:
    TIM_G16(TIM_TypeDef* a_p_driver, const Tick_counter::TIM_irq& a_irq)
        : tick_counter { a_p_driver, std::move(a_irq) }
        , channels { Channel { 0, &this->tick_counter } }
    {
    }
    template<typename Periph_t, std::uint32_t id> friend class xmcu::soc::peripheral;
};

} // namespace xmcu::soc::m4::stm32wb::peripherals

// RCC - template specialization
namespace xmcu::soc::m4::stm32wb {

template<> class rcc<peripherals::TIM_ADV, 1u>
{
public:
    static void enable()
    {
        bit_flag::set(&RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
    }
    static void disable()
    {
        bit_flag::clear(&RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
    }
    static bool is_enabled()
    {
        return bit_flag::is(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
    }
    static std::uint32_t get_frequency_Hz()
    {
        return rcc<system::mcu<1u>>::pclk<2>::get_Tim_frequency_Hz();
    }
};

template<> class rcc<peripherals::TIM_G16, 16u>
{
public:
    static void enable()
    {
        bit_flag::set(&RCC->APB2ENR, RCC_APB2ENR_TIM16EN);
    }
    static void disable()
    {
        bit_flag::clear(&RCC->APB2ENR, RCC_APB2ENR_TIM16EN);
    }
    static bool is_enabled()
    {
        return bit_flag::is(RCC->APB2ENR, RCC_APB2ENR_TIM16EN);
    }
    static std::uint32_t get_frequency_Hz()
    {
        return rcc<system::mcu<1u>>::pclk<2>::get_Tim_frequency_Hz();
    }
};
template<> class rcc<peripherals::TIM_G16, 17u>
{
public:
    static void enable()
    {
        bit_flag::set(&RCC->APB2ENR, RCC_APB2ENR_TIM17EN);
    }
    static void disable()
    {
        bit_flag::clear(&RCC->APB2ENR, RCC_APB2ENR_TIM17EN);
    }
    static bool is_enabled()
    {
        return bit_flag::is(RCC->APB2ENR, RCC_APB2ENR_TIM17EN);
    }
    static std::uint32_t get_frequency_Hz()
    {
        return rcc<system::mcu<1u>>::pclk<2>::get_Tim_frequency_Hz();
    }
};

template<> inline void peripherals::GPIO::Alternate_function::enable<peripherals::timer::Channel, 1u>(
    Limited<std::uint32_t, 0, 15> a_id,
    const Enable_config& a_config,
    Pin* a_p_pin)
{
#if defined(STM32WB35xx) || defined(STM32WB55xx)
    hkm_assert((0 == this->p_port->idx && (7 == a_id || 8 == a_id || 9 == a_id || 10 == a_id || 11 == a_id)) ||
               (1 == this->p_port->idx && (8 == a_id || 9 == a_id)));
#endif
    this->enable(a_id, a_config, 0x1u, a_p_pin);
}

template<> inline void peripherals::GPIO::Alternate_function::enable<peripherals::timer::Channel, 16u>(
    Limited<std::uint32_t, 0, 15> a_id,
    const Enable_config& a_config,
    Pin* a_p_pin)
{
#if defined(STM32WB35xx) || defined(STM32WB55xx)
    hkm_assert((0 == this->p_port->idx && (6 == a_id)) || (1 == this->p_port->idx && (6 == a_id || 8 == a_id)));
#endif
    this->enable(a_id, a_config, 14u, a_p_pin);
}

template<> inline void peripherals::GPIO::Alternate_function::enable<peripherals::timer::Channel, 17u>(
    Limited<std::uint32_t, 0, 15u> a_id,
    const Enable_config& a_config,
    Pin* a_p_pin)
{
#if defined(STM32WB35xx) || defined(STM32WB55xx)
    hkm_assert((0 == this->p_port->idx && (7 == a_id)) || (1 == this->p_port->idx && (7 == a_id || 9 == a_id)));
#endif
    this->enable(a_id, a_config, 14u, a_p_pin);
}

} // namespace xmcu::soc::m4::stm32wb

// peripheral - template specialization
namespace xmcu {
namespace soc {

template<> class peripheral<m4::stm32wb::peripherals::TIM_ADV, 1u>
{
    static constexpr m4::stm32wb::peripherals::timer::Tim_advanced::TIM_irq_t irq {
        .brk = m4::stm32wb::peripherals::timer::TIM_irq::TIM1_BRK,
        .cc = m4::stm32wb::peripherals::timer::TIM_irq::TIM1_CC,
        .trg = m4::stm32wb::peripherals::timer::TIM_irq::TIM1_TRG,
        .up = m4::stm32wb::peripherals::timer::TIM_irq::TIM1_UP
    };

public:
    static m4::stm32wb::peripherals::TIM_ADV create()
    {
        return { TIM1, irq };
    }
};

template<> class peripheral<m4::stm32wb::peripherals::TIM_G16, 16u>
{
public:
    static m4::stm32wb::peripherals::TIM_G16 create()
    {
        return { TIM16, m4::stm32wb::peripherals::timer::TIM_irq::TIM16_IRQ };
    }
};

template<> class peripheral<m4::stm32wb::peripherals::TIM_G16, 17u>
{
public:
    static m4::stm32wb::peripherals::TIM_G16 create()
    {
        return { TIM17, m4::stm32wb::peripherals::timer::TIM_irq::TIM17_IRQ };
    }
};

} // namespace soc
} // namespace xmcu
