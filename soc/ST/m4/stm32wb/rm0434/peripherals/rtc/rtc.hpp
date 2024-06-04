#pragma once

/**/

#include <xmcu/Duration.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/rcc.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/hse.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lse.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/sources/lsi.hpp>

namespace xmcu::soc::m4::stm32wb::peripherals {

class backup_domain : Non_constructible
{
};

class rtc : Non_constructible
{
public:
    enum class Alarm_id
    {
        A,
        B
    };

    enum Alarm_mask : std::uint32_t
    {
        seconds = 1 << 0,
        minutes = 1 << 1,
        hours = 1 << 2,
        days = 1 << 3,
    };

    struct Alarm_handler
    {
        using Function = void (*)(Alarm_id a_id, void* a_data);
        Function function;
        void* p_data;
    };

    /**
     * @brief Sets the RTC time to a given time, in milliseconds.
     *
     * The time will be set to a rounded-down second precision.
     *
     * @param a_world_millis Time to be set, in milliseconds.
     */
    static void set_clock(Milliseconds a_world_millis);

    /**
     * @brief Checks if RTC is set up.
     *
     * Useful to check if RTC holds configuration from before CPU1 reboot.
     *
     * @return True if RTC is already configured, otherwise false.
     */
    static bool is_clock_set();

    /**
     * @brief Reads the RTC time in a synchronized way with RTC's 1Hz counter.
     *
     * This is not to be used near sleep/stop modes.
     *
     * @return RTC time, rounded down to a second, in milliseonds.
     */
    static Milliseconds get_time();

    static uint32_t read_bkp_register(std::size_t a_index);

    static void write_bkp_register(std::size_t a_index, uint32_t a_value);

    static void
    enable_alarm(Alarm_id a_id, Alarm_mask a_mask, Milliseconds a_world_millis, Alarm_handler a_handler);

    static void disable_alarm(Alarm_id a_id);

    static bool is_alarm_triggered(Alarm_id a_id);

private:
    static void wait_for_sync();
};

constexpr rtc::Alarm_mask operator|(rtc::Alarm_mask a_f1, rtc::Alarm_mask a_f2)
{
    return static_cast<rtc::Alarm_mask>(static_cast<std::uint32_t>(a_f1) | static_cast<std::uint32_t>(a_f2));
}

} // namespace xmcu::soc::m4::stm32wb::peripherals

namespace xmcu::soc::m4::stm32wb {

template<> class rcc<peripherals::rtc> : private Non_constructible
{
public:
    template<typename Source_t> static void enable(bool a_enable_in_lp) = delete;

    static void disable();

    static bool is_enabled();
};

template<> void rcc<peripherals::rtc>::enable<sources::lsi>(bool a_enable_in_lp);
template<> void rcc<peripherals::rtc>::enable<sources::lse>(bool a_enable_in_lp);
template<> void rcc<peripherals::rtc>::enable<sources::hse>(bool a_enable_in_lp);

} // namespace xmcu::soc::m4::stm32wb

namespace xmcu {
namespace soc {
template<> class Scoped_guard<m4::stm32wb::peripherals::backup_domain> : public Non_copyable
{
public:
    Scoped_guard()
    {
        // Disable write protection of backup domain peripherals
        bit_flag::set(&PWR->CR1, PWR_CR1_DBP);
    }

    ~Scoped_guard()
    {
        // Re-enable write-protection of backup domain peripherals
        bit_flag::clear(&PWR->CR1, PWR_CR1_DBP);
    }
};

template<> class Scoped_guard<m4::stm32wb::peripherals::rtc> : private Non_copyable
{
public:
    Scoped_guard()
    {
        // Disable write protection of RTC registers
        RTC->WPR = 0xCA;
        RTC->WPR = 0x53;
    }

    ~Scoped_guard()
    {
        // Re-enable write-protection of RTC registers
        RTC->WPR = 0xFF;
    }

private:
    Scoped_guard<m4::stm32wb::peripherals::backup_domain> bd_guard; // Must be unlocked prior to unlocking RTC
};

} // namespace soc
} // namespace xmcu
