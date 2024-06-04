#pragma once

/**/

// std
#include <cstdint>

// externals
#include <stm32l0xx.h>

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/Not_null.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/various.hpp>
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m0/nvic.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/defs.hpp>

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace peripherals {

class internal_flash : private Non_constructible
{
public:
    using Word = std::uint64_t;

    struct s : private Non_constructible
    {
        static constexpr std::size_t start              = FLASH_BASE;
        static constexpr std::size_t page_size_in_bytes = 128u;

#if STM32L010XX_CATEGORY == 1
        static constexpr std::size_t pages_count = 128u;
#elif STM32L010XX_CATEGORY == 2
        static constexpr std::size_t pages_count = 256u;
#elif STM32L010XX_CATEGORY == 3
        static_assert(false,
                      "TODO: For 32 Kbyte category 3 devices, the Flash program memory is divided into 256 pages of "
                      "128 bytes each. Which ones?");
        static constexpr std::size_t pages_count = 512u;
#elif STM32L010XX_CATEGORY == 5
        static constexpr std::size_t pages_count = 1024u;
#else
        static_assert(false, "Unkown MCU model");
#endif
        static constexpr std::size_t size_in_bytes = page_size_in_bytes * pages_count;
    };

    enum class Latency : std::uint32_t
    {
        _0 = 0 << FLASH_ACR_LATENCY_Pos,
        _1 = 1 << FLASH_ACR_LATENCY_Pos,
    };

    enum class Cache_mode_flag : std::uint32_t
    {
        disabled  = 0x0u,
        prefetech = FLASH_ACR_PRFTEN
    };

    enum class Status_flag : std::uint32_t
    {
        ok                         = 0x0u,
        read_protection_error      = FLASH_SR_RDERR,
        write_protection_error     = FLASH_SR_WRPERR,
        size_error                 = FLASH_SR_SIZERR,
        programming_aligment_error = FLASH_SR_PGAERR,
        fetch_wr_abort_error       = FLASH_SR_FWWERR,
        overwrite_not_zero_error   = FLASH_SR_NOTZEROERR,
        option_bytes_load_error    = FLASH_SR_OPTVERR,
        locked                     = 0x80000000
    };

    class unlocker : private Non_constructible
    {
    public:
        static void unlock();
        static bool unlock(Milliseconds a_timeout);

        static void lock();
    };
    class cache_disabler : private Non_constructible
    {
    public:
        static void disable();
        static bool disable(Milliseconds a_timeout);

        static void enable();
        static bool enable(Milliseconds a_timeout);

    private:
        static inline Cache_mode_flag cache_mode = various::get_enum_incorrect_value<Cache_mode_flag>();
    };

    class polling : private Non_constructible
    {
    public:
        enum class Bank_id : std::uint32_t
        {
            _0
        };

        struct Result
        {
            Status_flag status = various::get_enum_incorrect_value<Status_flag>();
            std::size_t words  = 0;
        };

        static Result write(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                            Not_null<const Word*> a_p_data,
                            std::size_t a_size_in_double_words);
        static Result write(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                            Not_null<const Word*> a_p_data,
                            std::size_t a_size_in_double_words,
                            Milliseconds a_timeout);

        static Result read(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                           Not_null<void*> a_p_data,
                           Limited<std::size_t, 1, s::page_size_in_bytes> a_size_in_bytes);
        static Result read(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                           Not_null<void*> a_p_data,
                           Limited<std::size_t, 1, s::page_size_in_bytes> a_size_in_bytes,
                           Milliseconds a_timeout);

        static Result erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index);
        static Result erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index,
                                 Milliseconds a_timeout);

        static Result erase_bank(Bank_id a_id);
        static Result erase_bank(Bank_id a_id, Milliseconds a_timeout);
    };

    static void set_latency(Latency a_latency);
    static bool set_latency(Latency a_latency, Milliseconds a_timeout);

    static void set_cache_mode(Cache_mode_flag a_cache_mode);
    static bool set_cache_mode(Cache_mode_flag a_cache_mode, Milliseconds a_timeout);

    static Cache_mode_flag get_cache_mode()
    {
        return static_cast<Cache_mode_flag>(bit_flag::get(FLASH->ACR, FLASH_ACR_PRFTEN));
    }

    static Latency get_latency()
    {
        return static_cast<Latency>(bit_flag::get(FLASH->ACR, FLASH_ACR_LATENCY));
    }
};

} // namespace peripherals
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
template<> class Scoped_guard<m0::stm32l0::rm0451::peripherals::internal_flash::unlocker> : private Non_copyable
{
public:
    Scoped_guard()
        : unlocked(false)
    {
        m0::stm32l0::rm0451::peripherals::internal_flash::unlocker::unlock();
        this->unlocked = true;
    }

    Scoped_guard(Milliseconds a_timeout)
        : unlocked(m0::stm32l0::rm0451::peripherals::internal_flash::unlocker::unlock(a_timeout))
    {
    }

    ~Scoped_guard()
    {
        m0::stm32l0::rm0451::peripherals::internal_flash::unlocker::lock();
    }

    bool is_unlocked() const
    {
        return this->unlocked;
    }

private:
    bool unlocked;
};
template<> class Scoped_guard<m0::stm32l0::rm0451::peripherals::internal_flash::cache_disabler>
    : private Non_copyable
{
public:
    Scoped_guard()
        : disabled(false)
    {
        m0::stm32l0::rm0451::peripherals::internal_flash::cache_disabler::disable();
        this->disabled = true;
    }

    Scoped_guard(Milliseconds a_timeout)
        : disabled(m0::stm32l0::rm0451::peripherals::internal_flash::cache_disabler::disable(a_timeout))
    {
    }

    ~Scoped_guard()
    {
        m0::stm32l0::rm0451::peripherals::internal_flash::cache_disabler::enable();
    }

    bool is_disabled() const
    {
        return this->disabled;
    }

private:
    bool disabled;
};
} // namespace soc
} // namespace xmcu
