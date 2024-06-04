#pragma once

/**/

// std
#include <cstdint>

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Limited.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/Not_null.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/various.hpp>
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m4/nvic.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {
class internal_flash : private Non_constructible
{
public:
    using Word = std::uint64_t;
    using Function_lock = void (*)(bool a_lock);

    struct s : private Non_constructible
    {
        static constexpr std::size_t start = FLASH_BASE;
        static constexpr std::size_t page_size_in_bytes = 4096u;

#if defined(STM32WB55CG) || defined(STM32WB55RG) || defined(STM32WB55VG)
        static constexpr std::size_t pages_count = 256u;
#elif defined(STM32WB55CE) || defined(STM32WB55RE) || defined(STM32WB55VE) || defined(STM32WB35CE)
        static constexpr std::size_t pages_count = 128u;
#elif defined(STM32WB55CC) || defined(STM32WB55RC) || defined(STM32WB55VC)
        static constexpr std::size_t pages_count = 64u;
#else
        static_assert(false, "Unkown MCU model");
#endif
        static constexpr std::size_t size_in_bytes = page_size_in_bytes * pages_count;
    };

    enum class Latency : std::uint32_t
    {
        _0 = FLASH_ACR_LATENCY_0WS,
        _1 = FLASH_ACR_LATENCY_1WS,
        _2 = FLASH_ACR_LATENCY_2WS,
        _3 = FLASH_ACR_LATENCY_3WS
    };

    enum class Cache_mode_flag : std::uint32_t
    {
        disabled     = 0x0u,
        data         = FLASH_ACR_DCEN,
        instructions = FLASH_ACR_ICEN,
        prefetech    = FLASH_ACR_PRFTEN
    };

    enum class Status_flag : std::uint32_t
    {
        ok                                = 0x0u,
        operation_error                   = FLASH_SR_OPERR,
        read_protection_error             = FLASH_SR_RDERR,
        write_protection_error            = FLASH_SR_WRPERR,
        size_error                        = FLASH_SR_SIZERR,
        programming_sequential_error      = FLASH_SR_PROGERR,
        programming_aligment_error        = FLASH_SR_PGAERR,
        programming_sequence_error        = FLASH_SR_PGSERR,
        data_miss_during_fast_programming = FLASH_SR_MISERR,
        fast_programming_error            = FLASH_SR_FASTERR,
        locked                            = 0x80000000
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

        static Result erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index,
                                 Function_lock a_lock_function);
        static Result erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index,
                                 Function_lock a_lock_function,
                                 Milliseconds a_timeout);

        static Result erase_bank(Bank_id a_id, Function_lock a_lock_function);
        static Result erase_bank(Bank_id a_id, Function_lock a_lock_function, Milliseconds a_timeout);
    };

    static void set_latency(Latency a_latency);
    static bool set_latency(Latency a_latency, Milliseconds a_timeout);

    static void set_cache_mode(Cache_mode_flag a_cache_mode);
    static bool set_cache_mode(Cache_mode_flag a_cache_mode, Milliseconds a_timeout);

    static Cache_mode_flag get_cache_mode()
    {
        return static_cast<Cache_mode_flag>(bit_flag::get(FLASH->ACR, FLASH_ACR_DCEN | FLASH_ACR_ICEN));
    }

    static Latency get_latency()
    {
        return static_cast<Latency>(bit_flag::get(FLASH->ACR, FLASH_ACR_LATENCY));
    }
};
} // namespace peripherals
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
template<> class Scoped_guard<m4::stm32wb::peripherals::internal_flash::unlocker> : private Non_copyable
{
public:
    Scoped_guard()
        : unlocked(false)
    {
        m4::stm32wb::peripherals::internal_flash::unlocker::unlock();
        this->unlocked = true;
    }

    Scoped_guard(Milliseconds a_timeout)
        : unlocked(m4::stm32wb::peripherals::internal_flash::unlocker::unlock(a_timeout))
    {
    }

    ~Scoped_guard()
    {
        m4::stm32wb::peripherals::internal_flash::unlocker::lock();
    }

    bool is_unlocked() const
    {
        return this->unlocked;
    }

private:
    bool unlocked;
};
template<> class Scoped_guard<m4::stm32wb::peripherals::internal_flash::cache_disabler> : private Non_copyable
{
public:
    Scoped_guard()
        : disabled(false)
    {
        m4::stm32wb::peripherals::internal_flash::cache_disabler::disable();
        this->disabled = true;
    }

    Scoped_guard(Milliseconds a_timeout)
        : disabled(m4::stm32wb::peripherals::internal_flash::cache_disabler::disable(a_timeout))
    {
    }

    ~Scoped_guard()
    {
        m4::stm32wb::peripherals::internal_flash::cache_disabler::enable();
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