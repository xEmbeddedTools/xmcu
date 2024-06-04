#pragma once

/**/

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/Non_constructible.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/soc/Scoped_guard.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {
class option_bytes : private Non_constructible
{
public:
    class unlocker : private Non_constructible
    {
    public:
        static void unlock();
        static bool unlock(Milliseconds a_timeout);

        static void lock();
    };

    struct secure_flash : private Non_constructible
    {
        static std::uint32_t get_start_address();
        static std::uint32_t get_start_address(Milliseconds a_timeout);
    };

    struct BOR : private Non_constructible
    {
        enum class Level : std::uint32_t
        {
            _1_7V = 0x0u,
            _2_0V = FLASH_OPTR_BOR_LEV_0,
            _2_2V = FLASH_OPTR_BOR_LEV_1,
            _2_5V = FLASH_OPTR_BOR_LEV_0 | FLASH_OPTR_BOR_LEV_1,
            _2_8V = FLASH_OPTR_BOR_LEV_2
        };

        static bool set(Level a_level);
        static Level get();
    };
};
} // namespace peripherals
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

namespace xmcu {
namespace soc {
template<> class Scoped_guard<m4::stm32wb::peripherals::option_bytes::unlocker> : private Non_copyable
{
public:
    Scoped_guard()
        : unlocked(false)
    {
        m4::stm32wb::peripherals::option_bytes::unlocker::unlock();
        this->unlocked = true;
    }

    Scoped_guard(Milliseconds a_timeout)
        : unlocked(m4::stm32wb::peripherals::option_bytes::unlocker::unlock(a_timeout))
    {
    }

    ~Scoped_guard()
    {
        m4::stm32wb::peripherals::option_bytes::unlocker::lock();
    }

    bool is_unlocked() const
    {
        return this->unlocked;
    }

private:
    bool unlocked;
};
} // namespace soc
} // namespace xmcu