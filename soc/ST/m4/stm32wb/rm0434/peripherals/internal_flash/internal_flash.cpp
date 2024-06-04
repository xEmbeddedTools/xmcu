/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/internal_flash/internal_flash.hpp>

// std
#include <cstring>

// xmcu
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/hsem/hsem.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

// debug
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/delay.hpp>

namespace {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::peripherals;

void clear_FLASH_SR_errors()
{
    bit_flag::set(&(FLASH->SR),
                  FLASH_SR_OPERR | FLASH_SR_RDERR | FLASH_SR_WRPERR | FLASH_SR_SIZERR | FLASH_SR_PROGERR |
                      FLASH_SR_PGAERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);
}

bool is_FLASH_SR_error()
{
    return bit::is_any(FLASH->SR,
                       FLASH_SR_OPERR | FLASH_SR_RDERR | FLASH_SR_WRPERR | FLASH_SR_SIZERR | FLASH_SR_PROGERR |
                           FLASH_SR_PGAERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR);
}

internal_flash::Status_flag get_status_flag_from_FLASH_SR()
{
    uint32_t SR = (FLASH->SR & 0x3F8u);

    if (0x0u == SR && bit_flag::is(FLASH->SR, FLASH_SR_BSY))
    {
        return internal_flash::Status_flag::locked;
    }

    return static_cast<internal_flash::Status_flag>(SR);
}
} // namespace

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace peripherals {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::system;
using namespace utils;

void internal_flash::unlocker::unlock()
{
    wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

    if (true == bit_flag::is(FLASH->CR, FLASH_CR_LOCK))
    {
        Scoped_guard<nvic> interrupt_guard;

        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }

    wait_until::all_bits_are_cleared(FLASH->CR, FLASH_CR_LOCK);
}
bool internal_flash::unlocker::unlock(Milliseconds a_timeout)
{
    bool ret = wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY, a_timeout);

    if (true == ret && true == bit_flag::is(FLASH->CR, FLASH_CR_LOCK))
    {
        Scoped_guard<nvic> interrupt_guard;

        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }

    return false == bit_flag::is(FLASH->CR, FLASH_CR_LOCK);
}
void internal_flash::unlocker::lock()
{
    bit_flag::set(&(FLASH->CR), FLASH_CR_LOCK);
}

void internal_flash::cache_disabler::disable()
{
    cache_mode = get_cache_mode();
    set_cache_mode(internal_flash::Cache_mode_flag::disabled);
}
bool internal_flash::cache_disabler::disable(Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    cache_mode = get_cache_mode();
    return set_cache_mode(internal_flash::Cache_mode_flag::disabled,
                          a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
}
void internal_flash::cache_disabler::enable()
{
    set_cache_mode(cache_mode);
}
bool internal_flash::cache_disabler::enable(Milliseconds a_timeout)
{
    return set_cache_mode(cache_mode, a_timeout);
}

void internal_flash::set_latency(Latency a_latency)
{
    Scoped_guard<hsem::_1_step> sem2_guard(0x2u);

    bit_flag::set(&(FLASH->ACR), FLASH_ACR_LATENCY, static_cast<std::uint32_t>(a_latency));
    wait_until::all_bits_are_set(FLASH->ACR, static_cast<std::uint32_t>(a_latency));
}
bool internal_flash::set_latency(Latency a_latency, Milliseconds a_timeout)
{
    Scoped_guard<hsem::_1_step> sem2_guard(0x2u, a_timeout);

    if (true == sem2_guard.is_locked())
    {
        bit_flag::set(&(FLASH->ACR), FLASH_ACR_LATENCY, static_cast<std::uint32_t>(a_latency));
        wait_until::all_bits_are_set(FLASH->ACR, static_cast<std::uint32_t>(a_latency));

        return true;
    }

    return false;
}

void internal_flash::set_cache_mode(Cache_mode_flag a_cache_mode)
{
    Scoped_guard<hsem::_1_step> sem2_guard(0x2u);

    bit_flag::set(
        &(FLASH->ACR), FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN, static_cast<std::uint32_t>(a_cache_mode));
}
bool internal_flash::set_cache_mode(Cache_mode_flag a_cache_mode, Milliseconds a_timeout)
{
    Scoped_guard<hsem::_1_step> sem2_guard(0x2u, a_timeout);

    if (true == sem2_guard.is_locked())
    {
        bit_flag::set(&(FLASH->ACR),
                      FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN,
                      static_cast<std::uint32_t>(a_cache_mode));

        return true;
    }

    return false;
}

internal_flash::polling::Result
internal_flash::polling::write(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                               Not_null<const Word*> a_p_data,
                               std::size_t a_size_in_double_words)
{
    hkm_assert(a_size_in_double_words > 0);

    Scoped_guard<hsem::_1_step> sem2_guard(0x2u);
    Scoped_guard<unlocker> unlock_guard;

    if (true == is_FLASH_SR_error())
    {
        clear_FLASH_SR_errors();
    }

    for (std::size_t i = 0; i < a_size_in_double_words;)
    {
        wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY | FLASH_SR_PESD);

        Scoped_guard<nvic> interrupt_guard;

        if (false == hsem::is_locked(0x6u) && true == hsem::_1_step::try_lock(0x7u))
        {
            Scoped_guard<cache_disabler> cache_guard;

            bit_flag::set(&(FLASH->CR), FLASH_CR_PG);

            volatile std::uint32_t* p_address = reinterpret_cast<volatile std::uint32_t*>(a_address.get());

            *(p_address + i * 2u + 0u) = static_cast<std::uint32_t>(a_p_data[i] >> 0x00u);
            __ISB();
            *(p_address + i * 2u + 1u) = static_cast<std::uint32_t>(a_p_data[i] >> 0x20u);

            wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

            if (true == bit_flag::is(FLASH->SR, FLASH_SR_EOP))
            {
                bit_flag::set(&(FLASH->SR), FLASH_SR_EOP);
            }

            bit_flag::clear(&(FLASH->CR), FLASH_CR_PG);

            i++;

            hsem::_1_step::unlock(0x7u);
        }
    }

    return { get_status_flag_from_FLASH_SR(), a_size_in_double_words };
}

internal_flash::polling::Result
internal_flash::polling::write(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                               Not_null<const Word*> a_p_data,
                               std::size_t a_size_in_double_words,
                               Milliseconds a_timeout)
{
    hkm_assert(a_size_in_double_words > 0);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    Scoped_guard<hsem::_1_step> sem2_guard(0x2u, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
    if (true == sem2_guard.is_locked())
    {
        Scoped_guard<unlocker> unlock_guard(a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
        if (true == unlock_guard.is_unlocked())
        {
            if (true == is_FLASH_SR_error())
            {
                clear_FLASH_SR_errors();
            }

            bool timeout  = false;
            std::size_t i = 0;
            while (i < a_size_in_double_words && false == timeout)
            {
                timeout = false == wait_until::all_bits_are_cleared(FLASH->SR,
                                                                    FLASH_SR_BSY | FLASH_SR_PESD,
                                                                    a_timeout.get() -
                                                                        (tick_counter<Milliseconds>::get() - start));

                if (false == timeout)
                {
                    Scoped_guard<nvic> interrupt_guard;

                    if (false == hsem::is_locked(0x6u) && true == hsem::_1_step::try_lock(0x7u))
                    {
                        Scoped_guard<cache_disabler> cache_guard;

                        bit_flag::set(&(FLASH->CR), FLASH_CR_PG);

                        volatile std::uint32_t* p_address = reinterpret_cast<volatile std::uint32_t*>(a_address.get());

                        *(p_address + i * 2u + 0u) = static_cast<std::uint32_t>(a_p_data[i] >> 0x00u);
                        __ISB();
                        *(p_address + i * 2u + 1u) = static_cast<std::uint32_t>(a_p_data[i] >> 0x20u);

                        timeout =
                            false ==
                            wait_until::all_bits_are_cleared(
                                FLASH->SR, FLASH_SR_BSY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

                        if (false == timeout)
                        {
                            if (true == bit_flag::is(FLASH->SR, FLASH_SR_EOP))
                            {
                                bit_flag::set(&(FLASH->SR), FLASH_SR_EOP);
                            }

                            i++;
                        }

                        bit_flag::clear(&(FLASH->CR), FLASH_CR_PG);

                        hsem::_1_step::unlock(0x7u);
                    }
                    else
                    {
                        timeout = (tick_counter<Milliseconds>::get() - start >= a_timeout.get());
                    }
                }
            }
            return { get_status_flag_from_FLASH_SR(), i };
        }
        return { Status_flag::locked, 0x0u };
    }
    return { get_status_flag_from_FLASH_SR(), 0x0u };
}

internal_flash::polling::Result
internal_flash::polling::read(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                              Not_null<void*> a_p_data,
                              Limited<std::size_t, 1u, s::page_size_in_bytes> a_size_in_bytes)
{
    Scoped_guard<hsem::_1_step> sem2_guard(0x2u);
    Scoped_guard<unlocker> unlock_guard;

    if (true == unlock_guard.is_unlocked())
    {
        std::memcpy(a_p_data, reinterpret_cast<const uint8_t*>(a_address.get()), a_size_in_bytes);
    }

    return { get_status_flag_from_FLASH_SR(), a_size_in_bytes };
}

internal_flash::polling::Result
internal_flash::polling::read(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                              Not_null<void*> a_p_data,
                              Limited<std::size_t, 1, s::page_size_in_bytes> a_size_in_bytes,
                              Milliseconds a_timeout)
{
    hkm_assert(a_timeout > 0_ms);

    const std::uint64_t start = tick_counter<Milliseconds>::get();

    Scoped_guard<hsem::_1_step> sem2_guard(0x2u, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == sem2_guard.is_locked())
    {
        Scoped_guard<unlocker> unlock_guard(a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

        if (true == unlock_guard.is_unlocked())
        {
            std::memcpy(a_p_data, reinterpret_cast<const uint8_t*>(a_address.get()), a_size_in_bytes);
            return { get_status_flag_from_FLASH_SR(), a_size_in_bytes };
        }

        return { Status_flag::locked, 0x0u };
    }

    return { get_status_flag_from_FLASH_SR(), 0x0u };
}

internal_flash::polling::Result
internal_flash::polling::erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index,
                                    Function_lock a_external_lock_function)
{
    Scoped_guard<hsem::_1_step> sem2_guard(0x2u);
    Scoped_guard<unlocker> unlock_guard;

    if (true == is_FLASH_SR_error())
    {
        clear_FLASH_SR_errors();
    }

    if (nullptr != a_external_lock_function)
    {
        a_external_lock_function(true);
    }

    bool done = false;
    while (false == done)
    {
        wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY | FLASH_SR_PESD);

        Scoped_guard<nvic> interrupt_guard;

        if (false == hsem::is_locked(0x6u) && true == hsem::_1_step::try_lock(0x7u))
        {
            Scoped_guard<cache_disabler> cache_guard;

            bit_flag::set(&(FLASH->CR),
                          FLASH_CR_PNB | FLASH_CR_PER | FLASH_CR_STRT,
                          (a_page_index << FLASH_CR_PNB_Pos) | FLASH_CR_PER | FLASH_CR_STRT);
            wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);
            bit_flag::clear(&(FLASH->CR), FLASH_CR_PER | FLASH_CR_STRT);

            hsem::_1_step::unlock(0x7u);

            done = true;
        }
    }

    if (nullptr != a_external_lock_function)
    {
        a_external_lock_function(false);
    }

    return { get_status_flag_from_FLASH_SR(), 1u };
}

internal_flash::polling::Result
internal_flash::polling::erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index,
                                    Function_lock a_external_lock_function,
                                    Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    Scoped_guard<hsem::_1_step> sem2_guard(0x2u, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
    if (true == sem2_guard.is_locked())
    {
        Scoped_guard<unlocker> unlock_guard(a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
        if (true == unlock_guard.is_unlocked())
        {
            if (true == is_FLASH_SR_error())
            {
                clear_FLASH_SR_errors();
            }

            if (nullptr != a_external_lock_function)
            {
                a_external_lock_function(true);
            }

            bool done    = false;
            bool timeout = false;
            while (false == done && false == timeout)
            {
                wait_until::all_bits_are_cleared(FLASH->SR,
                                                 FLASH_SR_BSY | FLASH_SR_PESD,
                                                 a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

                Scoped_guard<nvic> interrupt_guard;

                if (false == hsem::is_locked(0x6u) && true == hsem::_1_step::try_lock(0x7u))
                {
                    Scoped_guard<cache_disabler> cache_guard;

                    bit_flag::set(&(FLASH->CR),
                                  FLASH_CR_PNB | FLASH_CR_PER | FLASH_CR_STRT,
                                  (a_page_index << FLASH_CR_PNB_Pos) | FLASH_CR_PER | FLASH_CR_STRT);
                    timeout =
                        false ==
                        wait_until::all_bits_are_cleared(
                            FLASH->SR, FLASH_SR_BSY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

                    bit_flag::clear(&(FLASH->CR), FLASH_CR_PER | FLASH_CR_STRT);

                    hsem::_1_step::unlock(0x7u);

                    if (false == timeout)
                    {
                        done = true;
                    }
                }
                else
                {
                    timeout = (tick_counter<Milliseconds>::get() - start >= a_timeout.get());
                }
            }

            if (nullptr != a_external_lock_function)
            {
                a_external_lock_function(false);
            }

            if (true == done)
            {
                return { get_status_flag_from_FLASH_SR(), 1u };
            }
        }
        return { Status_flag::locked, 0u };
    }

    return { get_status_flag_from_FLASH_SR(), 0u };
}

internal_flash::polling::Result internal_flash::polling::erase_bank(Bank_id a_id, Function_lock a_external_lock_function)
{
    Scoped_guard<hsem::_1_step> sem2_guard(0x2u);
    Scoped_guard<unlocker> unlock_guard;

    if (true == is_FLASH_SR_error())
    {
        clear_FLASH_SR_errors();
    }

    if (nullptr != a_external_lock_function)
    {
        a_external_lock_function(true);
    }

    bool done = false;
    while (false == done)
    {
        wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY | FLASH_SR_PESD);

        Scoped_guard<nvic> interrupt_guard;

        if (false == hsem::is_locked(0x6u) && true == hsem::_1_step::try_lock(0x7u))
        {
            Scoped_guard<cache_disabler> cache_guard;

            bit_flag::set(&(FLASH->CR), FLASH_CR_MER | FLASH_CR_STRT);
            wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

            hsem::_1_step::unlock(0x7u);

            done = true;
        }
    }

    if (nullptr != a_external_lock_function)
    {
        a_external_lock_function(false);
    }

    return { get_status_flag_from_FLASH_SR(), 1u };
}

internal_flash::polling::Result internal_flash::polling::erase_bank(Bank_id a_id, Function_lock a_external_lock_function, Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();

    Scoped_guard<hsem::_1_step> sem2_guard(0x2u, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
    if (true == sem2_guard.is_locked())
    {
        Scoped_guard<unlocker> unlock_guard(a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
        if (true == unlock_guard.is_unlocked())
        {
            if (true == is_FLASH_SR_error())
            {
                clear_FLASH_SR_errors();
            }

            if (nullptr != a_external_lock_function)
            {
                a_external_lock_function(true);
            }

            bool done    = false;
            bool timeout = false;
            while (false == done && false == timeout)
            {
                wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY | FLASH_SR_PESD);

                Scoped_guard<nvic> interrupt_guard;

                if (false == hsem::is_locked(0x6u) && true == hsem::_1_step::try_lock(0x7u))
                {
                    Scoped_guard<cache_disabler> cache_guard;

                    bit_flag::set(&(FLASH->CR), FLASH_CR_MER | FLASH_CR_STRT);
                    timeout =
                        false ==
                        wait_until::all_bits_are_cleared(
                            FLASH->SR, FLASH_SR_BSY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

                    hsem::_1_step::unlock(0x7u);

                    if (false == timeout)
                    {
                        done = true;
                    }
                }
                else
                {
                    timeout = (tick_counter<Milliseconds>::get() - start >= a_timeout.get());
                }
            }

            if (nullptr != a_external_lock_function)
            {
                a_external_lock_function(false);
            }

            if (true == done)
            {
                return { get_status_flag_from_FLASH_SR(), 1u };
            }
        }
    }

    return { get_status_flag_from_FLASH_SR(), 0u };
}
} // namespace peripherals
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif