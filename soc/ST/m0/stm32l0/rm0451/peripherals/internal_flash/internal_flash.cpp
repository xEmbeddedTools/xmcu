/**/

// xmcu
#include <xmcu/bit.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/peripherals/internal_flash/internal_flash.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/wait_until.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/delay.hpp>

// std
#include <cstring>

namespace {
using namespace xmcu;
using namespace xmcu::soc::m0::stm32l0::rm0451::peripherals;

void clear_FLASH_SR_errors()
{
    bit_flag::set(&(FLASH->SR), FLASH_SR_RDERR | FLASH_SR_WRPERR | FLASH_SR_SIZERR | FLASH_SR_PGAERR);
}

bool is_FLASH_SR_error()
{
    return bit::is_any(FLASH->SR, FLASH_SR_RDERR | FLASH_SR_WRPERR | FLASH_SR_SIZERR | FLASH_SR_PGAERR);
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
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace peripherals {
using namespace xmcu;
using namespace xmcu::soc::m0::stm32l0::rm0451::system;
using namespace utils;

void internal_flash::unlocker::unlock()
{
    wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

    if (true == bit_flag::is(FLASH->PECR, FLASH_PECR_PRGLOCK))
    {
        Scoped_guard<nvic> interrupt_guard;

        if (true == bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK))
        {
            FLASH->PEKEYR = 0x89ABCDEFu;
            FLASH->PEKEYR = 0x02030405u;
        }

        FLASH->PRGKEYR = 0x8C9DAEBFu;
        FLASH->PRGKEYR = 0x13141516u;
    }

    wait_until::all_bits_are_cleared(FLASH->PECR, FLASH_PECR_PRGLOCK);
}
bool internal_flash::unlocker::unlock(Milliseconds a_timeout)
{
    bool isCleared = wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY, a_timeout);
    if (false == isCleared)
    {
        return false;
    }

    if (true == bit_flag::is(FLASH->PECR, FLASH_PECR_PRGLOCK))
    {
        Scoped_guard<nvic> interrupt_guard;

        if (true == bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK))
        {
            FLASH->PEKEYR = 0x89ABCDEFu;
            FLASH->PEKEYR = 0x02030405u;
        }

        FLASH->PRGKEYR = 0x8C9DAEBFu;
        FLASH->PRGKEYR = 0x13141516u;
    }

    return false == bit_flag::is(FLASH->PECR, FLASH_PECR_PRGLOCK);
}
void internal_flash::unlocker::lock()
{
    bit_flag::set(&FLASH->PECR, FLASH_PECR_PRGLOCK);
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
    bit_flag::set(&(FLASH->ACR), FLASH_ACR_LATENCY, static_cast<std::uint32_t>(a_latency));
    wait_until::all_bits_are_set(FLASH->ACR, static_cast<std::uint32_t>(a_latency));
}
bool internal_flash::set_latency(Latency a_latency, Milliseconds a_timeout)
{
    bit_flag::set(&(FLASH->ACR), FLASH_ACR_LATENCY, static_cast<std::uint32_t>(a_latency));
    wait_until::all_bits_are_set(FLASH->ACR, static_cast<std::uint32_t>(a_latency));

    return true;
}

void internal_flash::set_cache_mode(Cache_mode_flag a_cache_mode)
{
    bit_flag::set(&(FLASH->ACR), FLASH_ACR_PRFTEN, static_cast<std::uint32_t>(a_cache_mode));
}
bool internal_flash::set_cache_mode(Cache_mode_flag a_cache_mode, Milliseconds a_timeout)
{
    bit_flag::set(&(FLASH->ACR), FLASH_ACR_PRFTEN, static_cast<std::uint32_t>(a_cache_mode));

    return true;
}

internal_flash::polling::Result
internal_flash::polling::write(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                               Not_null<const Word*> a_p_data,
                               std::size_t a_size_in_double_words)
{
    hkm_assert(a_size_in_double_words > 0);

    Scoped_guard<unlocker> unlock_guard;

    if (true == is_FLASH_SR_error())
    {
        clear_FLASH_SR_errors();
    }

    Scoped_guard<nvic> interrupt_guard;
    Scoped_guard<cache_disabler> cache_guard;
    bit_flag::set(&FLASH->PECR, FLASH_PECR_PROG);
    for (std::size_t i = 0; i < a_size_in_double_words;)
    {
        wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

        volatile std::uint32_t* p_address = reinterpret_cast<volatile std::uint32_t*>(a_address.get());
        *(p_address + i * 2u + 0u)        = static_cast<std::uint32_t>(a_p_data[i] >> 0x00u);
        __ISB();
        *(p_address + i * 2u + 1u) = static_cast<std::uint32_t>(a_p_data[i] >> 0x20u);

        wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);
        if (true == bit_flag::is(FLASH->SR, FLASH_SR_EOP))
        {
            bit_flag::set(&(FLASH->SR), FLASH_SR_EOP);
        }

        i++;
    }

    bit_flag::clear(&FLASH->PECR, FLASH_PECR_PROG);
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
    Scoped_guard<unlocker> unlock_guard(a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == unlock_guard.is_unlocked())
    {
        if (true == is_FLASH_SR_error())
        {
            clear_FLASH_SR_errors();
        }

        bool timeout  = false;
        std::size_t i = 0;
        Scoped_guard<nvic> interrupt_guard;
        Scoped_guard<cache_disabler> cache_guard;
        bit_flag::set(&FLASH->PECR, FLASH_PECR_PROG);
        while (i < a_size_in_double_words && false == timeout)
        {
            timeout =
                false == wait_until::all_bits_are_cleared(
                             FLASH->SR, FLASH_SR_BSY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

            if (false == timeout)
            {
                volatile std::uint32_t* p_address = reinterpret_cast<volatile std::uint32_t*>(a_address.get());

                *(p_address + i * 2u + 0u) = static_cast<std::uint32_t>(a_p_data[i] >> 0x00u);
                __ISB();
                *(p_address + i * 2u + 1u) = static_cast<std::uint32_t>(a_p_data[i] >> 0x20u);

                timeout = false ==
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
            }
        }
        bit_flag::clear(&FLASH->PECR, FLASH_PECR_PROG);
        return { get_status_flag_from_FLASH_SR(), i };
    }
    return { Status_flag::locked, 0x0u };
}

internal_flash::polling::Result
internal_flash::polling::read(Limited<std::uint32_t, s::start, s::start + s::size_in_bytes> a_address,
                              Not_null<void*> a_p_data,
                              Limited<std::size_t, 1u, s::page_size_in_bytes> a_size_in_bytes)
{
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
    Scoped_guard<unlocker> unlock_guard(a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == unlock_guard.is_unlocked())
    {
        std::memcpy(a_p_data, reinterpret_cast<const uint8_t*>(a_address.get()), a_size_in_bytes);
        return { get_status_flag_from_FLASH_SR(), a_size_in_bytes };
    }

    return { Status_flag::locked, 0x0u };
}

internal_flash::polling::Result
internal_flash::polling::erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index)
{
    Scoped_guard<unlocker> unlock_guard;

    if (true == is_FLASH_SR_error())
    {
        clear_FLASH_SR_errors();
    }

    do
    {
        wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

        Scoped_guard<nvic> interrupt_guard;
        Scoped_guard<cache_disabler> cache_guard;

        bit_flag::set(&FLASH->PECR, FLASH_PECR_ERASE | FLASH_PECR_PROG);
        volatile std::uint32_t* page_address =
            reinterpret_cast<volatile std::uint32_t*>(s::start + s::page_size_in_bytes * a_page_index);
        *page_address = 0u;
        wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);

        if (bit_flag::get(FLASH->SR, FLASH_SR_EOP))
        {
            bit_flag::set(&FLASH->SR, FLASH_SR_EOP);
        }
        else
        {
            // TODO: error handling
        }
        bit_flag::clear(&FLASH->PECR, FLASH_PECR_ERASE | FLASH_PECR_PROG);
    } while (false);

    return { .status = get_status_flag_from_FLASH_SR(), .words = 1u };
}

internal_flash::polling::Result
internal_flash::polling::erase_page(Limited<std::uint32_t, 0u, s::pages_count - 1> a_page_index, Milliseconds a_timeout)
{
    const std::uint64_t start = tick_counter<Milliseconds>::get();
    Scoped_guard<unlocker> unlock_guard(a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

    if (true == unlock_guard.is_unlocked())
    {
        if (true == is_FLASH_SR_error())
        {
            clear_FLASH_SR_errors();
        }

        bool is_timeout = false;
        do
        {
            wait_until::all_bits_are_cleared(
                FLASH->SR, FLASH_SR_BSY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));

            Scoped_guard<nvic> interrupt_guard;
            Scoped_guard<cache_disabler> cache_guard;

            bit_flag::set(&FLASH->PECR, FLASH_PECR_ERASE | FLASH_PECR_PROG);
            volatile std::uint32_t* page_address =
                reinterpret_cast<volatile std::uint32_t*>(s::start + s::page_size_in_bytes * a_page_index);
            *page_address = 0u;
            is_timeout    = wait_until::all_bits_are_cleared(
                FLASH->SR, FLASH_SR_BSY, a_timeout.get() - (tick_counter<Milliseconds>::get() - start));
            bit_flag::clear(&FLASH->PECR, FLASH_PECR_ERASE | FLASH_PECR_PROG);

            if (false == is_timeout) // TODO: it doesn't look like proper handling
            {
                return { .status = get_status_flag_from_FLASH_SR(), .words = 1u };
            }
        } while (false);
    }
    return { Status_flag::locked, 0u };
}

} // namespace peripherals
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
