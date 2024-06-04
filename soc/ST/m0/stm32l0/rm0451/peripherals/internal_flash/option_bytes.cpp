/**/

// this
#include <xmcu/soc/ST/m0/stm32l0/rm0451/peripherals/internal_flash/option_bytes.hpp>

// externals
#include <stm32l0xx.h>

// xmcu
#include <xmcu/bit_flag.hpp>
#include <xmcu/soc/Scoped_guard.hpp>
#include <xmcu/soc/ST/m0/nvic.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/peripherals/internal_flash/internal_flash.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m0/stm32l0/rm0451/utils/wait_until.hpp>

namespace {
struct OB_record
{
    struct Entries
    {
        std::uint8_t byte_0 = 0x0u;
        std::uint8_t byte_1 = 0x0u;
        std::uint8_t complemented_byte_0 = 0x0u;
        std::uint8_t complemented_byte_1 = 0x0u;
    };

    union
    {
        Entries entries;
        std::uint32_t word = 0x0u;
    };
};

constexpr std::uint32_t OB_BASE_ADDRESS_SLOT_0 = OB_BASE;
constexpr std::uint32_t OB_BASE_ADDRESS_SLOT_1 = OB_BASE_ADDRESS_SLOT_0 + 4u;
constexpr std::uint32_t OB_BASE_ADDRESS_SLOT_2 = OB_BASE_ADDRESS_SLOT_1 + 4u;
constexpr std::uint32_t OB_BASE_ADDRESS_SLOT_3 = OB_BASE_ADDRESS_SLOT_2 + 4u;
constexpr std::uint32_t OB_BASE_ADDRESS_SLOT_4 = OB_BASE_ADDRESS_SLOT_3 + 4u;
} // namespace

namespace xmcu {
namespace soc {
namespace m0 {
namespace stm32l0 {
namespace rm0451 {
namespace peripherals {
using namespace xmcu;
using namespace xmcu::soc::m0::stm32l0::rm0451::utils;
using namespace xmcu::soc::m0::stm32l0::rm0451::system;

void option_bytes::reload()
{
    // PELOCK/OPTLOCK must be 0
    Scoped_guard<internal_flash::unlocker> flash_guard;

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard;

        if (true == ob_guard.is_unlocked())
        {
            bit_flag::set(&FLASH->PECR, FLASH_PECR_OBL_LAUNCH);
        }
    }
}

void option_bytes::unlocker::unlock()
{
    wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);
    if (true == bit_flag::is(FLASH->PECR, FLASH_PECR_OPTLOCK))
    {
        Scoped_guard<nvic> interrupt_guard;
        if (true == bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK))
        {
            FLASH->PEKEYR = 0x89ABCDEFu;
            FLASH->PEKEYR = 0x02030405u;
        }

        FLASH->OPTKEYR = 0xFBEAD9C8u;
        FLASH->OPTKEYR = 0x24252627u;
    }
}
bool option_bytes::unlocker::unlock(Milliseconds a_timeout)
{
    bool isCleared = wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY, a_timeout);
    if (!isCleared)
    {
        return false;
    }

    if (true == bit_flag::is(FLASH->PECR, FLASH_PECR_OPTLOCK))
    {
        Scoped_guard<nvic> interrupt_guard;
        if (true == bit_flag::is(FLASH->PECR, FLASH_PECR_PELOCK))
        {
            FLASH->PEKEYR = 0x89ABCDEFu;
            FLASH->PEKEYR = 0x02030405u;
        }

        FLASH->OPTKEYR = 0xFBEAD9C8u;
        FLASH->OPTKEYR = 0x24252627u;
    }

    return true;
}
void option_bytes::unlocker::lock()
{
    bit_flag::set(&(FLASH->PECR), FLASH_PECR_OPTLOCK);
}

bool option_bytes::BOR::set(Level a_level)
{
    Scoped_guard<internal_flash::unlocker> flash_guard;

    if (true == flash_guard.is_unlocked())
    {
        Scoped_guard<option_bytes::unlocker> ob_guard;

        if (true == ob_guard.is_unlocked())
        {
            OB_record record_slot_1 = *(reinterpret_cast<OB_record*>(OB_BASE_ADDRESS_SLOT_1));
            bit_flag::set(&record_slot_1.entries.byte_0, 0xFu, static_cast<std::uint8_t>(a_level));
            record_slot_1.entries.complemented_byte_0 = static_cast<std::uint8_t>(~(record_slot_1.entries.byte_0));

            (reinterpret_cast<OB_record*>(OB_BASE_ADDRESS_SLOT_1))->word = record_slot_1.word;

            wait_until::all_bits_are_cleared(FLASH->SR, FLASH_SR_BSY);
            return true;
        }
    }

    return false;
}

option_bytes::BOR::Level option_bytes::BOR::get()
{
    Scoped_guard<internal_flash::unlocker> flash_guard;
    return static_cast<Level>(bit_flag::get(FLASH->OPTR, FLASH_OPTR_BOR_LEV) >> FLASH_OPTR_BOR_LEV_Pos);
}

std::uint32_t option_bytes::USER::get()
{
    Scoped_guard<internal_flash::unlocker> flash_guard;
    return bit_flag::get(FLASH->OPTR, FLASH_OPTR_nRST_STOP | FLASH_OPTR_nRST_STDBY | FLASH_OPTR_IWDG_SW);
}
} // namespace peripherals
} // namespace rm0451
} // namespace stm32l0
} // namespace m0
} // namespace soc
} // namespace xmcu
