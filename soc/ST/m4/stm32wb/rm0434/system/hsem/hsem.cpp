/**/

#if defined(STM32WB)

// this
#include <xmcu/soc/ST/m4/stm32wb/rm0434/system/hsem/hsem.hpp>

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// xmcu
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
namespace system {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::utils;

void hsem::_1_step::lock(Limited<std::uint8_t, 0, 31> a_semaphore_id)
{
    wait_until::all_bits_are_set(HSEM->RLR[a_semaphore_id], HSEM_CR_COREID_CURRENT | HSEM_RLR_LOCK);
}
bool hsem::_1_step::try_lock(Limited<std::uint8_t, 0, 31> a_semaphore_id, Milliseconds a_timeout)
{
    return wait_until::all_bits_are_set(HSEM->RLR[a_semaphore_id], HSEM_CR_COREID_CURRENT | HSEM_RLR_LOCK, a_timeout);
}
bool hsem::_1_step::try_lock(Limited<std::uint8_t, 0, 31> a_semaphore_id)
{
    return HSEM->RLR[a_semaphore_id] == (HSEM_CR_COREID_CURRENT | HSEM_RLR_LOCK);
}
void hsem::_1_step::unlock(Limited<std::uint8_t, 0, 31> a_semaphore_id)
{
    HSEM->R[a_semaphore_id] = (HSEM_CR_COREID_CURRENT);
}

void hsem::_2_step::lock(Limited<std::uint8_t, 0, 31> a_semaphore_id, std::uint8_t a_process_id)
{
    do
    {
        HSEM->R[a_semaphore_id] = (HSEM_R_LOCK | HSEM_CR_COREID_CURRENT | a_process_id);
    } while (HSEM->R[a_semaphore_id] != (HSEM_R_LOCK | HSEM_CR_COREID_CURRENT | a_process_id));
}

bool hsem::_2_step::try_lock(Limited<std::uint8_t, 0, 31> a_semaphore_id,
                             std::uint8_t a_process_id,
                             Milliseconds a_timeout)
{
    const std::uint64_t timeout_end = tick_counter<Milliseconds>::get() + a_timeout.get();

    bool ret = false;
    do
    {
        HSEM->R[a_semaphore_id] = (HSEM_R_LOCK | HSEM_CR_COREID_CURRENT | a_process_id);
        ret                     = HSEM->R[a_semaphore_id] == (HSEM_R_LOCK | HSEM_CR_COREID_CURRENT | a_process_id);
    } while (false == ret && tick_counter<Milliseconds>::get() < timeout_end);

    return ret;
}

bool hsem::_2_step::try_lock(Limited<std::uint8_t, 0, 31> a_semaphore_id, std::uint8_t a_process_id)
{
    HSEM->R[a_semaphore_id] = (HSEM_R_LOCK | HSEM_CR_COREID_CURRENT | a_process_id);
    return HSEM->R[a_semaphore_id] == (HSEM_R_LOCK | HSEM_CR_COREID_CURRENT | a_process_id);
}

void hsem::_2_step::unlock(Limited<std::uint8_t, 0, 31> a_semaphore_id, std::uint8_t a_process_id)
{
    HSEM->R[a_semaphore_id] = (a_process_id | HSEM_CR_COREID_CURRENT);
}
} // namespace system
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu

#endif