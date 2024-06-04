/**/

#include <xmcu/bit.hpp>
#include <xmcu/Duration.hpp>
#include <xmcu/time_utils.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/peripherals/rtc/rtc.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/tick_counter.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/utils/wait_until.hpp>
#include <stm32wbxx.h>

namespace {
using namespace xmcu;
using namespace xmcu::soc;
using namespace xmcu::soc::m4::stm32wb;
using namespace xmcu::soc::m4::stm32wb::peripherals;

struct alarm_registers
{
    volatile uint32_t* p_alarm_reg;
    volatile uint32_t* p_subseconds_reg;
    std::uint32_t enable_mask;
    std::uint32_t int_mask;
    std::uint32_t int_enable_mask;
};

constexpr alarm_registers f_alarm_regs[2] = {
    [various::to_underlying(rtc::Alarm_id::A)] = { .p_alarm_reg = &RTC->ALRMAR,
                                                   .p_subseconds_reg = &RTC->ALRMASSR,
                                                   .enable_mask = RTC_CR_ALRAE,
                                                   .int_mask = RTC_ISR_ALRAF,
                                                   .int_enable_mask = RTC_CR_ALRAIE },
    [various::to_underlying(rtc::Alarm_id::B)] = { .p_alarm_reg = &RTC->ALRMBR,
                                                   .p_subseconds_reg = &RTC->ALRMBSSR,
                                                   .enable_mask = RTC_CR_ALRBE,
                                                   .int_mask = RTC_ISR_ALRBF,
                                                   .int_enable_mask = RTC_CR_ALRBIE },
};

rtc::Alarm_handler f_alarm_handlers[2] = { { .function = nullptr, .p_data = nullptr } };

constexpr const alarm_registers& get_alarm_regs(rtc::Alarm_id a_id)
{
    return f_alarm_regs[various::to_underlying(a_id)];
}

void rcc_enable_rtc(bool a_enable_in_lp, std::uint32_t a_select_flag)
{
    if (true == rcc<rtc>::is_enabled())
    {
        return;
    }

    Scoped_guard<rtc> unlocker;

    bit_flag::set(&(RCC->APB1ENR1), RCC_APB1ENR1_RTCAPBEN);
    bit_flag::set(&(RCC->BDCR), RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL, RCC_BDCR_RTCEN | a_select_flag);

    if (true == a_enable_in_lp)
    {
        bit_flag::set(&(RCC->APB1SMENR1), RCC_APB1SMENR1_RTCAPBSMEN);
    }
    else
    {
        bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_RTCAPBSMEN);
    }
}

void rtc_alarm_interrupt_handler()
{
    const rtc::Alarm_handler* p_handler;

    p_handler = &::f_alarm_handlers[various::to_underlying(rtc::Alarm_id::A)];
    if (bit_flag::is(RTC->ISR, RTC_ISR_ALRAF) && p_handler->function != nullptr)
    {
        p_handler->function(rtc::Alarm_id::A, p_handler->p_data);
        bit_flag::clear(&RTC->ISR, RTC_ISR_ALRAF);
    }

    p_handler = &::f_alarm_handlers[various::to_underlying(rtc::Alarm_id::B)];
    if (bit_flag::is(RTC->ISR, RTC_ISR_ALRBF) && p_handler->function != nullptr)
    {
        p_handler->function(rtc::Alarm_id::B, p_handler->p_data);
        bit_flag::clear(&RTC->ISR, RTC_ISR_ALRBF);
    }

    // ES0394 2.12.1 - double-check alarm A due to possible masking by interrupt caused by alarm B
    p_handler = &::f_alarm_handlers[various::to_underlying(rtc::Alarm_id::A)];
    if (bit_flag::is(RTC->ISR, RTC_ISR_ALRAF) && p_handler->function != nullptr)
    {
        p_handler->function(rtc::Alarm_id::A, p_handler->p_data);
        bit_flag::clear(&RTC->ISR, RTC_ISR_ALRAF);
    }
}
} // namespace

extern "C" {

void RTC_Alarm_IRQHandler()
{
    EXTI->PR1 = EXTI_PR1_PIF17;
    rtc_alarm_interrupt_handler();
}
}

namespace xmcu::soc::m4::stm32wb::peripherals {
using namespace xmcu;
using namespace xmcu::soc::m4::stm32wb::utils;

void rtc::set_clock(Milliseconds a_world_millis)
{
    Scoped_guard<rtc> unlocker;

    // Enter calendar initialization mode
    bit_flag::set(&RTC->ISR, RTC_ISR_INIT);
    wait_until::any_bit_is_set(RTC->ISR, RTC_ISR_INITF);

    // Set prescalers to generate 1Hz
    bit_flag::set(&RTC->PRER, RTC_PRER_PREDIV_A_Msk, (128 - 1) << RTC_PRER_PREDIV_A_Pos);
    bit_flag::set(&RTC->PRER, RTC_PRER_PREDIV_S_Msk, (256 - 1) << RTC_PRER_PREDIV_S_Pos);

    // Configure 24-hour format
    bit_flag::clear(&RTC->CR, RTC_CR_FMT);

    // Set time/date
    time_utils::Timestamp date_and_time = time_utils::from_unix_epoch(a_world_millis.get() / 1000u);
    bit_flag::clear(&RTC->TR, RTC_TR_PM);
    RTC->TR = ((date_and_time.time.hour / 10) << RTC_TR_HT_Pos) | ((date_and_time.time.hour % 10) << RTC_TR_HU_Pos) |
              ((date_and_time.time.minute / 10) << RTC_TR_MNT_Pos) |
              ((date_and_time.time.minute % 10) << RTC_TR_MNU_Pos) |
              ((date_and_time.time.second / 10) << RTC_TR_ST_Pos) | ((date_and_time.time.second % 10) << RTC_TR_SU_Pos);

    RTC->DR = (((date_and_time.date.year / 10) % 10) << RTC_DR_YT_Pos) |
              ((date_and_time.date.year % 10) << RTC_DR_YU_Pos) | ((date_and_time.date.month / 10) << RTC_DR_MT_Pos) |
              ((date_and_time.date.month % 10) << RTC_DR_MU_Pos) | ((date_and_time.date.day / 10) << RTC_DR_DT_Pos) |
              ((date_and_time.date.day % 10) << RTC_DR_DU_Pos);

    // Exit init mode
    bit_flag::clear(&RTC->ISR, RTC_ISR_INIT);
    wait_until::any_bit_is_set(RTC->ISR, RTC_ISR_INITS);
}

bool rtc::is_clock_set()
{
    // Checks if year is 00
    bool is_set = bit_flag::is(RTC->ISR, RTC_ISR_INITS);
    return is_set;
}

Milliseconds rtc::get_time()
{
    // TODO: perhaps TR/DR latch prevention could be moved to the end of set_clock()? But it's safer here.
    rtc::wait_for_sync();

    // Read TR until coherent value is obtained.
    std::uint32_t time = RTC->TR;
    while (true)
    {
        std::uint32_t next_time = RTC->TR;
        if (next_time == time)
        {
            break;
        }
        time = next_time;
    }

    // The DR/SSR values will be latched after TR read, until DR is read.
    std::uint32_t date = RTC->DR;

    time_utils::Timestamp timestamp = { .time = { .hour = (bit_flag::get(time, RTC_TR_HT) >> RTC_TR_HT_Pos) * 10 +
                                                          (bit_flag::get(time, RTC_TR_HU) >> RTC_TR_HU_Pos),
                                                  .minute = (bit_flag::get(time, RTC_TR_MNT) >> RTC_TR_MNT_Pos) * 10 +
                                                            (bit_flag::get(time, RTC_TR_MNU) >> RTC_TR_MNU_Pos),
                                                  .second = (bit_flag::get(time, RTC_TR_ST) >> RTC_TR_ST_Pos) * 10 +
                                                            (bit_flag::get(time, RTC_TR_SU) >> RTC_TR_SU_Pos) },
                                        .date = { .day = (bit_flag::get(date, RTC_DR_DT) >> RTC_DR_DT_Pos) * 10 +
                                                         (bit_flag::get(date, RTC_DR_DU) >> RTC_DR_DU_Pos),
                                                  .month = (bit_flag::get(date, RTC_DR_MT) >> RTC_DR_MT_Pos) * 10 +
                                                           (bit_flag::get(date, RTC_DR_MU) >> RTC_DR_MU_Pos),
                                                  .year = 2000 +
                                                          (bit_flag::get(date, RTC_DR_YT) >> RTC_DR_YT_Pos) * 10 +
                                                          (bit_flag::get(date, RTC_DR_YU) >> RTC_DR_YU_Pos) } };
    Milliseconds world_millis(time_utils::to_unix_epoch(timestamp) * 1000u);
    return world_millis;
}

void rtc::wait_for_sync()
{
    Scoped_guard<rtc> unlocker;

    // Read out old values of RTC registers, they can be latched by some register accesses until read
    (void)RTC->TR;
    (void)RTC->DR;

    // Wait for next sync
    bit_flag::clear(&RTC->ISR, RTC_ISR_RSF);
    wait_until::any_bit_is_set(RTC->ISR, RTC_ISR_RSF);
}

void rtc::enable_alarm(Alarm_id a_id, Alarm_mask a_mask, xmcu::Milliseconds a_world_millis, Alarm_handler a_handler)
{
    Scoped_guard<rtc> unlocker;

    time_utils::Timestamp date_and_time = time_utils::from_unix_epoch(a_world_millis.get() / 1000u);
    const alarm_registers& alarm_regs = ::get_alarm_regs(a_id);
    bit_flag::clear(&RTC->CR, alarm_regs.enable_mask);

    bit_flag::clear(alarm_regs.p_alarm_reg, RTC_ALRMAR_PM);
    bit_flag::set(alarm_regs.p_alarm_reg, RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1);
    if (bit_flag::is(a_mask, Alarm_mask::days))
    {
        bit_flag::set(alarm_regs.p_alarm_reg, RTC_ALRMAR_DT, (date_and_time.date.day / 10) << RTC_ALRMAR_DT_Pos);
        bit_flag::set(alarm_regs.p_alarm_reg, RTC_ALRMAR_DU, (date_and_time.date.day % 10) << RTC_ALRMAR_DU_Pos);
        bit_flag::clear(alarm_regs.p_alarm_reg, RTC_ALRMAR_MSK4);
    }
    if (bit_flag::is(a_mask, Alarm_mask::hours))
    {
        bit_flag::set(alarm_regs.p_alarm_reg, RTC_ALRMAR_HT, (date_and_time.time.hour / 10) << RTC_ALRMAR_HT_Pos);
        bit_flag::set(alarm_regs.p_alarm_reg, RTC_ALRMAR_HU, (date_and_time.time.hour % 10) << RTC_ALRMAR_HU_Pos);
        bit_flag::clear(alarm_regs.p_alarm_reg, RTC_ALRMAR_MSK3);
    }
    if (bit_flag::is(a_mask, Alarm_mask::minutes))
    {
        bit_flag::set(alarm_regs.p_alarm_reg, RTC_ALRMAR_MNT, (date_and_time.time.minute / 10) << RTC_ALRMAR_MNT_Pos);
        bit_flag::set(alarm_regs.p_alarm_reg, RTC_ALRMAR_MNU, (date_and_time.time.minute % 10) << RTC_ALRMAR_MNU_Pos);
        bit_flag::clear(alarm_regs.p_alarm_reg, RTC_ALRMAR_MSK2);
    }
    if (bit_flag::is(a_mask, Alarm_mask::seconds))
    {
        bit_flag::set(alarm_regs.p_alarm_reg, RTC_ALRMAR_ST, (date_and_time.time.second / 10) << RTC_ALRMAR_ST_Pos);
        bit_flag::set(alarm_regs.p_alarm_reg, RTC_ALRMAR_SU, (date_and_time.time.second % 10) << RTC_ALRMAR_SU_Pos);
        bit_flag::clear(alarm_regs.p_alarm_reg, RTC_ALRMAR_MSK1);
    }

    *alarm_regs.p_subseconds_reg = 0; // We don't need subsecond precision

    bit_flag::clear(&RTC->CR, alarm_regs.int_enable_mask);
    bit_flag::clear(&RTC->ISR, alarm_regs.int_mask);

    bit_flag::set(&RTC->CR, alarm_regs.enable_mask);
    f_alarm_handlers[various::to_underlying(a_id)] = a_handler;
    if (a_handler.function != nullptr)
    {
        if (false == bit::is_any(RTC->CR, RTC_CR_ALRAIE | RTC_CR_ALRBIE))
        {
            bit_flag::set(&EXTI->IMR1, EXTI_IMR1_IM17);
            bit_flag::set(&EXTI->RTSR1, EXTI_RTSR1_RT17);
            NVIC_SetPriority(RTC_Alarm_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
            NVIC_EnableIRQ(RTC_Alarm_IRQn);
        }
        bit_flag::set(&RTC->CR, alarm_regs.int_enable_mask);
    }
}

void rtc::disable_alarm(Alarm_id a_id)
{
    Scoped_guard<rtc> unlocker;

    const alarm_registers& alarm_regs = ::get_alarm_regs(a_id);
    bit_flag::clear(&RTC->CR, alarm_regs.enable_mask);
    bit_flag::clear(&RTC->CR, alarm_regs.int_enable_mask);
    bit_flag::clear(&RTC->ISR, alarm_regs.int_mask);

    if (false == bit::is_any(RTC->CR, RTC_CR_ALRAIE | RTC_CR_ALRBIE))
    {
        bit_flag::clear(&EXTI->IMR1, EXTI_IMR1_IM17);
        bit_flag::clear(&EXTI->RTSR1, EXTI_RTSR1_RT17);
        NVIC_DisableIRQ(RTC_Alarm_IRQn);
    }
}

bool rtc::is_alarm_triggered(Alarm_id a_id)
{
    const alarm_registers& alarm_regs = ::get_alarm_regs(a_id);
    bool is_triggered = bit_flag::is(RTC->ISR, alarm_regs.int_mask);
    return is_triggered;
}

uint32_t rtc::read_bkp_register(std::size_t a_index)
{
    switch (a_index)
    {
        case 0:
            return RTC->BKP0R;
        case 1:
            return RTC->BKP1R;
        case 2:
            return RTC->BKP2R;
        case 3:
            return RTC->BKP3R;
        case 4:
            return RTC->BKP4R;
    }
    return 0x0;
}

void rtc::write_bkp_register(std::size_t a_index, uint32_t a_value)
{
    switch (a_index)
    {
        case 0:
            RTC->BKP0R = a_value;
            break;
        case 1:
            RTC->BKP1R = a_value;
            break;
        case 2:
            RTC->BKP2R = a_value;
            break;
        case 3:
            RTC->BKP3R = a_value;
            break;
        case 4:
            RTC->BKP4R = a_value;
            break;
    }
}

} // namespace xmcu::soc::m4::stm32wb::peripherals

namespace xmcu::soc::m4::stm32wb {
using namespace xmcu::soc::m4::stm32wb::sources;
using namespace xmcu::soc::m4::stm32wb::peripherals;

template<> void rcc<rtc>::enable<lsi>(bool a_enable_in_lp)
{
    ::rcc_enable_rtc(a_enable_in_lp, RCC_BDCR_RTCSEL_1);
}

template<> void rcc<rtc>::enable<lse>(bool a_enable_in_lp)
{
    ::rcc_enable_rtc(a_enable_in_lp, RCC_BDCR_RTCSEL_0);
}

template<> void rcc<rtc>::enable<hse>(bool a_enable_in_lp)
{
    ::rcc_enable_rtc(a_enable_in_lp, RCC_BDCR_RTCSEL_1 | RCC_BDCR_RTCSEL_0);
}

void rcc<rtc>::disable()
{
    Scoped_guard<backup_domain> unlocker;

    bit_flag::clear(&(RCC->APB1ENR1), RCC_APB1ENR1_RTCAPBEN);
    bit_flag::clear(&(RCC->BDCR), RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL);
    bit_flag::clear(&(RCC->APB1SMENR1), RCC_APB1SMENR1_RTCAPBSMEN);
}

bool rcc<rtc>::is_enabled()
{
    bool is_enabled = bit_flag::is(RCC->APB1ENR1, RCC_APB1ENR1_RTCAPBEN) && bit_flag::is(RCC->BDCR, RCC_BDCR_RTCEN) &&
                      bit::is_any(RCC->BDCR, RCC_BDCR_RTCSEL);
    return is_enabled;
}

} // namespace xmcu::soc::m4::stm32wb
