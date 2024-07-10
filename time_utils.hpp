#pragma once

// std
#include <cstdint>
#include <string_view>

// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/non_constructible.hpp>
#include <xmcu/Not_null.hpp>

namespace xmcu {
struct time_utils : private non_constructible
{
    struct Timestamp
    {
        struct Time
        {
            std::uint32_t hour = 0u;
            std::uint32_t minute = 0u;
            std::uint32_t second = 0u;
        };

        struct Date
        {
            std::uint32_t day = 0u;
            std::uint32_t month = 0u;
            std::uint32_t year = 0u;
        };

        Time time;
        Date date;
    };

    static std::uint64_t to_unix_epoch(const Timestamp& a_timestamp);
    static Timestamp from_unix_epoch(std::uint64_t a_epoch);
    static bool from_cstring(Not_null<Timestamp*> a_p_out,
                             std::string_view a_string); // format: Tue Jun 13 09:55:40 2023
    static bool is_leap_year(std::uint32_t a_year);

    static Milliseconds get_last_midnight(Milliseconds a_world_time);

    static constexpr Milliseconds get_milliseconds_in_day()
    {
        return 24 * 60 * 60 * 1000;
    }
};
} // namespace xmcu