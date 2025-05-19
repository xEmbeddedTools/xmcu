// xmcu
#include <xmcu/Duration.hpp>
#include <xmcu/time_utils.hpp>

// std
#include <charconv>

// hkm
#include <xmcu/assertion.hpp>

namespace xmcu {
Seconds time_utils::to_unix_epoch(const Timestamp& a_timestamp)
{
    constexpr std::uint32_t min_to_sec = 60u;
    constexpr std::uint32_t hour_to_sec = 60u * 60u;
    constexpr std::uint32_t day_to_sec = 60u * 60u * 24u;
    constexpr std::uint32_t year_to_sec = 60u * 60u * 24u * 365u;
    constexpr std::uint32_t four_years_to_sec = 60u * 60u * 24u * (3u * 365u + 366u);

    std::uint64_t epoch = a_timestamp.time.second + (a_timestamp.time.minute * min_to_sec) +
                          (a_timestamp.time.hour * hour_to_sec) + ((a_timestamp.date.day - 1) * day_to_sec);

    std::uint32_t temp_year = a_timestamp.date.year - 1970u + 1u; // + 1 to make calculations easier
    epoch += temp_year / 4u * four_years_to_sec - 1u * year_to_sec;
    // TODO: implement all rules for determining leap years, not only divisible by 4
    epoch += (temp_year % 4u) * year_to_sec;

    const std::uint32_t months_days[12u] = {
        31u, (true == is_leap_year(a_timestamp.date.year) ? 29u : 28u), 31u, 30u, 31u, 30u, 31u, 31u, 30u, 31u, 30u, 31u
    };
    for (std::uint32_t i = 0; i < a_timestamp.date.month - 1u; i++)
    {
        epoch += months_days[i] * day_to_sec;
    }

    return epoch;
}

time_utils::Timestamp time_utils::from_unix_epoch(Seconds a_epoch)
{
    Timestamp ret;

    volatile std::uint64_t temp_timestamp = a_epoch.get();
    std::uint32_t year = 1970u;

    if (temp_timestamp > 18934214400u)
    {
        temp_timestamp = 0u;
        year = 0u;
    }
    else if (temp_timestamp > 6311433600u)
    {
        temp_timestamp -= 6311433600u;
        year = 2170u;
    }
    else if (temp_timestamp > 3155760000u)
    {
        temp_timestamp -= 3155760000u;
        year = 2070u;
    }

    ret.time.second = temp_timestamp % 60u;
    temp_timestamp /= 60u;
    ret.time.minute = temp_timestamp % 60;
    temp_timestamp /= 60u;
    ret.time.hour = temp_timestamp % 24;
    temp_timestamp /= 24u;

    for (; temp_timestamp >= 365u;)
    {
        if (true == is_leap_year(year))
        {
            temp_timestamp = temp_timestamp - 1u;
        }
        ++year;
        temp_timestamp = temp_timestamp - 365u;
    }

    bool leap_year = is_leap_year(year);

    uint8_t months_days[12] = { 31, static_cast<uint8_t>(leap_year ? 29 : 28), 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

    temp_timestamp = temp_timestamp + 1u; // to count days from 1 insted of 0
    uint8_t i = 0;
    for (; months_days[i] < temp_timestamp; ++i)
    {
        temp_timestamp -= months_days[i];
    }
    ret.date.day = temp_timestamp;
    ret.date.month = i + 1;
    ret.date.year = year;

    return ret;
}

bool time_utils::from_cstring(Not_null<time_utils::Timestamp*> a_p_out, std::string_view a_string)
{
    if (a_string.length() >= 24u)
    {
        const std::string_view day = a_string.substr(8u, 2u);
        const std::string_view month = a_string.substr(4u, 3u);
        const std::string_view year = a_string.substr(20u, 4u);

        const bool day_result =
            std::from_chars(day.begin(), day.end(), a_p_out->date.day).ec != std::errc::invalid_argument;
        const bool year_result =
            std::from_chars(year.begin(), year.end(), a_p_out->date.year).ec != std::errc::invalid_argument;

        if (true == day_result && true == year_result)
        {
            const char* months_lut[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                           "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

            a_p_out->date.month = 0u;
            for (std::size_t i = 0u; i < 12u; i++)
            {
                if (months_lut[i] == month)
                {
                    a_p_out->date.month = i + 1u;
                    break;
                }
            }

            if (0u != a_p_out->date.month)
            {
                const std::string_view hour = a_string.substr(11u, 2u);
                const std::string_view minute = a_string.substr(14u, 2u);
                const std::string_view second = a_string.substr(17u, 2u);

                const bool hour_result =
                    std::from_chars(hour.begin(), hour.end(), a_p_out->time.hour).ec != std::errc::invalid_argument;
                const bool minute_result = std::from_chars(minute.begin(), minute.end(), a_p_out->time.minute).ec !=
                                           std::errc::invalid_argument;
                const bool second_result = std::from_chars(second.begin(), second.end(), a_p_out->time.second).ec !=
                                           std::errc::invalid_argument;

                return true == hour_result && true == minute_result && true == second_result;
            }
        }

        return false;
    }
    return false;
}

bool time_utils::is_leap_year(std::uint32_t a_year)
{
    return (0u == a_year % 4u && 0u != a_year % 100u) || 0u == a_year % 400u;
}

Milliseconds time_utils::get_last_midnight(Milliseconds a_world_time)
{
    constexpr std::uint32_t millis_in_day = get_milliseconds_in_day().get();
    Milliseconds midnight_time(a_world_time.get() - (a_world_time.get() % millis_in_day));
    return midnight_time;
}

} // namespace xmcu
