#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <array>
// #include <concepts>
// #include <cstdint>
// #include <tuple>
#include <type_traits>
// #include <ranges>

namespace xmcu::look_up_tables {

#define concept_allowed 1
#if concept_allowed

template<typename T>
concept is_row = requires(const T a) {
    typename T::value_type;
    typename T::containter_type;
    a.size();
} && requires(T a, T b) {
    { a |= b };
};

template<typename T>
concept is_desc = is_row<T> && requires {
    typename T::destination_type;
    typename T::reduced_type<1>;
};
template<typename T>
concept is_Input_table = requires {
    typename T::value_type;
    typename T::containter_type;
    { T::decode_state(0) } -> std::convertible_to<std::size_t>;
} && xmcu::look_up_tables::is_desc<typename T::value_type> && requires(const T& a) {
    a.size();
    a.lut_out(0);
};
#else
#define is_row typename
#define is_desc typename
#define is_Input_table typename
#endif

template<typename T, std::size_t N> class Row : public std::array<T, N>
{
public:
    using containter_type = std::array<T, N>;
    consteval containter_type& as_array()
    {
        return *this;
    }
    // operators
    constexpr Row<T, N>& operator|=(const Row<T, N>& a_rhs)
    {
        auto& ref_this = *this;

        for (std::size_t i = 0; i < this->size(); ++i)
        {
            ref_this[i] |= a_rhs[i];
        }
        return ref_this;
    }
    constexpr Row<T, N> operator|(const Row<T, N>& a_rhs) const
    {
        auto ref_this = *this;

        for (std::size_t i = 0; i < this->size(); ++i)
        {
            ref_this[i] |= a_rhs[i];
        }
        return ref_this;
    }
};

// single row
template<typename T, std::size_t N> class Input_row // might be join with row
    : public Row<T, N>
{
public:
    using destination_type = Row<T, N>;
    template<std::size_t N_> using reduced_type = Input_row<T, N_>;
};

namespace decoders {
struct Direct
{
    consteval static std::size_t decode(std::size_t a_value)
    {
        return a_value;
    }
};
template<int OFFSET> struct Offset
{
    consteval static std::size_t decode(std::size_t a_value)
    {
        return a_value + OFFSET;
    }
};
template<int OFFSET, std::size_t MASK> struct Offset_Mask
{
    consteval static std::size_t decode(std::size_t a_value)
    {
        return MASK & (a_value + OFFSET);
    }
};
} // namespace decoders

template<is_desc T, std::size_t N, typename Decoder = decoders::Direct> class Input_table : public std::array<T, N>
{
public:
    // introduced for local usage only (`lut_out`)
    using destination_type = typename T::destination_type;
    using containter_type = std::array<T, N>;
    consteval Input_table(const std::array<T, N>& a_table, Decoder = {})
        : std::array<T, N> { a_table }
    {
    }
    consteval static std::size_t decode_state(std::size_t a_state)
    {
        return Decoder::decode(a_state);
    }
    consteval auto lut_out(std::size_t a_index) const
    {
        auto binary = decode_state(a_index);
        auto result = destination_type {};
        for (std::size_t i = 0; i < this->size(); ++i)
        {
            if (0 == (binary & 1u << i))
            {
                continue;
            }
            result |= (*this)[i];
        }
        return result;
    }
};

template<is_desc T, std::size_t BITS, is_row U = T::destination_type, std::size_t N = 1u << BITS> class Table
    : public std::array<U, N>
{
public:
    using desc_type = T;
    static constexpr std::size_t output_words_count = BITS;

    template<std::size_t N_dest>
#if concept_allowed
        requires(N_dest < output_words_count && N_dest > 0)
#endif
    consteval Table<typename T::reduced_type<N_dest>, BITS>
    extract_words(const std::array<typename U::size_type, N_dest>& a_columns) const
    {
        static_assert(N_dest < output_words_count && N_dest > 0,
                      "N_dest should be in range [0 - LUT output word size]");
        using Row = typename T::reduced_type<N_dest>;
        Table<Row, BITS> result;

        for (std::size_t i = 0; i < N; ++i)
        {
            Row line {};
            const auto& source_line = this->at(i);
            for (std::size_t word = 0; word < a_columns.size(); ++word)
            {
                line[word] = source_line.at(a_columns[word]);
            }
            result[i] = line;
        }

        return result;
    }
    consteval Table operator|(const Table& a_rhs) const
    {
        Table result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result[i] = (*this)[i] | a_rhs[i];
        }

        return result;
    }
};

template<is_desc T, std::size_t N, is_Input_table I>
consteval auto compose_table_from_bitlist(const I& a_description, const Table<T, N>& input_table)
{
    auto result_array { input_table };
    for (std::size_t channel = 0; channel < result_array.size(); ++channel)
    {
        result_array.at(channel) |= a_description.lut_out(channel);
    }
    return result_array;
}

template<is_desc T, std::size_t N, is_row U = T::destination_type>
consteval auto compose_table_from_bitlist(const std::array<T, N>& a_description)
{
    const std::array<U, 1 << N> input_table = std::array<U, 1 << N> {};
    return compose_table_from_bitlist(a_description, input_table);
}

} // namespace xmcu::look_up_tables
