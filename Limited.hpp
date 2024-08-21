#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// hkm
#include <xmcu/assertion.hpp>

namespace xmcu {
template<typename Type_t, Type_t minimum, Type_t maximum> class Limited
{
    static_assert(maximum >= minimum);

public:
    constexpr Limited()
        : value(minimum)
    {
    }

    Limited(const Limited& a_other)
        : value(a_other.value)
    {
    }

    Limited(Limited&& a_other)
        : value(a_other.value)
    {
    }

    constexpr Limited(Type_t a_value)
        : value(a_value)
    {
        hkm_assert(this->value >= minimum && this->value <= maximum);
    }

    Limited& operator=(const Limited& a_other)
    {
        if (this != &(a_other))
        {
            this->value = a_other.value;
        }

        return *this;
    }

    operator Type_t()
    {
        return this->value;
    }
    operator Type_t() const
    {
        return this->value;
    }

    constexpr Type_t get() const
    {
        return this->value;
    }

private:
    Type_t value;
};
} // namespace xmcu