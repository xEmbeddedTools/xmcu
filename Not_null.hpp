#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cctype>
#include <type_traits>

// hkm
#include <xmcu/assertion.hpp>

namespace xmcu {
template<typename Type_t> class Not_null
{
    static_assert(std::is_assignable<Type_t&, std::nullptr_t>::value, "Type_t cannot be assigned nullptr.");

public:
    Not_null(Type_t a_value)
        : pointer(a_value)
    {
        hkm_assert(nullptr != this->pointer);
    }

    template<typename Other_t, typename Dummy_t = std::enable_if_t<std::is_convertible<Other_t, Type_t>::value>>
    Not_null(const Not_null<Other_t>& other)
    {
        *this = other;
    }

    template<typename Other_t, typename Dummy_t = std::enable_if_t<std::is_convertible<Other_t, Type_t>::value>>
    Not_null& operator=(const Not_null<Other_t>& other)
    {
        this->pointer = other.get();
        return *this;
    }

    Not_null(const Not_null&) = default;
    Not_null& operator=(const Not_null&) = default;

    Not_null(std::nullptr_t)  = delete;
    Not_null(int)             = delete;
    Not_null<Type_t>& operator=(std::nullptr_t) = delete;
    Not_null<Type_t>& operator=(int) = delete;

    Type_t get() const
    {
        return this->pointer;
    }

    operator Type_t() const
    {
        return this->get();
    }

    Type_t operator->() const
    {
        return this->get();
    }

    bool operator==(const Type_t& a_other) const
    {
        return this->pointer == a_other;
    }

    bool operator!=(const Type_t& a_other) const
    {
        return false == (*this == a_other);
    }

private:
    Not_null<Type_t>& operator++()            = delete;
    Not_null<Type_t>& operator--()            = delete;
    Not_null<Type_t> operator++(int)          = delete;
    Not_null<Type_t> operator--(int)          = delete;
    Not_null<Type_t>& operator+(std::size_t)  = delete;
    Not_null<Type_t>& operator+=(std::size_t) = delete;
    Not_null<Type_t>& operator-(std::size_t)  = delete;
    Not_null<Type_t>& operator-=(std::size_t) = delete;

    Type_t pointer;
};
} // namespace xmcu