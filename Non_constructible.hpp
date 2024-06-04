#pragma once

/**/

namespace xmcu {
class Non_constructible
{
protected:
    Non_constructible() = delete;
    Non_constructible(const Non_constructible&) = delete;
    Non_constructible(Non_constructible&&) = delete;

    Non_constructible& operator=(const Non_constructible&) = delete;
    Non_constructible& operator=(Non_constructible&&) = delete;
};
} // namespace xmcu