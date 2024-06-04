#pragma once

/**/

// externals
#pragma GCC diagnostic ignored "-Wvolatile"
#include <stm32wbxx.h>
#pragma GCC diagnostic pop

// std
#include <cstdint>

// xmcu
#include <xmcu/Non_constructible.hpp>
#include <xmcu/Non_copyable.hpp>
#include <xmcu/bit_flag.hpp>
#include <xmcu/various.hpp>
#include <xmcu/soc/ST/m4/stm32wb/rm0434/rcc.hpp>

namespace xmcu {
namespace soc {
namespace m4 {
namespace stm32wb {
template<typename Perihperal_t = void*> class DMA : private xmcu::Non_copyable
{
public:
    enum class Priority : std::uint32_t
    {
        very_high = DMA_CCR_PL_0 | DMA_CCR_PL_1,
        high      = DMA_CCR_PL_1,
        medium    = DMA_CCR_PL_0,
        low       = 0x0u
    };
    enum class Mode : std::uint32_t
    {
        single   = 0x0u,
        circular = DMA_CCR_CIRC
    };
    enum class Channel : std::uint32_t
    {
        _1,
        _2,
        _3,
        _4,
        _5,
        _6,
        _7
    };
    enum class Event_flag : std::uint32_t
    {
        none                   = 0x0u,
        full_transfer_complete = 0x1u,
        half_transfer_complete = 0x2u,
        transfer_error         = 0x4u
    };

    struct Result
    {
        Event_flag event                 = various::get_enum_incorrect_value<Event_flag>();
        std::size_t data_length_in_words = 0;
    };
    struct Callback
    {
        using Function = void (*)(Event_flag a_event, void* a_p_user_data);

        Function function = nullptr;
        void* p_user_data = nullptr;
    };
};

constexpr DMA<>::Event_flag operator|(DMA<>::Event_flag a_f1, DMA<>::Event_flag a_f2)
{
    return static_cast<DMA<>::Event_flag>(static_cast<std::uint32_t>(a_f1) | static_cast<std::uint32_t>(a_f2));
}

constexpr DMA<>::Event_flag operator&(DMA<>::Event_flag a_f1, DMA<>::Event_flag a_f2)
{
    return static_cast<DMA<>::Event_flag>(static_cast<std::uint32_t>(a_f1) & static_cast<std::uint32_t>(a_f2));
}

constexpr DMA<>::Event_flag operator|=(DMA<>::Event_flag& a_f1, DMA<>::Event_flag a_f2)
{
    a_f1 = a_f1 | a_f2;
    return a_f1;
}

template<> class rcc<DMA<>, 1> : private xmcu::Non_constructible
{
public:
    static void enable()
    {
        bit_flag::set(&(RCC->AHB1ENR), RCC_AHB1ENR_DMAMUX1EN);
        bit_flag::set(&(RCC->AHB1ENR), RCC_AHB1ENR_DMA1EN);
    }
    static void disable()
    {
        bit_flag::clear(&(RCC->AHB1ENR), RCC_AHB1ENR_DMA1EN);
        bit_flag::clear(&(RCC->AHB1ENR), RCC_AHB1ENR_DMAMUX1EN);
    }
};

template<> class rcc<DMA<>, 2> : private xmcu::Non_constructible
{
public:
    static void enable()
    {
        bit_flag::set(&(RCC->AHB1ENR), RCC_AHB1ENR_DMA2EN);
    }
    static void disable()
    {
        bit_flag::clear(&(RCC->AHB1ENR), RCC_AHB1ENR_DMA2EN);
    }
};
} // namespace stm32wb
} // namespace m4
} // namespace soc
} // namespace xmcu