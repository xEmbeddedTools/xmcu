#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// xmcu
#include <xmcu/bit_flag.hpp>
#include <xmcu/non_constructible.hpp>

#pragma GCC diagnostic ignored "-Wvolatile"
#if defined(STM32WB)
#include <stm32wbxx.h>
#define ASSERTION_TRAP_ENTER_ENABLED
#elif defined(STM32L0)
#include <stm32l0xx.h>
#define ASSERTION_TRAP_ENTER_ENABLED
#endif
#pragma GCC diagnostic pop

namespace xmcu {
namespace debug {
class assertion : private non_constructible
{
public:
#if defined(ASSERTION_TRAP_ENTER_ENABLED)
    enum class Trap_enter_mode : std::uint32_t { disabled, enabled };
#endif
    struct Halt_hadler
    {
        using Function = void (*)(void* a_p_user_data);

        Function p_function = nullptr;
        void* p_user_data = nullptr;
    };

    struct Print_handler
    {
        using Function = void (*)(const char* a_p_file,
                                  uint32_t a_line,
                                  const char* a_p_expression,
                                  void* a_p_user_data);

        Function p_function = nullptr;
        void* p_user_data = nullptr;
    };

    static void enable(const Halt_hadler& a_halt
#if defined(ASSERTION_TRAP_ENTER_ENABLED)
                       ,
                       Trap_enter_mode a_trap_enter_mode
#endif
    );
    static void disable();

    static void register_print(const Print_handler& a_print);

    static void print(const char* a_p_file, uint32_t a_line, const char* a_p_expression);
    static void halt();

#if defined(ASSERTION_TRAP_ENTER_ENABLED)
    static Trap_enter_mode get_Trap_enter_mode();
#endif
};

} // namespace debug
} // namespace xmcu

#if defined(HKM_ASSERT_ENABLED)
#if defined(ASSERTION_TRAP_ENTER_ENABLED)
#define hkm_assert(expression)                                                                                    \
    do                                                                                                            \
    {                                                                                                             \
        if (false == (expression))                                                                                \
        {                                                                                                         \
            xmcu::debug::assertion::print(__FILE__, static_cast<uint32_t>(__LINE__), #expression);                      \
            if (xmcu::debug::assertion::Trap_enter_mode::enabled == xmcu::debug::assertion::assertion::get_Trap_enter_mode()) \
            {                                                                                                     \
                []() -> bool {                                                                                    \
                    __BKPT(0);                                                                                    \
                    return true;                                                                                  \
                }();                                                                                              \
            }                                                                                                     \
            xmcu::debug::assertion::halt();                                                                             \
        }                                                                                                         \
    } while (false)
#else
#define hkm_assert(expression)                                                               \
    do                                                                                       \
    {                                                                                        \
        if (false == (expression))                                                           \
        {                                                                                    \
            xmcu::debug::assertion::print(__FILE__, static_cast<uint32_t>(__LINE__), #expression); \
            xmcu::debug::assertion::halt();                                                        \
        }                                                                                    \
    } while (false)
#endif
#else
#define hkm_assert(expression)    \
    do                            \
    {                             \
        (void)sizeof(expression); \
    } while (0)
#endif
