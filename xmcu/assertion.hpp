#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// std
#include <cstdint>

// xmcu
#include <xmcu/bit.hpp>
#include <xmcu/non_constructible.hpp>

#pragma GCC diagnostic ignored "-Wvolatile"
#if defined(XMCU_SOC_MODEL_STM32WB35CEU6A) || defined(XMCU_SOC_MODEL_STM32WB55CGU6)
#include <stm32wbxx.h>
#define XMCU_ASSERTION_TRAP_ENTER_ENABLED
#elif defined(XMCU_SOC_MODEL_STM32L010F4P6) || defined(XMCU_SOC_MODEL_STM32L010C6T6)
#include <stm32l0xx.h>
#define XMCU_ASSERTION_TRAP_ENTER_ENABLED
#endif
#pragma GCC diagnostic pop

namespace xmcu {
namespace debug {
class assertion : private non_constructible
{
public:
#if defined(XMCU_ASSERTION_TRAP_ENTER_ENABLED)
    enum class Trap_enter_mode : std::uint32_t
    {
        disabled,
        enabled
    };
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
#if defined(XMCU_ASSERTION_TRAP_ENTER_ENABLED)
                       ,
                       Trap_enter_mode a_trap_enter_mode
#endif
    );
    static void disable();

    static void register_print(const Print_handler& a_print);

#ifdef  XMCU_DISABLE_PRINTING_MESSAGE_ON_ASSERTION_FAIL
    static void print([[maybe_unused]] const char* a_p_file,
                      [[maybe_unused]] uint32_t a_line,
                      [[maybe_unused]] const char* a_p_expression)
    {
        return;
    }
#else
    static void print(const char* a_p_file, uint32_t a_line, const char* a_p_expression);
#endif
    static void halt();

#if defined(XMCU_ASSERTION_TRAP_ENTER_ENABLED)
    static Trap_enter_mode get_Trap_enter_mode();
#endif
};

} // namespace debug
} // namespace xmcu

#if defined(XMCU_ASSERT_ENABLED)
#if defined(XMCU_ASSERTION_TRAP_ENTER_ENABLED)
#define hkm_assert(expression)                                                                     \
    do                                                                                             \
    {                                                                                              \
        if (false == (expression))                                                                 \
        {                                                                                          \
            xmcu::debug::assertion::print(__FILE__, static_cast<uint32_t>(__LINE__), #expression); \
            if (xmcu::debug::assertion::Trap_enter_mode::enabled ==                                \
                xmcu::debug::assertion::assertion::get_Trap_enter_mode())                          \
            {                                                                                      \
                []() -> bool {                                                                     \
                    __BKPT(0);                                                                     \
                    return true;                                                                   \
                }();                                                                               \
            }                                                                                      \
            xmcu::debug::assertion::halt();                                                        \
        }                                                                                          \
    } while (false)
#else
#define hkm_assert(expression)                                                                     \
    do                                                                                             \
    {                                                                                              \
        if (false == (expression))                                                                 \
        {                                                                                          \
            xmcu::debug::assertion::print(__FILE__, static_cast<uint32_t>(__LINE__), #expression); \
            xmcu::debug::assertion::halt();                                                        \
        }                                                                                          \
    } while (false)
#endif
#else
#define hkm_assert(expression)    \
    do                            \
    {                             \
        (void)sizeof(expression); \
    } while (0)
#endif
