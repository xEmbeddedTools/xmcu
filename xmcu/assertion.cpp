/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

// this
#include <xmcu/assertion.hpp>

namespace {
using namespace xmcu::debug;

assertion::Halt_hadler halt_handler;
assertion::Print_handler print_handler;
#if defined(ASSERTION_TRAP_ENTER_ENABLED)
assertion::Trap_enter_mode trap_enter_mode;
#endif
} // namespace

namespace xmcu {
namespace debug {
void assertion::enable(const Halt_hadler& a_halt
#if defined(ASSERTION_TRAP_ENTER_ENABLED)
                       ,
                       Trap_enter_mode a_trap_enter_mode
#endif
)
{
    halt_handler = a_halt;
#if defined(ASSERTION_TRAP_ENTER_ENABLED)
    trap_enter_mode = a_trap_enter_mode;
#endif
}

void assertion::disable()
{
    halt_handler  = { nullptr, nullptr };
    print_handler = { nullptr, nullptr };
}

void assertion::register_print(const Print_handler& a_print) {
    print_handler = a_print;
}

void assertion::print(const char* a_p_file, uint32_t a_line, const char* a_p_expression)
{
    if (nullptr != print_handler.p_function)
    {
        print_handler.p_function(a_p_file, a_line, a_p_expression, print_handler.p_user_data);
    }
}

void assertion::halt()
{
    if (nullptr != halt_handler.p_function)
    {
        halt_handler.p_function(halt_handler.p_user_data);
    }
}

#if defined(ASSERTION_TRAP_ENTER_ENABLED)
assertion::Trap_enter_mode assertion::get_Trap_enter_mode()
{
    return trap_enter_mode;
}
#endif
} // namespace debug
} // namespace xmcu
