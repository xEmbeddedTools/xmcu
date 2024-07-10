# Coding Standards

## Names

### Snake Case
- Write all names in snake_case (lowercase letters with underscores separating words).

### Argument Names
- Start all argument names with `a_`.

### Pointer Names
- Start all pointer names with `p_`.

### Global Variables/Constants
- Start all global variables/constants with `g_`.

### File-Scoped Variables/Constants
- Start all variables/constants that are global within a file with `f_`.

### Class/Struct/Union Names
- Start all class, struct, or union names with a capital letter if there is a possibility to create an object of this type.
- Start all other case names with a lowercase letter.

## Class/Struct/Union

### Access Specifiers
- Use only one section per access specifier (`public`, `protected`, `private`).
- Order access specifiers as follows: `public` first, `protected` second, `private` last.

### Rule of 5 or Rule of 0
- Follow the Rule of 5 or the Rule of 0 for special member functions.

### Member Naming
- Do not use `m_` prefix for members. Use `this->` to reference members.

### Initialization List
- Maintain the same order in the initialization list as in the declaration in the class.
- Do not use members to create another in the initialization list.

## Standard Library Usage

### Use of Standard Libraries
- Prefer standard libraries over custom implementations for common functionalities to ensure code reliability and maintainability.
- Include necessary headers for the standard libraries you are using.
- Avoid using deprecated standard library features.
- Do not use `using namespace std;`.
- Use `std` types for `cstdint` or `stdint.h`.

## Miscellaneous

- Do not use `goto`.
- Use `auto` only if it is necessary.
- `using namespace` in `.hpp` files is not allowed.
- Add a comment to the end of a namespace (`// namespace name`).
- Function arguments can be pointers if they are modified, and const references if they are not modified.
- Use Yoda notation for comparisons (e.g., `if (5 == x)`).
- Use `const` always if you can.
- Group includes. Includes should be sorted inside each group.
- Write one-line `if` statements with brackets.