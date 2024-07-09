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

## Standard Library Usage

### Use of Standard Libraries
- Prefer standard libraries over custom implementations for common functionalities to ensure code reliability and maintainability.
- Include necessary headers for the standard libraries you are using.
- Avoid using deprecated standard library features.
- Do not use `using namespace std;`.
- Use `std` types for `cstdint` or `stdint.h`.
