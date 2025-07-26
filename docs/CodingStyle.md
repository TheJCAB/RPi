# RPi Kernel Coding Style Guide

This document describes the coding conventions used in the RPi bare-metal kernel project.

Human beings are expected to express their own voice and use these as a guideline.

AI agents must adhere to these guidelines strictly for any new code.


## Language standard

This codebase uses C++20.

Code should be kept loose. Prefer free functions to classes and class member functions unless classes keep the code simpler.

## File Structure

### Header Guards
- Use `#pragma once` instead of traditional include guards

### Include Order
1. Own header file (for .cpp files)
2. Blank line
3. Project headers in quotes (e.g., `#include "Cpu.h"`)
4. Blank line  
5. System headers in angle brackets (e.g., `#include <stdint.h>`)
6. Blank line
7. Standard library headers in angle brackets (e.g., `#include <atomic>`)

### Example:
```cpp
#include "Timer.h"

#include "Cpu.h"
#include "Mmio.h"
#include "Uart.h"

#include <stdint.h>
#include <stddef.h>

#include <atomic>
```

## Namespaces

### Structure
- Each module should have its own namespace (e.g., `Timer`, `Containers`, `Heap`)
- Opening brace on its own line
- Closing brace on its own line
- Namespace closing comment on separate line after closing brace

```cpp
namespace Heap
{

// Content here

}
// namespace Heap
```

## Naming Conventions

### Variables and Functions
- **Functions**: PascalCase (e.g., `FindNextScheduledTimerTriggerTime()`)
- **Local variables**: camelCase (e.g., `triggerTimeTicks`, `oldVal`)
- **Structure fields**: PascalCase (e.g., `Head`, `Tail`, `DataInfo`)
- **Class member variables**: m_camelCase (e.g., `m_head`, `m_tail`, `m_dataInfo`)
- **Constants**: PascalCase (e.g., `MaxScheduledTimers`)
- **Template parameters**: PascalCase (e.g., `typename TimeType`)
- **Macros**: UPPER_CASE (e.g., `MAX_SCHEDULED_TIMERS`) Use only when strictly needed.

### Types
- **Structs/Classes**: PascalCase (e.g., `ScheduledTimer`, `CircularFifo`)
- **Enums**: PascalCase for enum name, PascalCase for values, use `enum class` and specify the underlying type.
- **Type aliases**: PascalCase

## Formatting

### Braces and Spacing
- Opening brace on new line for functions, classes, structs, namespaces
- Opening brace on new line for control structures (if, for, while)
- Always use braces, even for single-line blocks

```cpp
void Function()
{
    if (condition)
    {
        DoSomething();
    }
}
```

### Indentation
- Use 4 spaces for indentation (no tabs)
- Align related elements carefully (preserve careful tabulation)

### Variable Declarations
- Use `auto const` for immutable values when type is obvious
- Use "east const" (e.g. `int const` but not `const int`)
- Prefer using sized integral types.
- Prefer unsigned integral types unless signedness is useful or makes sense for the domain represented.
- Explicit types when clarity is important
- Initialize variables at declaration when possible

```cpp
auto const mask = UINT64_MAX >> highestPos;
uint64_t const val64 = static_cast<uint8_t>(val) * 0x0101'0101'0101'0101ull;
```

### Comments
- Use `//` for single-line comments
- Place comments in their own line before the code, function or type they describe
- Place comments in the same line of the variable, field, data member or parameter they describe
- Align inline comments when they form a logical group

```cpp
// Structure to hold scheduled callback information
struct ScheduledTimer
{
    Cpu::PerformanceTime          trigger_time_ticks;  // Absolute time when callback should be triggered
    std::atomic<Scheduler::Spark> Spark;               // Function to call
};
```

## Function Definitions

### Parameter Formatting
- Place each parameter on same line if they fit
- Use `const&` for read-only objects
- Use `const` for parameters in function definitions
- Use descriptive parameter names
- Don't use parameter names in function prototypes where the type name is sufficiently descriptive.

```cpp
SparkHandle ScheduleSparkAtTime(Cpu::PerformanceTime, Scheduler::Spark const&);
SparkHandle ScheduleSparkAtTime(Cpu::PerformanceTime const absoluteTime, Scheduler::Spark const& spark)
{
    ...
}
```

### Return Types
- Place return type on same line as function name
- Use trailing return type syntax sparingly, only when necessary

## Templates

### Formatting
- Use spaces around angle brackets for readability
- Align template parameters when they're complex

```cpp
template < typename T, uint32_t capacity, char const* name = CircularFifoName >
struct CircularFifo
```

## Constants and Literals

### Numeric Constants
- Use `constexpr` for compile-time constants
- Use digit separators for large numbers (0x0101'0101'0101'0101u). 3 digits for decimals, 4 digits for hexadecimals and binaries
- Prefer strongly-typed constants over #define

```cpp
constexpr uint32_t MaxScheduledTimers = 64;
uint64_t const val64 = static_cast<uint8_t>(val) * 0x0101'0101'0101'0101ull;
```

## Control Flow

### Loops and Conditionals
- Always use braces
- Place opening brace on new line
- Prefer early returns to reduce nesting

```cpp
for (size_t i = 0; i < MaxScheduledTimers; ++i)
{
    if (!CoreData[coreId].ScheduledTimers[i])
    {
        continue;
    }
    // Process timer
}
```

## Assembly Code

### Inline Assembly
- Use proper constraints and clobbers
- Add comments explaining the assembly operations
- Use meaningful variable names for outputs

```cpp
asm volatile(
    "1: ldaxr  %0, [%3]\n"         // Load exclusive with acquire
    "   cmp    %0, %4\n"           // Compare with expected
    "   b.ne   2f\n"               // Branch if not equal
    "   stlxr  %w1, %5, [%3]\n"    // Store exclusive with release
    "   cbnz   %w1, 1b\n"          // Retry if store failed
    : "=&r"(old_val), "=&r"(success)
    : "r"(ptr), "r"(*expected), "r"(desired)
    : "memory", "cc"
);
```

## Error Handling

### Assertions and Panics
- Use `Cpu::Panic()` for unrecoverable errors with descriptive messages
- Include context in error messages (e.g., container name)

```cpp
if (Head == Tail)
{
    Cpu::Panic("%s overflow", name);
}
```

## Memory Management

### Pointers and References
- Prefer references over pointers when object must exist
- Use `const` liberally for immutable data
- Use `restrict` keyword for C-style functions when appropriate

```cpp
void* memcpy(void* restrict s1, const void* restrict s2, size_t n)
```

## File Organization

### Header Files (.h)
- Keep implementation details private
- Use forward declarations when possible
- Include minimal necessary headers

### Source Files (.cpp)
- Include own header first
- Group related functions together
- Use static for internal linkage functions

```cpp
static void DumpMMUState()
{
    // Internal implementation
}
```

## Attributes and Specifiers

### Function Attributes
- Use `[[nodiscard]]` for functions whose return value shouldn't be ignored
- Use `noexcept` for functions that don't throw
- Use `constexpr` for functions that can be evaluated at compile time

### Alignment
- Use `alignas()` when specific alignment is required
- Document why specific alignment is needed

```cpp
struct alignas(16) Word {
    uint64_t low;
    uint64_t high;
};
```

