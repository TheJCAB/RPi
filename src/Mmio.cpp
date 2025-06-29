#include "Mmio.h"

namespace Mmio
{

// We handle the various MMIO bases dynamically so we can relocate them via the MMU.
uintptr_t Base    = 0x3F00'0000u; // Raspberry 3 by default.
uintptr_t QA7Base = 0x4000'0000u; // Raspberry 3 by default.

}
// namespace Mmio
