#pragma once

#include "Exception.h"

#include <stdint.h>
#include <stddef.h>

namespace Interrupts
{

using Spark           = Exception::Spark;
using HandlerFunction = Exception::HandlerFunction;

void EnableUsb(HandlerFunction);

void EnableCoreVirtualTimerInterrupt(HandlerFunction handler);
void DisableCoreVirtualTimerInterrupt();

}
// namespace Interrupts
