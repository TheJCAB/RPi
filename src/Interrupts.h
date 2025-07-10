
namespace Interrupts
{

using HandlerFunction = void(*)();

void EnableUsb(HandlerFunction);

void EnableCoreVirtualTimerInterrupt(HandlerFunction handler);
void DisableCoreVirtualTimerInterrupt();

}
// namespace Interrupts
