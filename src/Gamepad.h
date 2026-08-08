#include "stdint.h"

struct UsbDriver;

namespace Gamepad
{

void Init(UsbDriver&);

enum class Button : uint8_t
{
    A,
    B,
    X,
    Y,
    LeftBumper,
    RightBumper,
    LeftTrigger,
    RightTrigger,
    Back,
    Start,
    Center,
    LeftStick,
    RightStick,
    DPadUp,
    DPadDown,
    DPadLeft,
    DPadRight,
    AnyUp,
    AnyDown,
    AnyLeft,
    AnyRight,

    // Total number of buttons
    Count
};

enum class Axis : uint8_t
{
    LeftX,
    LeftY,
    RightX,
    RightY,

    LeftTrigger,
    RightTrigger,

    // Total number of axes
    Count
};

bool IsButtonPressed(UsbDriver&, Button);
int16_t GetAxis(UsbDriver&, Axis);

}
// namespace Gamepad
