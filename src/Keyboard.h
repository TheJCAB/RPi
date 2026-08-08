#pragma once

struct UsbDriver;

namespace Keyboard
{

void Init(UsbDriver&);

bool IsKeyPressed(UsbDriver&, char key);

}
// namespace Keyboard
