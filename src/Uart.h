#pragma once

#include <stdint.h>
#include <stddef.h>

namespace Uart
{

void Init();
void Putc(char c);
void Puts(char const* str);

}
// namespace Uart
