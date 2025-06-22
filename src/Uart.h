#pragma once

#include <stdint.h>
#include <stddef.h>

namespace Uart
{

void Init();
void Putc(char c);
char Getc();
char TryGetc();
void Puts(char const* str);
void PutHex(auto value);
void PutBin(auto value);
void PutDec(auto value);

}
// namespace Uart
