#pragma once

#include <stdint.h>

struct Color565
{
    uint16_t Blue  : 5;
    uint16_t Green : 6;
    uint16_t Red   : 5;
};

constexpr Color565 White   = { 0x1F, 0x3F, 0x1F };
constexpr Color565 Black   = { 0x00, 0x00, 0x00 };
constexpr Color565 Red     = { 0x1F, 0x00, 0x00 };
constexpr Color565 Green   = { 0x00, 0x3F, 0x00 };
constexpr Color565 Blue    = { 0x00, 0x00, 0x1F };
constexpr Color565 Yellow  = { 0x1F, 0x3F, 0x00 };
constexpr Color565 Cyan    = { 0x00, 0x3F, 0x1F };
constexpr Color565 Magenta = { 0x1F, 0x00, 0x1F };

extern uintptr_t GpuMemBase;

namespace Framebuffer
{
    
void Init(uint32_t width, uint32_t height);

extern uint32_t Width;
extern uint32_t Height;

void Flip();

void WritePixel(uint32_t x, uint32_t y, Color565 color);

void Panic(Color565 color, int divisions = 1, int which = 0, int repeat = 1);

} // namespace Framebuffer
