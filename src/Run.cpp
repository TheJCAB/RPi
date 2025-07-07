#include "Run.h"
#include "Mmio.h"
#include "Uart.h"
#include "Timer.h"
#include "Framebuffer.h"
#include "Keyboard.h"
#include "Gamepad.h"

#include "emb-stdio.h"

uint32_t rand()
{
    static uint32_t seed = 0xDEADBEEF; // Initial seed value
    seed = (seed * 1103515245 + 12345) & 0xFFFFFFFF; // Linear congruential generator
    return seed % 0xFFFFFFFF; // Return a pseudo-random number
}

//#include <random>

void Flashing()
{
    Keyboard::Init();

    Color565 background = Magenta;
    for (;;)
    {
        for (int i = 0; i < 2; ++i)
        {
            if (Keyboard::IsKeyPressed(' '))
            {
                background = Blue;
            }
            else
            {
                background = Magenta;
            }

            Timer::Delay(33'333); // Delay for 33.333 ms (30 FPS)
            for (uint32_t x = 0; x < Framebuffer::Width; ++x)
            {
                for (uint32_t y = 0; y < Framebuffer::Height; ++y)
                {
                    // Set pixel color (example: green)
                    if (x < 8)
                    {
                        Framebuffer::WritePixel(x, y, Blue);
                    }
                    else if (y < 8)
                    {
                        Framebuffer::WritePixel(x, y, Cyan);
                    }
                    else if (x >= Framebuffer::Width - 8)
                    {
                        Framebuffer::WritePixel(x, y, Red);
                    }
                    else if (y >= Framebuffer::Width - 8)
                    {
                        Framebuffer::WritePixel(x, y, Yellow);
                    }
                    else
                    {
                        Framebuffer::WritePixel(x, y, i == 0 ? Green : background);
                    }
                }
            }
            Framebuffer::Flip();
            //Uart::Puts("Framebuffer flipped.\n");
        }
    }
}

struct Cell
{
    uint32_t x;
    uint32_t y;

    friend bool operator==(const Cell& a, const Cell& b) = default;
};

constexpr uint32_t cellWidth  = 32;
constexpr uint32_t cellHeight = 32;

uint32_t gridWidth  = Framebuffer::Width  / cellWidth;
uint32_t gridHeight = Framebuffer::Height / cellHeight;

Cell head;
Cell tail[100];
uint32_t tailLength = 0;

Cell fruit;

enum Direction
{
    None,
    Up,
    Down,
    Left,
    Right
};
Direction direction = None;

uint64_t nextSimulationTime = 0;

void SpawnFruit()
{
    do
    {
        fruit.x = rand() % (gridWidth  - 2) + 1;
        fruit.y = rand() % (gridHeight - 2) + 1;
    } while (fruit == head);
}

void ResetSnake()
{
    gridWidth  = Framebuffer::Width  / cellWidth;
    gridHeight = Framebuffer::Height / cellHeight;

    head = Cell{ gridWidth / 2, gridHeight / 2 };
    tailLength = 0;
    direction = None;

    SpawnFruit();

    nextSimulationTime = 0;
}

void FlashScreen(Color565 color)
{
    Framebuffer::WriteRectangle(0, 0, Framebuffer::Width, Framebuffer::Height, color);
    Framebuffer::Flip();
    Timer::Delay(33'333); // Flash for 33.333 ms (30 FPS)
}

void Snake()
{
    Keyboard::Init();
    Gamepad::Init();

    using namespace Framebuffer;

    ResetSnake();

    for (;;)
    {
        if (Keyboard::IsKeyPressed('d') || Gamepad::IsButtonPressed(Gamepad::Button::AnyRight))
        {
            direction = Right;
        }
        if (Keyboard::IsKeyPressed('a') || Gamepad::IsButtonPressed(Gamepad::Button::AnyLeft))
        {
            direction = Left;
        }
        if (Keyboard::IsKeyPressed('s') || Gamepad::IsButtonPressed(Gamepad::Button::AnyDown))
        {
            direction = Down;
        }
        if (Keyboard::IsKeyPressed('w') || Gamepad::IsButtonPressed(Gamepad::Button::AnyUp))
        {
            direction = Up;
        }

        uint64_t time = Timer::GetPerformanceCounter();
        if (time >= nextSimulationTime)
        {
            nextSimulationTime = time + Timer::GetPerformanceFrequency() / 10;
            if (tailLength > 0)
            {
                // Move the tail
                for (uint32_t i = tailLength - 1; i > 0; --i)
                {
                    tail[i] = tail[i - 1];
                }
                tail[0] = head; // The first element of the tail is now the head
            }

            if (direction == Up)
            {
                head.y -= 1;
            }
            else if (direction == Down)
            {
                head.y += 1;
            }
            else if (direction == Left)
            {
                head.x -= 1;
            }
            else if (direction == Right)
            {
                head.x += 1;
            }

            // Reset the game if the head goes out of bounds
            if (head.x <= 0 || head.x >= gridWidth  - 1||
                head.y <= 0 || head.y >= gridHeight - 1)
            {
                FlashScreen(Red);
                ResetSnake();
            }

            for (uint32_t i = 0; i < tailLength; ++i)
            {
                if (tail[i] == head)
                {
                    FlashScreen(Red);
                    ResetSnake();
                    break;
                }
            }

            if (head == fruit)
            {
                // Generate a new fruit position
                do
                {
                    fruit.x = rand() % (gridWidth  - 2) + 1;
                    fruit.y = rand() % (gridHeight - 2) + 1;
                } while (fruit == head);

                tail[tailLength++] = head; // Add the head to the tail
            }
        }

        WriteRectangle(0                          , 0                            , Width    , cellHeight, Red);
        WriteRectangle(0                          , (gridHeight - 1) * cellHeight, Width    , Height - (gridHeight - 1) * cellHeight, Red);
        WriteRectangle(0                          , 0                            , cellWidth, Height    , Red);
        WriteRectangle((gridWidth - 1) * cellWidth, 0                            , cellWidth, Height    , Red);

        WriteRectangle(cellWidth, cellHeight, (gridWidth - 2) * cellWidth, (gridHeight - 2) * cellHeight, Black);

        WriteRectangle(fruit.x * cellWidth, fruit.y * cellHeight, cellWidth, cellHeight, Yellow);

        for (uint32_t i = 0; i < tailLength; ++i)
        {
            WriteRectangle(tail[i].x * cellWidth, tail[i].y * cellHeight, cellWidth, cellHeight, Green);
        }
        WriteRectangle(head.x * cellWidth, head.y * cellHeight, cellWidth, cellHeight, Green);

        Framebuffer::Flip();
        Timer::Delay(33'333);
    }
}

void Run()
{
    Snake();
}
