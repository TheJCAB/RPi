#include "HidUsbDevices.h"

#include "Cpu.h"
#include "Timer.h"

#include "emb-stdio.h"

#include <array>

namespace Keyboard
{

static uint8_t firstKbd = 0;

static uint8_t PressedKeys[6]{};
static bool KeyStates[256]{};

struct KeyModifiers
{
    bool LeftCtrl   : 1;
    bool LeftShift  : 1;
    bool LeftAlt    : 1;
    bool LeftGui    : 1;
    bool RightCtrl  : 1;
    bool RightShift : 1;
    bool RightAlt   : 1;
    bool RightGui   : 1;
};

KeyModifiers ModifierStates{};

constexpr auto CharToKeyCode = [](char c) constexpr
{
    std::array<uint8_t, 256> keyCodes{};
    for (auto& keyCode : keyCodes) keyCode = 1; // Default to rollover error which will never appear pressed
    for (char c = 'a'; c <= 'z'; ++c) keyCodes[c] = c - 'a' + 4; // 4 is the first key code for letters
    for (char c = 'A'; c <= 'Z'; ++c) keyCodes[c] = c - 'A' + 4; // Same for uppercase
    for (char c = '0'; c <= '9'; ++c) keyCodes[c] = c - '0' + 30; // 30 is the first key code for numbers
    keyCodes[' '] = 44; // Space
    keyCodes['\n'] = 40; // Enter
    keyCodes['\r'] = 40; // Enter
    keyCodes['\t'] = 43; // Tab
    return keyCodes;
};

void Init()
{
    // Detect the first keyboard on USB bus
    for (int i = 1; i <= MaximumDevices; i++)
    {
        if (IsKeyboard(i)) {
            firstKbd = i;
            break;
        }
    }
    if (firstKbd)
    {
        printf2("Keyboard detected\r\n");
        HIDEnableInterruptINSimple(firstKbd, 0);
        printf2("Keyboard configured\r\n");
    }
}

static void RefreshStateIfNeeded()
{
    if (firstKbd == 0)
    {
        return;
    }

    auto const time = Cpu::GetPerformanceCounter();
    static uint64_t nextRefresh = 0;
    if (time >= nextRefresh)
    {
        uint16_t const USB_HID_REPORT_TYPE_INPUT = 1;
        std::byte buf[8];
        auto const status = HIDReadInterruptReport(firstKbd, 0, buf, sizeof(buf), nullptr);
        //auto const status = HIDReadReport(firstKbd, 0, USB_HID_REPORT_TYPE_INPUT << 8 | 1, &buf[0], 8);
        if (status == RESULT::Ok)
        {
            nextRefresh = time + 10'000; // Refresh every 10 ms

            //printf("HID KBD REPORT: %08b %02X %02X %02X %02X %02X %02X\n",
            //    buf[0],         buf[2], buf[3],
            //    buf[4], buf[5], buf[6], buf[7]);

            ModifierStates = reinterpret_cast<KeyModifiers&>(buf[0]);

            // Clear previous key states
            for (int i = 0; i < 6; ++i)
            {
                // Don't clear keys where the report contains rollover errors (too many keys pressed).
                if (buf[i + 2] != std::byte{1})
                {
                    KeyStates[PressedKeys[i]] = false;
                    PressedKeys[i] = 0;
                }
            }
            for (int i = 0; i < 6; ++i)
            {
                auto const key = static_cast<uint8_t>(buf[i + 2]);
                
                // Ignore rollover errors
                if (key == 1) continue;

                PressedKeys[i] = key;

                KeyStates[key] = true;
            }
        }
        else
        {
            nextRefresh = time + 1'000; // Try again in 1ms
        }
    }
}

bool IsKeyPressed(char key)
{
    RefreshStateIfNeeded();
    return KeyStates[CharToKeyCode(key)[static_cast<uint8_t>(key)]];
}

}
// namespace Keyboard
