#pragma once

#include "UsbSpec.h"

#include <stdint.h>

struct PACKED UsbPipe
{
    uint16_t MaxPacketSizeInBytes;
    UsbSpeed Speed;
    uint8_t EndPoint;
    uint8_t Number;
    uint8_t splitNodePort;
    uint8_t splitNodePoint;
};
