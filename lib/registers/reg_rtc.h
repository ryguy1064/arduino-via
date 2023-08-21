#pragma once

#include <Arduino.h>
#include "registers.h"

#define RTC_BASE_ADDRESS    3

static uint32_t msSnapshot = 0;

inline uint8_t regN_RtcRead(struct _device_register_t *reg)
{
    uint8_t data;

    switch (reg->address - RTC_BASE_ADDRESS)
    {
    case 0:
        msSnapshot = millis();
        // Serial.println(msSnapshot, HEX);
        data = (msSnapshot >> 0) & 0xFF;
        break;
    case 1:
        data = (msSnapshot >> 8) & 0xFF;
        break;
    case 2:
        data = (msSnapshot >> 16) & 0xFF;
        break;
    case 3:
        data = (msSnapshot >> 24) & 0xFF;
        break;
    default:
        break;
    }

    return data;
}

// inline void regN_RtcWrite(struct _device_register_t *reg, uint8_t data)
// {
//     // RTC is a readonly register. Do nothing.
// }