#ifndef REG_SERIAL_H
#define REG_SERIAL_H

#include <Arduino.h>
#include "registers.h"

char charToSend;
bool needToSendChar = false;

// Register 0: Serial Data
inline uint8_t reg0_SerialRead(struct _device_register_t *reg)
{
    if (Serial.available())
    {
        return Serial.read();
    }

    return 0;
}

inline void reg0_SerialWrite(struct _device_register_t *reg, uint8_t data)
{
    charToSend = (char)data;
    needToSendChar = true;
    // Serial.write((char)data);
}

#endif // #ifndef REG_SERIAL_H
