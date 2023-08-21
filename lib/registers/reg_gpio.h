#pragma once

#include <Arduino.h>
#include "registers.h"

inline uint8_t reg_gpioIORead(struct _device_register_t *reg)
{
    uint8_t gpioData;

    // Get GPIO pin input data
    gpioData = PINC & 0x07;
    gpioData |= ((PINB & 0x3C) << 1);

    return gpioData;
}

inline void reg_gpioIOWrite(struct _device_register_t *reg, uint8_t data)
{
    // Set output data to GPIO pins
    PORTC = (PORTC & ~0x07) | (data & 0x07);
    PORTB = (PORTB & ~0x3C) | ((data & 0x78) >> 1);

    // Serial.println(PORTC, BIN);
    // Serial.println(PORTB, BIN);
}

inline uint8_t reg_gpioDirRead(struct _device_register_t *reg)
{
    return reg->data;
}

inline void reg_gpioDirWrite(struct _device_register_t *reg, uint8_t data)
{
    reg->data = data;

    // Set dir data to GPIO pins
    DDRC = (DDRC & ~0x07) | (data & 0x07);
    DDRB = (DDRB & ~0x3C) | ((data & 0x78) >> 1);

    // Serial.println(DDRC, BIN);
    // Serial.println(DDRB, BIN);
}