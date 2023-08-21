#ifndef REG_LCD_H
#define REG_LCD_H

#include <Arduino.h>
#include "LiquidCrystal_I2C.h"
#include "registers.h"

// uint8_t lcdDataToSend;
// bool needToSendLcdData = false;

inline void reg_LcdWrite(struct _device_register_t *reg, uint8_t data)
{
    auto lcd = (LiquidCrystal_I2C *)reg->context;

    switch (data)
    {
    case 0:
    case 1:
    case 2:
    case 3:
        lcd->setCursor(0, data);
        break;

    case 4:
        lcd->clear();
        break;

    default:
        // Only send selected characters, for now
        if (data >= ' ' && data <= '~')
        {
            lcd->write(data);
        }
        break;
    }
}

#endif // #ifndef REG_LCD_H
