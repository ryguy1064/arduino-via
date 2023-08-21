#ifndef REG_IRQ_H
#define REG_IRQ_H

#include <Arduino.h>
#include "registers.h"

typedef union
{
    struct
    {
        uint8_t serialRxDataAvailable       : 1;
        uint8_t reserved                    : 7;
    } flags;
    uint8_t all;
} irq_flags_t;

inline uint8_t reg1_IrqFlagsRead(struct _device_register_t *reg)
{
    uint8_t flagsPrev = reg->data;

    // Reset all IRQ flags
    reg->data = 0;

    return flagsPrev;
}

// inline void reg1_IrqFlagsWrite(struct _device_register_t *reg, uint8_t data)
// {
//     // IRQ Flags is a readonly register. Do nothing.
// }

#endif // #ifndef REG_SERIAL_H
