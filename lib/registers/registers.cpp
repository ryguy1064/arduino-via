#include "registers.h"

device_register_t *registerList = NULL;

static uint8_t defaultReadHandler(struct _device_register_t *reg)
{
    return reg->data;
}

static void defaultWriteHandler(struct _device_register_t *reg, uint8_t data)
{
    reg->data = data;
}

device_register_t * addRegister(uint8_t address, read_func_t readFunc, write_func_t writeFunc, bool noDefaultFunc, void *context)
{
    device_register_t *r = registerList;

    // Special case for first register added
    if (registerList == NULL)
    {
        registerList = (device_register_t *)malloc(sizeof(device_register_t));
        r = registerList;
    }
    else
    {
        // Advance to last register in list
        while (r->nextReg != NULL)
        {
            r = r->nextReg;
        }

        // Create new register at the end
        r->nextReg = (device_register_t *)malloc(sizeof(device_register_t));
        r = r->nextReg;
    }

    // Initialize register params
    r->address = address;
    r->readHandler = readFunc;
    if (r->readHandler == NULL)
    {
        r->readHandler = (noDefaultFunc) ? NULL : defaultReadHandler;
    }
    r->writeHandler = writeFunc;
    if (r->writeHandler == NULL)
    {
        r->writeHandler = (noDefaultFunc) ? NULL : defaultWriteHandler;
    }
    r->data = 0;
    r->context = context;
    r->nextReg = NULL;

    return r;
}

uint8_t handleRegisterRead(uint8_t address)
{
    uint8_t data = 0;

    for (device_register_t *r = registerList; r != NULL; r = r->nextReg)
    {
        if (r->address == address)
        {
            if (r->readHandler != NULL)
            {
                data = r->readHandler(r);
            }
            break;
        }
    }

    return data;
}

void handleRegisterWrite(uint8_t address, uint8_t data)
{
    for (device_register_t *r = registerList; r != NULL; r = r->nextReg)
    {
        if (r->address == address)
        {
            if (r->writeHandler != NULL)
            {
                r->writeHandler(r, data);
            }
            break;
        }
    }
}