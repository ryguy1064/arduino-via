#ifndef REGISTERS_H
#define REGISTERS_H

#include <Arduino.h>

struct _device_register_t;
typedef uint8_t (*read_func_t)(struct _device_register_t *reg);
typedef void (*write_func_t)(struct _device_register_t *reg, uint8_t data);

typedef struct _device_register_t
{
    uint8_t address;
    read_func_t readHandler;
    write_func_t writeHandler;
    uint8_t data;
    struct _device_register_t *nextReg;
    void *context;
} device_register_t;

uint8_t handleRegisterRead(uint8_t address);
void handleRegisterWrite(uint8_t address, uint8_t data);
device_register_t * addRegister(uint8_t address, read_func_t readFunc = NULL, write_func_t writeFunc = NULL, bool noDefaultFunc = false, void *context = NULL);

#endif // #ifndef REGISTERS_H