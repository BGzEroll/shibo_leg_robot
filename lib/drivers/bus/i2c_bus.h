#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <Arduino.h>
#include <Wire.h>

typedef struct i2c_bus i2c_bus_t;

struct i2c_bus
{
    void (*read_bytes)(i2c_bus_t *self, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
    void (*write_bytes)(i2c_bus_t *self, uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len);
    TwoWire *(*get_TwoWire_handle)(i2c_bus_t *self);

    // 上下文
    void * const ctx;
};

extern i2c_bus_t i2c1;
extern i2c_bus_t i2c2;

void i2c_bus_init(i2c_bus_t *self);

#endif
