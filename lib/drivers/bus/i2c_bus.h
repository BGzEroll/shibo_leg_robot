#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <Arduino.h>
#include <Wire.h>

enum class i2c_result : uint8_t
{
    OK = 0,
    INVALID_BUS,
    INVALID_ARG,
    NOT_INITIALIZED,
    TX_BUFFER_FULL,
    ADDR_NACK,
    DATA_NACK,
    OTHER_ERROR,
    TIMEOUT,
    SHORT_READ,
    SHORT_WRITE
};

class i2c_bus
{
    public:
        explicit i2c_bus(uint8_t bus_id = 0);

    public:
        i2c_result init();
        i2c_result read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
        i2c_result write_bytes(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len);
        TwoWire *get_TwoWire_handle() const;

    private:
        uint8_t bus_id;
};

#endif
