#ifndef UART_BUS_H
#define UART_BUS_H

#include <Arduino.h>

enum class uart_result : uint8_t
{
    OK = 0,
    INVALID_BUS,
    INVALID_ARG,
    NOT_INITIALIZED,
    SHORT_WRITE,
    OTHER_ERROR
};

class uart_bus
{
    public:
        explicit uart_bus(uint8_t bus_id = 0);

    public:
        uart_result init();
        uart_result read_bytes(uint8_t *buf, uint32_t max_len, uint32_t &out_len);
        uart_result write_bytes(const uint8_t *buf, uint32_t len);
        HardwareSerial *get_HardwareSerial_handle() const;

    private:
        uint8_t bus_id;
};

#endif
