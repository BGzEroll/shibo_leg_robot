#ifndef UART_BUS_H
#define UART_BUS_H

#include <Arduino.h>

typedef struct uart_bus uart_bus_t;

struct uart_bus
{
    uint32_t (*read_bytes)(uart_bus_t *self, uint8_t *buf, uint32_t max_len);
    void (*write_bytes)(uart_bus_t *self, const uint8_t *buf, uint32_t len);
    HardwareSerial *(*get_HardwareSerial_handle)(uart_bus_t *self);

    // 上下文
    void * const ctx;
};

extern uart_bus_t uart0;
extern uart_bus_t uart2;

void uart_bus_init(uart_bus_t *self);

#endif
