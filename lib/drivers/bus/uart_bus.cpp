#include "uart_bus.h"

extern HardwareSerial Serial;
extern HardwareSerial Serial2;

typedef struct uart_bus_ctx {
    HardwareSerial *uart_handle;
    uint32_t baudrate;
    uint8_t is_init;
} uart_bus_ctx_t;

// uart1 实例
static uart_bus_ctx_t uart0Ctx = {.uart_handle = &Serial, .baudrate = 230400};
uart_bus_t uart0 = {.ctx = &uart0Ctx};

// uart2 实例
static uart_bus_ctx_t uart2Ctx = {.uart_handle = &Serial2, .baudrate = 1000000};
uart_bus_t uart2 = {.ctx = &uart2Ctx};

/**
 * @brief uart 连续读字节
 * 
 * @return 字节数
 */
static uint32_t read_bytes(uart_bus_t *self, uint8_t *buf, uint32_t max_len)
{
    uart_bus_ctx_t *p = (uart_bus_ctx_t *)self->ctx;

    uint32_t len = 0;
    while(p->uart_handle->available() && len < max_len)
    {
        buf[len++] = p->uart_handle->read();
    }

    return len;
}

/**
 * @brief uart 连续写字节
 */
static void write_bytes(uart_bus_t *self, const uint8_t *buf, uint32_t len)
{
    uart_bus_ctx_t *p = (uart_bus_ctx_t *)self->ctx;

    p->uart_handle->write(buf, len);
    p->uart_handle->flush();
}

/**
 * @brief 获取 uart 句柄供 Arduino 其余库使用
 */
static HardwareSerial *get_HardwareSerial_handle(uart_bus_t *self)
{
    uart_bus_ctx_t *p = (uart_bus_ctx_t *)self->ctx;

    return p->uart_handle;
}

/**
 * @brief 初始化 uart 驱动
 */
void uart_bus_init(uart_bus_t *self)
{
    if(!self){return;}

    uart_bus_ctx_t *p = (uart_bus_ctx_t *)self->ctx;
    if(p->is_init){return;}
    p->is_init = 1;

    p->uart_handle->begin(p->baudrate);

    self->read_bytes = read_bytes;
    self->write_bytes = write_bytes;
    self->get_HardwareSerial_handle = get_HardwareSerial_handle;
}
