#include "i2c_bus.h"

static TwoWire wire_1 = TwoWire(0);
static TwoWire wire_2 = TwoWire(1);

typedef struct i2c_bus_ctx
{
    TwoWire *i2c_handle;
    uint8_t scl_pin;
    uint8_t sda_pin;
    uint32_t frequency;
    uint8_t is_init;
} i2c_bus_ctx_t;

// i2c1 实例
static i2c_bus_ctx_t i2c1Ctx = {.i2c_handle = &wire_1, .scl_pin = 18, .sda_pin = 19, .frequency = 400000};
i2c_bus_t i2c1 = {.ctx = &i2c1Ctx};

// i2c2 实例
static i2c_bus_ctx_t i2c2Ctx = {.i2c_handle = &wire_2, .scl_pin = 5, .sda_pin = 23, .frequency = 400000};
i2c_bus_t i2c2 = {.ctx = &i2c2Ctx};

/**
 * @brief 从 I2C 设备连续读取寄存器数据
 *
 * @param self 总线实例指针
 * @param addr 设备地址
 * @param reg 寄存器地址
 * @param buf 数据缓冲区
 * @param len 数据长度
 */
static void read_bytes(i2c_bus_t *self, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_bus_ctx_t *p = (i2c_bus_ctx_t *)self->ctx;

    p->i2c_handle->beginTransmission(addr);
    p->i2c_handle->write(reg);
    p->i2c_handle->endTransmission(false);      // 重复起始

    p->i2c_handle->requestFrom(addr, len);
    p->i2c_handle->readBytes(buf, len);
}

/**
 * @brief 向 I2C 设备连续写入寄存器数据
 *
 * @param self 总线实例指针
 * @param addr 设备地址
 * @param reg 寄存器地址
 * @param buf 数据缓冲区
 * @param len 数据长度
 */
static void write_bytes(i2c_bus_t *self, uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len)
{
    i2c_bus_ctx_t *p = (i2c_bus_ctx_t *)self->ctx;

    p->i2c_handle->beginTransmission(addr);
    p->i2c_handle->write(reg);
    p->i2c_handle->write(buf, len);
    p->i2c_handle->endTransmission(true);
}

/**
 * @brief 获取底层 TwoWire 句柄
 *
 * @param self 总线实例指针
 *
 * @return 返回值
 */
static TwoWire *get_TwoWire_handle(i2c_bus_t *self)
{
    i2c_bus_ctx_t *p = (i2c_bus_ctx_t *)self->ctx;

    return p->i2c_handle;
}

/**
 * @brief 初始化 I2C 总线实例
 *
 * @param self 总线实例指针
 */
void i2c_bus_init(i2c_bus_t *self)
{
    if(!self){return;}

    i2c_bus_ctx_t *p = (i2c_bus_ctx_t *)self->ctx;
    if(p->is_init){return;}
    p->is_init = 1;

    p->i2c_handle->begin(p->sda_pin, p->scl_pin, p->frequency);

    self->read_bytes = read_bytes;
    self->write_bytes = write_bytes;
    self->get_TwoWire_handle = get_TwoWire_handle;
}
