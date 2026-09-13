#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c.h"

// 每条总线只创建一个实例；运行期由 sensor_task 独占。
class i2c_bus
{
    public:
        i2c_bus(i2c_port_t port, gpio_num_t scl, gpio_num_t sda);

    public:
        bool init();
        bool read_bytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t size);
        bool write_byte(uint8_t addr, uint8_t reg, uint8_t value);

    private:
        i2c_port_t port;
        gpio_num_t scl;
        gpio_num_t sda;
        bool ready = false;
};

#endif
