#pragma once
#include "gpio.h"
#include <stddef.h>
enum i2c_port_t {I2C_NUM_0, I2C_NUM_1};
constexpr int I2C_MODE_MASTER = 1;
struct i2c_config_t
{
    int mode, sda_io_num, scl_io_num, sda_pullup_en, scl_pullup_en;
    struct {uint32_t clk_speed;} master;
};
inline int i2c_param_config(i2c_port_t, const i2c_config_t *){return 0;}
inline int i2c_driver_install(i2c_port_t, int, int, int, int){return 0;}
int i2c_master_write_read_device(i2c_port_t, uint8_t, const uint8_t *, size_t,
    uint8_t *, size_t, unsigned);
inline int i2c_master_write_to_device(i2c_port_t, uint8_t, const uint8_t *, size_t, unsigned)
{
    return 0;
}
