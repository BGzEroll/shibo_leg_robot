#include "i2c_bus.h"

/** @brief 保存固定总线编号和引脚 */
i2c_bus::i2c_bus(i2c_port_t port, gpio_num_t scl, gpio_num_t sda)
    : port(port), scl(scl), sda(sda)
{
}

/** @brief 初始化 400 kHz IDF 主机驱动；失败时允许下次重试 */
bool i2c_bus::init()
{
    if(ready){return true;}
    i2c_config_t config{};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = sda;
    config.scl_io_num = scl;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = 400000;
    ready = i2c_param_config(port, &config) == ESP_OK &&
        i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0) == ESP_OK;
    return ready;
}

/** @brief 使用重复起始读取连续寄存器；最多等待两个 RTOS tick */
bool i2c_bus::read_bytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t size)
{
    return ready && data && size &&
        i2c_master_write_read_device(port, addr, &reg, 1, data, size, 2) == ESP_OK;
}

/** @brief 写入一个配置寄存器 */
bool i2c_bus::write_byte(uint8_t addr, uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return ready && i2c_master_write_to_device(port, addr, data, 2, 2) == ESP_OK;
}
