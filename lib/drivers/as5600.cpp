#include "as5600.h"

#include "esp_timer.h"

/** @brief 关联实例使用的 I2C 总线 */
as5600::as5600(i2c_bus &bus) : bus(bus)
{
}

/** @brief 读取固定 RAW_ANGLE 寄存器，仅成功事务更新时间戳 */
as5600::sample as5600::read()
{
    sample value;
    uint8_t data[2];
    if(!bus.read_bytes(0x36, 0x0C, data, 2)){return value;}
    value.timestamp_us = (uint32_t)esp_timer_get_time();
    value.raw = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
    value.valid = true;
    return value;
}
