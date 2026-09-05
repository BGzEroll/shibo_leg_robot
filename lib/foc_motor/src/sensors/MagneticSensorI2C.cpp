#include "MagneticSensorI2C.h"

#include "../common/foc_utils.h"

MagneticSensorI2CConfig_s AS5600_I2C =
{
    0x36,
    12,
    0x0C,
    11
};

/**
 * @brief 根据寄存器布局初始化角度解码参数
 *
 * @param config 地址、分辨率及数据位布局
 */
MagneticSensorI2C::MagneticSensorI2C(MagneticSensorI2CConfig_s config)
{
    chip_address = config.chip_address;
    angle_register_msb = config.angle_register;
    cpr = pow(2, config.bit_resolution);
    int32_t bits_used_msb = config.data_start_bit - 7;
    lsb_used = config.bit_resolution - bits_used_msb;
    lsb_mask = (uint8_t)((2 << lsb_used) - 1);
    msb_mask = (uint8_t)((2 << bits_used_msb) - 1);
    wire = &Wire;
}

/**
 * @brief 读取角度寄存器并转换为弧度
 *
 * @return 单圈机械角，单位 rad
 */
float MagneticSensorI2C::getSensorAngle()
{
    return (read_angle() / (float)cpr) * _2PI;
}

/**
 * @brief 关联 I2C 总线并初始化角度差分基准
 *
 * @param bus 已配置的 I2C 总线
 */
void MagneticSensorI2C::init(TwoWire *bus)
{
    wire = bus;
    wire->begin();
    Sensor::init();
}

/**
 * @brief 按高字节在前的顺序读取并拼接角度计数
 *
 * @return 原始角度计数
 */
int32_t MagneticSensorI2C::read_angle()
{
    uint8_t data[2];
    wire->beginTransmission(chip_address);
    wire->write(angle_register_msb);
    wire->endTransmission(false);
    wire->requestFrom(chip_address, (uint8_t)2);
    for(uint8_t i = 0; i < 2; i++)
    {
        data[i] = wire->read();
    }
    uint16_t value = data[1] & lsb_mask;
    value += (data[0] & msb_mask) << lsb_used;
    return value;
}
