#ifndef MAGNETIC_SENSOR_I2C_H
#define MAGNETIC_SENSOR_I2C_H

#include "../common/base_classes/Sensor.h"
#include <Wire.h>

// 保留应用使用的上游配置类型和 AS5600 配置入口。
struct MagneticSensorI2CConfig_s
{
    int32_t chip_address;
    int32_t bit_resolution;
    int32_t angle_register;
    int32_t data_start_bit;
};

extern MagneticSensorI2CConfig_s AS5600_I2C;

/** @brief 通过 I2C 读取 AS5600 角度 */
class MagneticSensorI2C : public Sensor
{
    public:
        MagneticSensorI2C(MagneticSensorI2CConfig_s config);

    public:
        float getSensorAngle() override;
        void init(TwoWire *wire = &Wire);

    private:
        int32_t read_angle();

    private:
        float cpr;
        uint16_t lsb_used;
        uint8_t lsb_mask;
        uint8_t msb_mask;
        uint8_t angle_register_msb;
        uint8_t chip_address;
        TwoWire *wire;
};

#endif
