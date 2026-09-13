#ifndef MPU6050_H
#define MPU6050_H

#include "bus/i2c_bus.h"

class mpu6050
{
    public:
        mpu6050(i2c_bus &i2c, uint8_t addr, float acc_coef);

    public:
        bool init(bool cail = false);
        bool update();

    public:
        float temperature;
        float acc[3];
        float gyro[3];
        float angle[3];

    private:
        i2c_bus &i2c;
        uint8_t addr;
        float acc_coef;

    private:
        void process_data();
        bool write_cfg(uint8_t reg, uint8_t val);
        bool get_gyro_offset();

    private:
        uint8_t raw[14];
        float gyro_offset[3]{};
        uint32_t prev_Ts;
        float gyro_angle[3];
};

#endif
