#ifndef MPU6050_DEV_H
#define MPU6050_DEV_H

#include "mpu6050.h"
#include "ports/latest_value.h"

namespace mpu6050_dev
{
    struct data
    {
        uint32_t timestamp_us;
        float temperature;
        float acc[3];
        float gyro[3];
        float angle[3];
    };

    struct output_ports
    {
        port::latest_writer<data> measurement;
    };

    extern mpu6050 imu;

    void publish(const data &measurement);
    void init(const output_ports &outputs);
}

#endif
