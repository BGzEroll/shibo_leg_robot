#ifndef MPU6050_DEV_H
#define MPU6050_DEV_H

#include "mpu6050.h"

namespace mpu6050_dev {

struct data
{
    uint32_t timestamp_us;
    float temperature;
    float acc[3];
    float gyro[3];
    float angle[3];
};

extern mpu6050 imu;

QueueHandle_t queue();
void init();

}

#endif
