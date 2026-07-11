#ifndef MPU6050_DEV_H
#define MPU6050_DEV_H

#include "timed_snapshot.h"
#include "mpu6050.h"

namespace mpu6050_dev
{
    struct data
    {
        float temperature = 0.0f;
        float acc[3]{};
        float gyro[3]{};
        float angle[3]{};
    };

    extern mpu6050 imu;

    bool latest(foundation::timed_snapshot<data> &out);
    bool publish(const data &value, uint32_t timestamp_us);
    void init();
}

#endif
