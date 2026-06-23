#include "mpu6050_dev.h"

namespace mpu6050_dev {

mpu6050 imu(&i2c2, 0x68, 0.02f);
static QueueHandle_t mpu6050_data_queue = nullptr;

QueueHandle_t queue()
{
    return mpu6050_data_queue;
}

void init()
{
    imu.init(1);

    mpu6050_data_queue = xQueueCreate(1, sizeof(data));
}

}
