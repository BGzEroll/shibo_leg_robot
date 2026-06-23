#include "mpu6050_dev.h"

mpu6050 mpu6050_dev::imu(&i2c2, 0x68, 0.02f);
static QueueHandle_t mpu6050_data_queue = nullptr;

/**
 * @brief 获取 MPU6050 数据队列
 *
 * @return 队列句柄
 */
QueueHandle_t mpu6050_dev::queue()
{
    return mpu6050_data_queue;
}

/**
 * @brief 初始化 MPU6050 设备模块
 */
void mpu6050_dev::init()
{
    imu.init(1);

    mpu6050_data_queue = xQueueCreate(1, sizeof(mpu6050_dev::data));
}
