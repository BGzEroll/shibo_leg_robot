#include "mpu6050_dev.h"

#include "bus/i2c_bus.h"

static i2c_bus imu_i2c(1);
mpu6050 mpu6050_dev::imu(imu_i2c, 0x68, 0.02f);
static QueueHandle_t mpu6050_data_queue = nullptr;

/**
 * @brief 发布最新 MPU6050 数据
 *
 * @param value MPU6050 数据
 *
 * @return 队列存在且发布成功时返回 true
 */
bool mpu6050_dev::publish_data(const mpu6050_dev::data &value)
{
    return mpu6050_data_queue &&
           xQueueOverwrite(mpu6050_data_queue, &value) == pdTRUE;
}

/**
 * @brief 读取最新 MPU6050 数据
 *
 * @param out MPU6050 数据输出
 *
 * @return 队列存在且已有数据时返回 true
 */
bool mpu6050_dev::peek_data(mpu6050_dev::data &out)
{
    return mpu6050_data_queue &&
           xQueuePeek(mpu6050_data_queue, &out, 0) == pdTRUE;
}

/**
 * @brief 初始化 MPU6050 设备模块
 */
void mpu6050_dev::init()
{
    imu.init(true);

    mpu6050_data_queue = xQueueCreate(1, sizeof(mpu6050_dev::data));
}
