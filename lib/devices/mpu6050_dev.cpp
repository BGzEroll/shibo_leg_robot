#include "mpu6050_dev.h"

#include "bus/i2c_bus.h"

static i2c_bus imu_i2c(1);
mpu6050 mpu6050_dev::imu(imu_i2c, 0x68, 0.02f);
static mpu6050_dev::output_ports module_outputs;

/**
 * @brief 发布最新 MPU6050 测量
 *
 * @param measurement MPU6050 测量
 */
void mpu6050_dev::publish(const mpu6050_dev::data &measurement)
{
    module_outputs.measurement.publish(measurement);
}

/**
 * @brief 初始化 MPU6050 设备模块
 *
 * @param outputs MPU6050 输出端口
 */
void mpu6050_dev::init(const mpu6050_dev::output_ports &outputs)
{
    module_outputs = outputs;
    imu.init(true);
}
