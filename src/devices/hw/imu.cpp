#include "imu.h"

#include "sensor.h"
#include "esp_timer.h"
#include "mpu6050.h"
#include "util/latest.h"

static mpu6050 imu_driver(hw::sensor::right_bus, 0x68, 0.02f);
static util::latest<hw::imu::state> imu_latest;

/**
 * @brief 读取最近一次 IMU 快照
 *
 * @param out IMU 状态输出
 *
 * @return 已有 IMU 快照时返回 true
 */
bool hw::imu::latest(hw::imu::state &out)
{
    return imu_latest.get(out);
}

/**
 * @brief 初始化 MPU6050 和 IMU 快照
 */
bool hw::imu::init()
{
    return imu_latest.init() && imu_driver.init(true);
}

/**
 * @brief 读取一次 MPU6050 并发布姿态快照
 */
void hw::imu::sample()
{
    if(!imu_driver.update()){return;}

    hw::imu::state value;
    value.timestamp_us = (uint32_t)esp_timer_get_time();
    value.temperature = imu_driver.temperature;
    for(uint8_t i = 0; i < 3; i++)
    {
        value.acc[i] = imu_driver.acc[i];
        value.gyro[i] = imu_driver.gyro[i];
        value.angle[i] = imu_driver.angle[i];
    }
    imu_latest.set(value);
}
