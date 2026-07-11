#include "mpu6050_dev.h"

#include "bus/i2c_bus.h"
#include "latest_channel.h"

static i2c_bus imu_i2c(1);
mpu6050 mpu6050_dev::imu(imu_i2c, 0x68, 0.02f);
static foundation::latest_channel<foundation::timed_snapshot<mpu6050_dev::data>> imu_channel;
static uint32_t imu_sequence = 0;

/**
 * @brief 获取最新 MPU6050 数据快照
 *
 * @param out 数据快照输出
 *
 * @return 存在已发布快照时返回 true
 */
bool mpu6050_dev::latest(foundation::timed_snapshot<mpu6050_dev::data> &out)
{
    if(!imu_channel.latest(out)){return false;}
    return out.valid;
}

/**
 * @brief 发布 MPU6050 数据快照
 *
 * @param value MPU6050 数据
 * @param timestamp_us 采样时间戳，单位微秒
 *
 * @return 发布成功时返回 true
 */
bool mpu6050_dev::publish(const mpu6050_dev::data &value, uint32_t timestamp_us)
{
    foundation::timed_snapshot<mpu6050_dev::data> snapshot;
    snapshot.set(value, timestamp_us, ++imu_sequence);
    return imu_channel.publish(snapshot);
}

/**
 * @brief 初始化 MPU6050 设备模块
 */
void mpu6050_dev::init()
{
    imu.init(true);
    imu_sequence = 0;
    imu_channel.init();
}
