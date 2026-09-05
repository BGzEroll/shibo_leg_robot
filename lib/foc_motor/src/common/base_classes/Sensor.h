#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

enum Direction : int8_t
{
    CW = 1,
    CCW = -1,
    UNKNOWN = 0
};

/**
 * @brief 维护单圈角、多圈累计和采样速度
 *
 * 运行期由 sensor task 调用 update()，FOC 使用快照适配器读取已有样本。
 */
class Sensor
{
    public:
        virtual float getMechanicalAngle();
        virtual float getAngle();
        virtual float getVelocity();
        virtual void update();

    public:
        float min_elapsed_time = 0.000100;

    protected:
        virtual float getSensorAngle() = 0;
        virtual void init();

    protected:
        float velocity = 0.0f;
        float angle_prev = 0.0f;
        int32_t angle_prev_ts = 0;
        float vel_angle_prev = 0.0f;
        int32_t vel_angle_prev_ts = 0;
        int32_t full_rotations = 0;
        int32_t vel_full_rotations = 0;
};

#endif
