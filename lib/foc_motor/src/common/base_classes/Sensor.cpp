#include "Sensor.h"

#include "../foc_utils.h"
#include "../time_utils.h"

/**
 * @brief 获取最近采样的单圈机械角
 *
 * @return 单圈角度，单位 rad
 */
float Sensor::getMechanicalAngle()
{
    return angle_prev;
}

/**
 * @brief 获取包含整圈累计的机械角
 *
 * @return 多圈角度，单位 rad；长时间累计受 float 精度限制
 */
float Sensor::getAngle()
{
    return (float)full_rotations * _2PI + angle_prev;
}

/**
 * @brief 以采样时间差计算轴速度，过短间隔沿用上次结果
 *
 * @return 轴速度，单位 rad/s
 */
float Sensor::getVelocity()
{
    float dt = (angle_prev_ts - vel_angle_prev_ts) * 1e-6;
    if(dt < min_elapsed_time){return velocity;}
    velocity = ((float)(full_rotations - vel_full_rotations) * _2PI +
        (angle_prev - vel_angle_prev)) / dt;
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return velocity;
}

/** @brief 读取一次单圈角并按原跨圈阈值更新累计圈数 */
void Sensor::update()
{
    float angle = getSensorAngle();
    angle_prev_ts = _micros();
    float delta_angle = angle - angle_prev;
    if(abs(delta_angle) > 0.8f * _2PI)
    {
        full_rotations += delta_angle > 0 ? -1 : 1;
    }
    angle_prev = angle;
}

/** @brief 按原采样次序初始化角度及速度差分基准，避免启动跳变 */
void Sensor::init()
{
    getSensorAngle();
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle();
    vel_angle_prev_ts = _micros();
    delay(1);
    getSensorAngle();
    delayMicroseconds(1);
    angle_prev = getSensorAngle();
    angle_prev_ts = _micros();
}
