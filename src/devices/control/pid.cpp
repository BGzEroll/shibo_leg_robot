#include "pid.h"

#include "esp_timer.h"
#include <algorithm>

/**
 * @brief 初始化 PID 参数及历史状态
 *
 * @param p 比例增益
 * @param i 积分增益
 * @param d 微分增益
 * @param ramp 输出每秒最大变化量，非正值表示不限制
 * @param output_limit 输出绝对值上限
 */
pid::pid(float p, float i, float d, float ramp, float output_limit)
    : p(p),
      i(i),
      d(d),
      output_ramp(ramp),
      limit(output_limit),
      error_prev(0.0f),
      output_prev(0.0f),
      integral_prev(0.0f)
{
    timestamp_prev = (uint32_t)esp_timer_get_time();
}

/**
 * @brief 按实测时间差计算 PID 输出并更新历史状态
 *
 * @param error 本次控制误差
 *
 * @return 限幅和斜率约束后的控制输出
 */
float pid::operator()(float error)
{
    uint32_t timestamp_now = (uint32_t)esp_timer_get_time();
    float dt = (timestamp_now - timestamp_prev) * 1e-6f;
    if(dt <= 0 || dt > 0.5f){dt = 1e-3f;}

    float proportional = p * error;
    // 梯形积分及积分限幅，保留原计算顺序。
    float integral = integral_prev + i * dt * 0.5f * (error + error_prev);
    integral = std::min(std::max(integral, -limit), limit);
    float derivative = d * (error - error_prev) / dt;
    float output = proportional + integral + derivative;
    output = std::min(std::max(output, -limit), limit);

    if(output_ramp > 0)
    {
        float output_rate = (output - output_prev) / dt;
        if(output_rate > output_ramp)
        {
            output = output_prev + output_ramp * dt;
        }
        else if(output_rate < -output_ramp)
        {
            output = output_prev - output_ramp * dt;
        }
    }
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}
