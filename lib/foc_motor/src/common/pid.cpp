#include "pid.h"

#include "foc_utils.h"
#include "time_utils.h"

/**
 * @brief 初始化 PID 参数及历史状态
 *
 * @param p 比例增益
 * @param i 积分增益
 * @param d 微分增益
 * @param ramp 输出每秒最大变化量，非正值表示不限制
 * @param output_limit 输出绝对值上限
 */
PIDController::PIDController(float p, float i, float d, float ramp, float output_limit)
    : P(p),
      I(i),
      D(d),
      output_ramp(ramp),
      limit(output_limit),
      error_prev(0.0f),
      output_prev(0.0f),
      integral_prev(0.0f)
{
    timestamp_prev = _micros();
}

/**
 * @brief 按实测时间差计算 PID 输出并更新历史状态
 *
 * @param error 本次控制误差
 *
 * @return 限幅和斜率约束后的控制输出
 */
float PIDController::operator()(float error)
{
    uint32_t timestamp_now = _micros();
    float dt = (timestamp_now - timestamp_prev) * 1e-6f;
    if(dt <= 0 || dt > 0.5f){dt = 1e-3f;}

    float proportional = P * error;
    // 梯形积分及积分限幅，保留原计算顺序。
    float integral = integral_prev + I * dt * 0.5f * (error + error_prev);
    integral = _constrain(integral, -limit, limit);
    float derivative = D * (error - error_prev) / dt;
    float output = proportional + integral + derivative;
    output = _constrain(output, -limit, limit);

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
