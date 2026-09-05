#include "lowpass_filter.h"

#include "time_utils.h"

/**
 * @brief 初始化滤波时间常数和输出状态
 *
 * @param time_constant 时间常数，单位秒
 */
LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant),
      y_prev(0.0f)
{
    timestamp_prev = _micros();
}

/**
 * @brief 更新滤波输出；间隔超过 0.3 秒时直接采用输入
 *
 * @param input 本次输入值
 *
 * @return 滤波输出
 */
float LowPassFilter::operator()(float input)
{
    uint32_t timestamp = _micros();
    float dt = (timestamp - timestamp_prev) * 1e-6f;
    if(dt < 0.0f)
    {
        dt = 1e-3f;
    }
    else if(dt > 0.3f)
    {
        y_prev = input;
        timestamp_prev = timestamp;
        return input;
    }
    float alpha = Tf / (Tf + dt);
    float output = alpha * y_prev + (1.0f - alpha) * input;
    y_prev = output;
    timestamp_prev = timestamp;
    return output;
}
