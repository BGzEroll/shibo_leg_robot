#include "lowpass_filter.h"

#include "esp_timer.h"

/**
 * @brief 初始化滤波时间常数和输出状态
 *
 * @param time_constant 时间常数，单位秒
 */
lowpass_filter::lowpass_filter(float time_constant)
    : time_constant(time_constant),
      y_prev(0.0f)
{
    timestamp_prev = (uint32_t)esp_timer_get_time();
}

/**
 * @brief 更新滤波输出；间隔超过 0.3 秒时直接采用输入
 *
 * @param input 本次输入值
 *
 * @return 滤波输出
 */
float lowpass_filter::operator()(float input)
{
    uint32_t timestamp = (uint32_t)esp_timer_get_time();
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
    float alpha = time_constant / (time_constant + dt);
    float output = alpha * y_prev + (1.0f - alpha) * input;
    y_prev = output;
    timestamp_prev = timestamp;
    return output;
}
