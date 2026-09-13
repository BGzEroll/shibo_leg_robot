#include "pwm_servo.h"

/**
 * @brief 构造 PWM 舵机驱动对象
 *
 * @param pin 引脚号
 * @param channel PWM 通道
 * @param freq PWM 频率
 * @param resolution PWM 分辨率
 * @param min_us 最小脉宽
 * @param max_us 最大脉宽
 */
pwm_servo::pwm_servo(uint8_t pin, uint8_t channel, uint32_t freq, uint8_t resolution, uint16_t min_us, uint16_t max_us)
    : pin(pin), channel(channel), freq(freq), resolution(resolution), min_us(min_us), max_us(max_us)
{
    if(!this->min_us){this->min_us = 500;}
    if(!this->max_us){this->max_us = 2500;}
    if(!this->freq){this->freq = 50;}
    if(!this->resolution){this->resolution = 16;}

    ledcSetup(this->channel, this->freq, this->resolution);
    ledcAttachPin(this->pin, this->channel);
}

/**
 * @brief 按角度设置舵机输出
 *
 * @param angle 角度值
 */
void pwm_servo::set_angle(uint16_t angle)
{
    if(angle > 180){angle = 180;}

    uint32_t span = max_us - min_us;
    uint32_t us = min_us + (uint32_t)angle * span / 180;

    set_us(us);
}

/**
 * @brief 按脉宽设置舵机输出
 *
 * @param us 脉宽值
 */
void pwm_servo::set_us(uint16_t us)
{
    uint32_t duty_max = ((uint32_t)1 << resolution) - 1;
    uint32_t duty = (uint32_t)us * duty_max / (1000000 / freq);

    ledcWrite(channel, duty);
}
