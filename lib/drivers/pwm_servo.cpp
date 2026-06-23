#include "pwm_servo.h"

/**
 * @brief pwm 舵机构造函数
 * 
 * @param pin 舵机引脚
 * @param channel 舵机通道
 * @param freq 舵机频率
 * @param resolution 舵机分辨率
 * @param min_us 舵机最小脉冲宽度
 * @param max_us 舵机最大脉冲宽度
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
 * @brief 设置舵机角度
 * 
 * @param angle 舵机角度
 */
void pwm_servo::set_angle(uint16_t angle)
{
    if(angle > 180){angle = 180;}

    uint32_t span = max_us - min_us;
    uint32_t us = min_us + (uint32_t)angle * span / 180;

    set_us(us);
}

/**
 * @brief 设置舵机脉冲宽度
 * 
 * @param us 舵机脉冲宽度
 */
void pwm_servo::set_us(uint16_t us)
{
    uint32_t duty_max = ((uint32_t)1 << resolution) - 1;
    uint32_t duty = (uint32_t)us * duty_max / (1000000 / freq);

    ledcWrite(channel, duty);
}
