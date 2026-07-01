#include "led.h"

/**
 * @brief 构造 LED 驱动对象
 *
 * @param pin 引脚号
 */
led::led(uint8_t pin)
    : pin(pin)
{
}

/**
 * @brief 初始化 LED 引脚
 */
void led::init()
{
    pinMode(pin, OUTPUT);
}

/**
 * @brief 点亮 LED
 */
void led::on()
{
    digitalWrite(pin, HIGH);
}

/**
 * @brief 熄灭 LED
 */
void led::off()
{
    digitalWrite(pin, LOW);
}

/**
 * @brief 切换 LED 输出状态
 */
void led::toggle()
{
    digitalWrite(pin, !digitalRead(pin));
}
