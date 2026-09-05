#include "time_utils.h"

#include <Arduino.h>

/**
 * @brief 使用 Arduino 时基执行阻塞等待
 *
 * @param ms 等待时间，单位毫秒
 */
void _delay(unsigned long ms)
{
    delay(ms);
}

/**
 * @brief 获取 ESP32 Arduino 微秒时基
 *
 * @return 微秒时间戳，按平台 32 位无符号计数回绕
 */
unsigned long _micros()
{
    return micros();
}
