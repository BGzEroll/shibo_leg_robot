#ifndef FOC_UTILS_H
#define FOC_UTILS_H

#include <Arduino.h>

// 保留实际使用的上游数值常量和宏，避免改变浮点计算及调用边界。
#ifndef _round
#define _round(value) ((value) >= 0 ? (int32_t)((value) + 0.5f) : (int32_t)((value) - 0.5f))
#endif
#define _constrain(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define _isset(value) ((value) != NOT_SET)

#define _SQRT3 1.73205080757f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _RPM_TO_RADS 0.10471975512f
#define NOT_SET -12345.0

struct DQVoltage_s
{
    float d;
    float q;
};

float _sin(float angle);
float _normalizeAngle(float angle);

#endif
