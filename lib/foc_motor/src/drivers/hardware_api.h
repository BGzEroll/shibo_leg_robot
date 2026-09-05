#ifndef HARDWARE_API_H
#define HARDWARE_API_H

#include "../common/foc_utils.h"
#include "../common/time_utils.h"

#define SIMPLEFOC_DRIVER_INIT_FAILED ((void *)-1)

// 保留上游硬件边界的函数名及参数类型。
void *_configure3PWM(long pwm_frequency, int pin_a, int pin_b, int pin_c);
void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c, void *params);

#endif
