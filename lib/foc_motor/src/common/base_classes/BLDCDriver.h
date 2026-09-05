#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

#include <Arduino.h>

/** @brief 三相 PWM 驱动接口 */
class BLDCDriver
{
    public:
        virtual void enable() = 0;
        virtual void disable() = 0;
        virtual void setPwm(float ua, float ub, float uc) = 0;
        virtual int init() = 0;

    public:
        int32_t pwm_frequency = 0;
        float voltage_power_supply = 0.0f;
        float voltage_limit = 0.0f;
        float dc_a = 0.0f;
        float dc_b = 0.0f;
        float dc_c = 0.0f;
        bool initialized = false;
        void *params = nullptr;
};

#endif
