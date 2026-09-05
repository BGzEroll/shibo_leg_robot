#ifndef BLDC_DRIVER_3PWM_H
#define BLDC_DRIVER_3PWM_H

#include "../common/base_classes/BLDCDriver.h"
#include "../common/foc_utils.h"

/** @brief 三路 PWM 和一路公共使能的无刷电机驱动 */
class BLDCDriver3PWM : public BLDCDriver
{
    public:
        BLDCDriver3PWM(int phase_a, int phase_b, int phase_c, int enable_pin = NOT_SET);

    public:
        void enable() override;
        void disable() override;
        void setPwm(float ua, float ub, float uc) override;
        int init() override;

    public:
        int32_t pwmA;
        int32_t pwmB;
        int32_t pwmC;
        int32_t enableA_pin;
        bool enable_active_high = true;
};

#endif
