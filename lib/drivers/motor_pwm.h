#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "foc_motor.h"
#include "driver/mcpwm.h"

class motor_pwm
{
    public:
        motor_pwm(mcpwm_operator_t output, gpio_num_t a, gpio_num_t b,
            gpio_num_t c, gpio_num_t enable);

    public:
        static bool init_timers();
        bool init();
        void set_enabled(bool enabled);
        void write(const foc_motor::duty &duty);

    private:
        mcpwm_operator_t output;
        gpio_num_t pins[3];
        gpio_num_t enable;
        bool enabled = false;
};

#endif
