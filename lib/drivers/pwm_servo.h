#ifndef PWM_SERVO_H
#define PWM_SERVO_H

#include <Arduino.h>

class pwm_servo {
    public:
        pwm_servo(uint8_t pin, uint8_t channel, uint32_t freq, uint8_t resolution, uint16_t min_us, uint16_t max_us);
    
    public:
        void set_angle(uint16_t angle);

    private:
        uint8_t pin;
        uint8_t channel;
        uint32_t freq;
        uint8_t resolution;
        uint16_t min_us;
        uint16_t max_us;
    
    private:
        void set_us(uint16_t us);
};

#endif
