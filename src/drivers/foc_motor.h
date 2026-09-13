#ifndef FOC_MOTOR_H
#define FOC_MOTOR_H

#include <stdint.h>

// 固定 7 极对、AS5600、Ud=0；不依赖 Arduino、RTOS 或 PWM 外设。
class foc_motor
{
    public:
        struct duty
        {
            uint16_t phase[3]; // Q15，32768 表示 100%。
        };

        void sample(uint16_t raw, uint32_t timestamp_us);
        void align(int8_t direction, uint16_t mechanical_phase);
        duty update(int32_t torque_uNm, uint32_t now_us) const;
        static duty svpwm(int32_t uq, uint16_t phase);

        int64_t full_count = 0;
        int32_t speed_mrad_s = 0;
        uint32_t timestamp_us = 0;
        int8_t direction = 0;

    private:
        uint16_t last_raw = 0;
        uint16_t zero_phase = 0;
        bool sampled = false;
};

#endif
