#ifndef HW_MOTOR_H
#define HW_MOTOR_H

#include <stdint.h>

namespace hw
{
    namespace motor
    {
        struct command
        {
            uint32_t timestamp_us = 0;
            float left = 0.0f;  // N·m，直接采用平衡算法原输出。
            float right = 0.0f;
            bool enabled = false;
        };

        struct encoder_state
        {
            uint32_t timestamp_us = 0; // 左右样本中较早的时间。
            float left_shaft_angle = 0.0f;
            float right_shaft_angle = 0.0f;
            float left_shaft_velocity = 0.0f;
            float right_shaft_velocity = 0.0f;
            bool valid = false;
        };

        bool latest_encoder(encoder_state &out);
        bool publish_command(const command &value);
        void sample_encoders();
        bool init();
        void foc_loop(void *arg);
    }
}

#endif
