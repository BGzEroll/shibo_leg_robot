#ifndef HW_MOTOR_H
#define HW_MOTOR_H

#include <Arduino.h>
#include "SimpleFOC.h"
#include "control/balance.h"

namespace hw
{
    namespace motor
    {
        struct encoder_state
        {
            uint32_t timestamp_us = 0;
            float left_shaft_angle = 0.0f;
            float right_shaft_angle = 0.0f;
            float left_shaft_velocity = 0.0f;
            float right_shaft_velocity = 0.0f;
        };

        bool latest_encoder(encoder_state &out);
        bool latest_command(control::motor_command &out);
        bool publish_encoder(const encoder_state &value);
        bool publish_command(const control::motor_command &value);
        bool sample_encoders();
        bool apply_latest_encoder_sample(uint32_t &timestamp_us);

        extern BLDCMotor left;
        extern BLDCMotor right;

        void init();
    }
}

#endif
