#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "SimpleFOC.h"

namespace motor
{
    struct encoder_data
    {
        uint32_t timestamp_us;
        float left_shaft_angle;
        float right_shaft_angle;
        float left_shaft_velocity;
        float right_shaft_velocity;
    };

    struct target_data
    {
        uint32_t timestamp_us;
        float left_torque;
        float right_torque;
    };

    extern BLDCMotor left;
    extern BLDCMotor right;

    bool publish_encoder(const encoder_data &value);
    bool peek_encoder(encoder_data &out);
    bool publish_target(const target_data &value);
    bool peek_target(target_data &out);
    void init();
}

#endif
