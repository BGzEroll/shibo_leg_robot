#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "SimpleFOC.h"
#include "ports/latest_value.h"

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
        bool enabled = false;
    };

    struct input_ports
    {
        port::latest_reader<target_data> target;
    };

    struct output_ports
    {
        port::latest_writer<encoder_data> encoder;
    };

    extern BLDCMotor left;
    extern BLDCMotor right;

    bool read_target(target_data &out);
    void publish_encoder(const encoder_data &data);
    void init(const input_ports &inputs, const output_ports &outputs);
}

#endif
