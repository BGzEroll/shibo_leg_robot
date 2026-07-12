#ifndef BALANCE_CORE_H
#define BALANCE_CORE_H

#include <Arduino.h>
#include "ports/latest_value.h"
#include "ports/actuator_ports.h"
#include "motor.h"
#include "mpu6050_dev.h"

namespace balance_core
{
    struct motion_control
    {
        bool enable_motor = false;
        bool enable_balance = false;
        bool enable_steering = false;
        bool reset_reference = false;
        bool reset_yaw_integral = false;
        float linear_vel = 0.0f;
        float yaw_rate = 0.0f;
    };

    struct direct_output_control
    {
        bool enable = false;
        float left = 0.0f;
        float right = 0.0f;
    };

    struct recover_control
    {
        bool enable = false;
        float output_blend = 1.0f;
    };

    struct feedback_override
    {
        bool enable_linear_feedback = true;
        bool enable_yaw_feedback = true;
        bool enable_yaw_integral = true;
    };

    struct motion_status
    {
        uint32_t timestamp_us = 0;
        float pitch_angle = 0.0f;
        float pitch_rate = 0.0f;
        float avg_linear_vel = 0.0f;
        float yaw_angle = 0.0f;
        float yaw_rate = 0.0f;
        float roll_angle = 0.0f;
        float avg_leg_height = 0.0f;
    };

    struct debug_snapshot
    {
        uint32_t timestamp_us = 0;
        float pitch_angle = 0.0f;
        float pitch_rate = 0.0f;
        float avg_linear_pos = 0.0f;
        float avg_linear_vel = 0.0f;
        float yaw_angle = 0.0f;
        float yaw_rate = 0.0f;
        float reference_linear_vel = 0.0f;
        float reference_yaw_rate = 0.0f;
        float input[2]{};
        float feedback_vector[6]{};
        float output[2]{};
        float roll_angle = 0.0f;
        float leg_height[2]{};
    };

    struct info
    {
        float max_linear_vel = 0.0f;
        float max_steer_vel = 0.0f;
    };

    struct command
    {
        motion_control motion;
        direct_output_control direct_output;
        recover_control recover;
        feedback_override feedback;
    };

    struct input_ports
    {
        port::latest_reader<command> control;
        port::latest_reader<motor::encoder_data> encoder;
        port::latest_reader<mpu6050_dev::data> imu;
        port::latest_reader<actuator_port::leg_status> leg_status;
    };

    struct output_ports
    {
        port::latest_writer<motor::target_data> motor_target;
        port::latest_writer<motion_status> motion;
        port::latest_writer<debug_snapshot> debug;
    };

    info get_info();
    void step(uint32_t tick_ms);

    void init(const input_ports &inputs, const output_ports &outputs);
}

#endif
