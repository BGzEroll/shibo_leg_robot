#ifndef BALANCE_CORE_H
#define BALANCE_CORE_H

#include <Arduino.h>

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

    void apply_motion_control(const motion_control &control);
    void apply_direct_output(const direct_output_control &control);
    void apply_recover_control(const recover_control &control);
    void apply_feedback_override(const feedback_override &override_control);
    bool get_motion_status(motion_status &out);
    bool get_debug_snapshot(debug_snapshot &out);
    info get_info();

    void init();
    void core_task_entry(void *arg);
    void control_task_entry(void *arg);
}

#endif
