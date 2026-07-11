#ifndef MOTION_PORT_H
#define MOTION_PORT_H

#include <Arduino.h>

namespace controller
{
    enum class balance_drive_mode : uint8_t
    {
        STOP = 0,
        BALANCE,
        DIRECT_OUTPUT,
        RECOVER
    };

    struct balance_request
    {
        balance_drive_mode mode = balance_drive_mode::STOP;
        bool enable_steering = false;
        bool reset_reference = false;
        bool reset_yaw_integral = false;
        float linear_vel = 0.0f;
        float yaw_rate = 0.0f;
        float direct_left = 0.0f;
        float direct_right = 0.0f;
        float recover_blend = 1.0f;
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

    struct motion_info
    {
        float max_linear_vel = 0.0f;
        float max_steer_vel = 0.0f;
    };

    class motion_port
    {
        public:
            virtual ~motion_port() = default;

        public:
            virtual void init() = 0;
            virtual bool latest_status(motion_status &out) = 0;
            virtual motion_info info() const = 0;
            virtual void apply(const balance_request &request) = 0;
    };
}

#endif
