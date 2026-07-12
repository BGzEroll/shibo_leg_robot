#ifndef ACTUATOR_PORTS_H
#define ACTUATOR_PORTS_H

#include <Arduino.h>

namespace actuator_port
{
    struct leg_status
    {
        uint32_t timestamp_us = 0;
        int16_t left_position = 0;
        int16_t right_position = 0;
    };

    struct services
    {
        void (*set_leg_pose)(int16_t left, int16_t right, uint16_t speed, uint8_t accel) = nullptr;
        void (*set_leg_torque)(uint8_t type) = nullptr;
        void (*calibrate_leg_middle)() = nullptr;
        void (*set_camera_angle)(uint16_t angle) = nullptr;
        void (*set_frontier_angle)(uint16_t angle) = nullptr;
    };
}

#endif
