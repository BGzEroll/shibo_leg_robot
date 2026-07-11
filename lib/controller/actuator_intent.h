#ifndef ACTUATOR_INTENT_H
#define ACTUATOR_INTENT_H

#include <Arduino.h>

namespace controller
{
    struct leg_pose_intent
    {
        bool valid = false;
        int16_t left = 0;
        int16_t right = 0;
        uint16_t speed = 0;
        uint8_t accel = 0;
    };

    struct leg_torque_intent
    {
        bool valid = false;
        uint8_t type = 0;
    };

    struct accessory_intent
    {
        bool camera_valid = false;
        uint16_t camera_angle = 0;
        bool frontier_valid = false;
        uint16_t frontier_angle = 0;
    };

    struct actuator_intent
    {
        leg_pose_intent leg_pose;
        leg_torque_intent leg_torque;
        accessory_intent accessory;
        bool calibrate_middle = false;
    };

    struct actuator_feedback
    {
        int16_t left_leg_position = 0;
        int16_t right_leg_position = 0;
    };
}

#endif
