#ifndef CONTROL_INPUT_H
#define CONTROL_INPUT_H

#include <Arduino.h>

namespace controller
{
    enum class input_source : uint8_t
    {
        NONE = 0,
        XBOX,
        WEB,
        HOST
    };

    enum class action_request : uint8_t
    {
        NONE = 0,
        STOP,
        BOOT,
        BOOT_CONFIRM,
        RESET_BALANCE,
        SIT,
        MIDDLE_CALIBRATION,
        JUMP_IN_PLACE,
        JUMP_FORWARD,
        JUMP_BACKWARD,
        JUMP_LEFT,
        JUMP_RIGHT,
        KICK_PLACE,
        KICK_RUN,
        KICK_EXIT,
        EXIT_ACTION,
        ACTION_DONE
    };

    struct control_input
    {
        input_source source = input_source::NONE;
        uint32_t timestamp_us = 0;
        float linear_cmd = 0.0f;
        float yaw_cmd = 0.0f;
        int8_t camera_direction = 0;
        int8_t leg_height_direction = 0;
        int8_t roll_direction = 0;
        action_request request = action_request::NONE;
        bool fresh = false;
        bool reset_leg = false;
        bool disable_leg_torque = false;
        bool exit_action = false;
        bool middle_calibration_request = false;
    };
}

#endif
