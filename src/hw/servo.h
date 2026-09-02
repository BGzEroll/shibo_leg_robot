#ifndef HW_SERVO_H
#define HW_SERVO_H

#include <Arduino.h>
#include "pwm_servo.h"

namespace hw
{
    namespace servo
    {
        constexpr uint8_t LEG_LEFT = 1;
        constexpr uint8_t LEG_RIGHT = 2;
        constexpr int16_t LEG_HEIGHT_BASE = 20;
        constexpr int16_t LEG_LEFT_MIN = 2048 + 40;
        constexpr int16_t LEG_RIGHT_MIN = 2048 - 40;
        constexpr int16_t LEG_LEFT_MAX = 2048 + 450;
        constexpr int16_t LEG_RIGHT_MAX = 2048 - 450;
        constexpr uint16_t CAMERA_MIN = 0;
        constexpr uint16_t CAMERA_MAX = 180;
        constexpr uint16_t FRONTIER_MIN = 0;
        constexpr uint16_t FRONTIER_MAX = 180;

        struct state
        {
            uint8_t id = 0;
            int16_t position = 0;
            int16_t load = 0;
        };

        extern state leg_status[2];
        extern pwm_servo camera;
        extern pwm_servo frontier;

        void get_position_and_load();
        void set_torque(uint8_t id, uint8_t type);
        void set(uint8_t id, int16_t position, int16_t speed, uint8_t acceleration);
        void move();
        void calibrate_middle();
        void init();
    }
}

#endif
