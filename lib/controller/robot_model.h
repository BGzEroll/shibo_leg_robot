#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <Arduino.h>

namespace controller
{
    namespace robot_model
    {
        static constexpr float LEG_HEIGHT_BASE = 20.0f;
        static constexpr int16_t SERVO_LEFT_MIN = 2048 + 40;
        static constexpr int16_t SERVO_RIGHT_MIN = 2048 - 40;
        static constexpr int16_t SERVO_LEFT_MAX = 2048 + 450;
        static constexpr int16_t SERVO_RIGHT_MAX = 2048 - 450;
        static constexpr uint16_t CAMERA_SERVO_MIN = 0;
        static constexpr uint16_t CAMERA_SERVO_MAX = 180;
        static constexpr uint16_t FRONTIER_SERVO_MIN = 0;
        static constexpr uint16_t FRONTIER_SERVO_MAX = 180;
    }
}

#endif
