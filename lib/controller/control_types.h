#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

#include <Arduino.h>
#include "SimpleFOC.h"
#include "lqi.h"
#include "sts3032.h"
#include "xbox.h"

namespace controller {

enum class mode_id : uint8_t {
    BOOT = 0,
    BALANCE,
    SIT,
    JUMP,
    STOP
};

enum class jump_command : uint8_t {
    IN_PLACE = 0,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT
};

struct control_input {
    uint16_t raw_buttons = 0;
    uint16_t buttons = 0;
    uint16_t pressed_buttons = 0;
    float axes[6]{};
    float linear_cmd = 0.0f;
    float yaw_cmd = 0.0f;
};

struct balance_command {
    bool enable_motor = false;
    bool enable_balance = false;
    bool enable_steering = false;

    bool reset_reference = false;
    bool reset_yaw_integral = false;

    float target_linear_vel = 0.0f;
    float target_yaw_rate = 0.0f;

    bool manual_output = false;
    float manual_left = 0.0f;
    float manual_right = 0.0f;

    bool suppress_linear_feedback = false;
    bool suppress_yaw_feedback = false;
    bool suppress_yaw_integral = false;
    bool recover_active = false;
    float output_blend = 1.0f;
};

struct sensor_snapshot {
    bool imu_valid = false;
    bool encoder_valid = false;
    uint32_t timestamp_us = 0;
    lqi::feedback_state feedback{};
    float roll_angle = 0.0f;
    float leg_height[2]{};
    float avg_leg_height = 0.0f;
    int16_t servo_position[2]{};
};

struct balance_status {
    uint32_t timestamp_us = 0;
    mode_id mode = mode_id::BOOT;
    lqi::feedback_state feedback{};
    lqi::reference_state reference{};
    float input[2]{};
    float feedback_vector[6]{};
    float output[2]{};
    float roll_angle = 0.0f;
    float leg_height[2]{};
    float avg_leg_height = 0.0f;
};

struct leg_runtime {
    void reset_roll_pid()
    {
        roll_pid = PIDController{8.0f, 30.0f, 0.0f, 100000.0f, 450.0f};
    }

    float roll_adjust = 0.0f;
    float height_base = (float)LEG_HEIGHT_BASE;
    PIDController roll_pid{8.0f, 30.0f, 0.0f, 100000.0f, 450.0f};
    LowPassFilter roll_lpf{0.3f};
};

float servo_count_to_height(int16_t position);

}

#endif
