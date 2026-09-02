#ifndef CONTROL_BALANCE_H
#define CONTROL_BALANCE_H

#include <Arduino.h>

namespace control
{
    enum class balance_mode : uint8_t
    {
        OFF = 0,
        BALANCE,
        DIRECT,
        RECOVER
    };

    /**
     * @brief 动作层直接交给平衡算法的统一控制命令
     */
    struct balance_command
    {
        balance_mode mode = balance_mode::OFF;
        float linear_vel = 0.0f;
        float yaw_rate = 0.0f;
        float direct_left = 0.0f;
        float direct_right = 0.0f;
        float recover_blend = 1.0f;
        bool steering = false;
        bool linear_feedback = true;
        bool yaw_feedback = true;
        bool yaw_integral = true;
        bool reset_reference = false;
        bool reset_yaw_integral = false;
    };

    /**
     * @brief control task 发送给 FOC task 的最新电机命令
     */
    struct motor_command
    {
        uint32_t timestamp_us = 0;
        float left = 0.0f;
        float right = 0.0f;
        bool enabled = false;
    };

    /**
     * @brief 一个控制周期使用的传感器快照
     */
    struct sensor_snapshot
    {
        bool imu_valid = false;
        bool encoder_valid = false;
        uint32_t timestamp_us = 0;
        float pitch_angle = 0.0f;
        float pitch_rate = 0.0f;
        float yaw_angle = 0.0f;
        float yaw_rate = 0.0f;
        float roll_angle = 0.0f;
        float avg_linear_pos = 0.0f;
        float avg_linear_vel = 0.0f;
        float leg_height[2]{};
        int16_t servo_position[2]{};
        float avg_leg_height = 0.0f;
    };

    /**
     * @brief 平衡算法输出及调试状态
     */
    struct status
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
        float avg_leg_height = 0.0f;
        motor_command motor;
    };

    struct info
    {
        float max_linear_vel = 0.0f;
        float max_steer_vel = 0.0f;
    };

    namespace balance
    {
        info get_info();
        status step(const balance_command &command, const sensor_snapshot &sensor,
            uint32_t tick_ms);
        void init();
    }
}

#endif
