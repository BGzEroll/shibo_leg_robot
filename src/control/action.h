#ifndef CONTROL_ACTION_H
#define CONTROL_ACTION_H

#include "balance.h"
#include "input.h"
#include "SimpleFOC.h"

namespace control
{
    namespace action
    {
        constexpr int16_t LEG_HEIGHT_BASE = 20;

        enum class mode : uint8_t
        {
            BOOT = 0,
            BALANCE,
            SIT,
            JUMP,
            STOP,
            KICK_PLACE,
            KICK_RUN,
            MIDDLE_CALIBRATION
        };

        enum class jump_direction : uint8_t
        {
            IN_PLACE = 0,
            FORWARD,
            BACKWARD,
            TURN_LEFT,
            TURN_RIGHT
        };

        enum class phase : uint8_t
        {
            PREPARE = 0,
            WAIT_SIGNAL,
            INIT,
            INIT_PREPARE,
            INIT_RECOVER,
            MOVING,
            DONE,
            EXIT_PREPARE,
            EXIT_RECOVER,
            PUSH,
            FLY,
            LAND,
            RECOVER
        };

        struct action_runtime
        {
            phase current_phase = phase::PREPARE;
            uint32_t timer = 0;
            uint32_t ready_timer = 0;
            uint32_t elapsed = 0;
        };

        struct leg_runtime
        {
            void reset_roll_pid()
            {
                roll_pid = PIDController{8.0f, 30.0f, 0.0f, 100000.0f, 450.0f};
            }

            float roll_adjust = 0.0f;
            float height_base = (float)LEG_HEIGHT_BASE;
            PIDController roll_pid{8.0f, 30.0f, 0.0f, 100000.0f, 450.0f};
            LowPassFilter roll_lpf{0.3f};
        };

        struct jump_runtime
        {
            int8_t linear_dir = 0;
            int8_t turn_dir = 0;
            float target_yaw = 0.0f;
            float linear_cmd = 0.0f;
            float yaw_cmd = 0.0f;
        };

        struct kick_runtime
        {
            float cam_angle = 90.0f;
            float target_yaw = 0.0f;
            int16_t last_dy = 0;
            uint16_t frontier_angle = 181;
            uint32_t last_dy_time = 0;
            uint32_t last_vision_seq = 0;
            uint32_t kick_timer = 0;
            uint32_t kick_cooldown_timer = 0;
            uint32_t post_timer = 0;
            bool chased = false;
            bool aligned = false;
            bool kicking = false;
            bool post_kick = false;
            float cam_error = 0.0f;
            float cam_rate = 0.0f;
            float yaw_rate = 0.0f;
        };

        struct state
        {
            mode current_mode = mode::BOOT;
            action_runtime boot;
            action_runtime sit;
            action_runtime jump;
            action_runtime kick_place;
            action_runtime kick_run;
            jump_runtime jump_data;
            kick_runtime kick_place_data;
            kick_runtime kick_run_data;
            bool sit_exit_locked = false;
            bool middle_calibration_success = false;
        };

        struct context
        {
            context(control_input &input_value, const control::status &status_value,
                leg_runtime &leg_value, float max_linear_value,
                float max_steer_value, bool battery_is_valid, bool battery_is_low,
                int16_t servo_left_value, int16_t servo_right_value,
                bool vision_is_valid, int16_t vision_x, int16_t vision_y,
                uint32_t vision_sequence_value)
                : input(input_value),
                  status(status_value),
                  leg(leg_value),
                  max_linear_vel(max_linear_value),
                  max_steer_vel(max_steer_value),
                  battery_valid(battery_is_valid),
                  battery_low(battery_is_low),
                  servo_left_position(servo_left_value),
                  servo_right_position(servo_right_value),
                  vision_valid(vision_is_valid),
                  vision_dx(vision_x),
                  vision_dy(vision_y),
                  vision_sequence(vision_sequence_value)
            {
            }

            control_input &input;
            const control::status &status;
            leg_runtime &leg;
            float max_linear_vel = 0.0f;
            float max_steer_vel = 0.0f;
            bool battery_valid = false;
            bool battery_low = false;
            int16_t servo_left_position = 0;
            int16_t servo_right_position = 0;
            bool vision_valid = false;
            int16_t vision_dx = 0;
            int16_t vision_dy = 0;
            uint32_t vision_sequence = 0;
        };

        void init(state &state);
        mode current_mode(const state &state);
        balance_command step(state &state, context &ctx, uint32_t tick_ms);
    }
}

#endif
