#ifndef ACTION_COMMON_H
#define ACTION_COMMON_H

#include "../actions.h"

namespace controller
{
    namespace actions
    {
        enum phase : uint8_t
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

        struct pose_step
        {
            int16_t left;
            int16_t right;
            uint16_t speed;
            uint8_t accel;
            uint32_t hold_ms;
        };

        float wrap_pi(float angle);
        float angle_error(float target, float current);
        void set_pose(int16_t left, int16_t right, uint16_t speed, uint8_t accel);
        void set_torque(uint8_t type);
        void reset_leg(leg_runtime &leg);
        void run_leg_control(action_io &ctx, float height_count_offset = 0.0f);
        void begin_mode(action_state &state, mode_id mode, jump_command jump = jump_command::IN_PLACE);
        bool recover_ready(action_state &state, const balance_core::motion_status &status, uint32_t tick_ms,
            float pitch_limit, float rate_limit, uint32_t hold_ms, uint32_t timeout_ms);
        balance_request recover_command(action_state &state, action_io &ctx);
        bool run_pose_sequence(action_state &state, const pose_step *steps, uint8_t count, uint32_t tick_ms);
    }
}

#endif
