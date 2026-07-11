#ifndef ACTION_COMMON_H
#define ACTION_COMMON_H

#include "../actions.h"

namespace controller
{
    namespace actions
    {
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
        void set_pose(action_io &ctx, int16_t left, int16_t right, uint16_t speed, uint8_t accel);
        void set_torque(action_io &ctx, uint8_t type);
        void reset_leg(leg_runtime &leg);
        void run_leg_control(action_io &ctx, float height_count_offset = 0.0f);
        bool recover_ready(action_runtime &runtime, const motion_status &status, uint32_t tick_ms,
            float pitch_limit, float rate_limit, uint32_t hold_ms, uint32_t timeout_ms);
        balance_request recover_command(action_runtime &runtime, action_io &ctx);
        bool run_pose_sequence(action_runtime &runtime, action_io &ctx, const pose_step *steps,
            uint8_t count, uint32_t tick_ms);
    }
}

#endif
