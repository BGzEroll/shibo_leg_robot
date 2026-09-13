#ifndef CONTROL_ACTION_INTERNAL_H
#define CONTROL_ACTION_INTERNAL_H

#include "action.h"

namespace control
{
    namespace action
    {
        namespace internal
        {
            enum class transition : uint8_t
            {
                NONE = 0,
                DONE,
                BOOT,
                KICK_PLACE,
                KICK_RUN,
                SIT,
                MIDDLE_CALIBRATION,
                JUMP,
                RESET_BALANCE
            };

            struct step_result
            {
                control::balance_command balance;
                transition next = transition::NONE;
                control::action::jump_direction jump =
                    control::action::jump_direction::IN_PLACE;
            };

            float wrap_pi(float angle);
            float angle_error(float target, float current);
            void set_pose(int16_t left, int16_t right, uint16_t speed,
                uint8_t acceleration);
            void set_torque(uint8_t type);
            void reset_leg(control::action::leg_runtime &leg);
            void run_leg_control(control::action::context &ctx,
                float height_count_offset = 0.0f);
            bool recover_ready(control::action::action_runtime &runtime,
                const control::status &status, uint32_t tick_ms, float pitch_limit,
                float rate_limit, uint32_t hold_ms, uint32_t timeout_ms);
            control::balance_command recover_command(
                control::action::action_runtime &runtime,
                control::action::context &ctx);

            step_result step_boot(control::action::state &state,
                control::action::context &ctx, uint32_t tick_ms);
            step_result step_balance(control::action::state &state,
                control::action::context &ctx, uint32_t tick_ms);
            step_result step_stop(control::action::state &state,
                control::action::context &ctx, uint32_t tick_ms);
            void set_jump_direction(
                control::action::jump_runtime &data,
                control::action::jump_direction direction);
            step_result step_sit(control::action::state &state,
                control::action::context &ctx, uint32_t tick_ms);
            step_result step_middle_calibration(control::action::state &state,
                control::action::context &ctx, uint32_t tick_ms);
            step_result step_jump(control::action::state &state,
                control::action::context &ctx, uint32_t tick_ms);
            step_result step_kick(control::action::state &state,
                control::action::context &ctx, uint32_t tick_ms, bool run_mode);

            void enter_mode(control::action::state &state,
                control::action::mode next_mode,
                control::action::jump_direction jump =
                    control::action::jump_direction::IN_PLACE);
            void update_sit_exit_lock(control::action::state &state,
                const control::action::context &ctx);
            void apply_transition(control::action::state &state,
                const step_result &result);
        }
    }
}

#endif
