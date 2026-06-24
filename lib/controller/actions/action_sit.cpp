#include "action_sit.h"

#include "xbox.h"

namespace action = controller::actions;

/**
 * @brief 更新 SIT 模式状态机并生成平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 * @param tick_ms 本次更新周期，单位毫秒
 *
 * @return 生成的平衡请求
 */
controller::balance_request controller::actions::update_sit(action_state &state, action_io &ctx, uint32_t tick_ms)
{
    balance_request cmd;
    switch(state.phase)
    {
        case action::PREPARE:
            set_torque(2);
            state.timer = 0;
            state.phase = action::MOVING;
            break;

        case action::MOVING:
            state.timer += tick_ms;
            if(fabsf(ctx.status.pitch_angle) >= 0.25f || state.timer >= 700)
            {
                state.timer = 0;
                cmd.command.enable_motor = false;
                cmd.command.reset_reference = true;
                state.phase = action::DONE;
                break;
            }
            cmd.command.enable_motor = true;
            cmd.command.direct_output = true;
            cmd.target.direct_left = -0.15f;
            cmd.target.direct_right = -0.15f;
            break;

        case action::DONE:
            cmd.command.reset_reference = true;
            if(ctx.input.buttons & BTN_LS){set_torque(0);}
            if(ctx.input.buttons & BTN_RB)
            {
                set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
                reset_leg(ctx.leg);
                cmd.command.reset_reference = true;
                state.phase = action::EXIT_PREPARE;
            }
            break;

        case action::EXIT_PREPARE:
            if((state.timer += tick_ms) >= 350)
            {
                state.timer = 0;
                state.elapsed = 0;
                state.ready_timer = 0;
                cmd.command.reset_reference = true;
                state.phase = action::EXIT_RECOVER;
            }
            break;

        case action::EXIT_RECOVER:
            cmd = recover_command(state, ctx);
            if(recover_ready(state, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                cmd.command.reset_reference = true;
                begin_mode(state, mode_id::BALANCE);
            }
            break;
    }
    return cmd;
}
