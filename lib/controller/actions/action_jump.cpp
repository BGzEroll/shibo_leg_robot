#include "action_jump.h"

/**
 * @brief 根据跳跃阶段生成跳跃过程中的平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 *
 * @return 生成的平衡请求
 */
static controller::balance_request update_jump_command(controller::action_state &state, controller::action_io &ctx)
{
    controller::balance_request cmd;
    bool linear_jump = state.jump.linear_dir != 0;
    bool yaw_jump = state.jump.turn_dir != 0 || linear_jump;
    cmd.mode = controller::balance_drive_mode::BALANCE;
    cmd.enable_steering = yaw_jump;
    cmd.enable_yaw_integral = yaw_jump;

    float push_vel = 0.0f;
    uint32_t push_ramp_ms = 80;
    if(state.jump.linear_dir > 0)
    {
        push_vel = min(ctx.max_linear_vel, 0.40f);
        push_ramp_ms = 160;
    }
    else if(state.jump.linear_dir < 0)
    {
        push_vel = min(ctx.max_linear_vel, 0.34f);
        push_ramp_ms = 240;
    }

    if(state.phase == controller::actions::PUSH)
    {
        float ramp = constrain((float)state.timer / (float)push_ramp_ms, 0.0f, 1.0f);
        state.jump.linear_cmd = (float)state.jump.linear_dir * push_vel * ramp;
    }
    else
    {
        state.jump.linear_cmd = 0.0f;
    }

    cmd.linear_vel = state.jump.linear_cmd;
    if(yaw_jump)
    {
        float err = controller::actions::angle_error(state.jump.target_yaw, ctx.status.yaw_angle);
        float ff = 0.0f;
        float kp = state.jump.turn_dir == 0 ? 3.0f : 1.0f;
        float max_rate = state.jump.turn_dir == 0 ? 1.8f : 0.6f;

        if(state.jump.turn_dir != 0)
        {
            if(state.phase == controller::actions::PUSH){ff = 1.2f; kp = 1.4f; max_rate = 1.8f;}
            if(state.phase == controller::actions::FLY){ff = 6.4f; kp = 2.0f; max_rate = 6.4f;}
            if(state.phase == controller::actions::LAND){kp = 0.35f; max_rate = 0.4f;}
            if(state.phase == controller::actions::RECOVER){kp = 0.8f; max_rate = 0.5f;}
        }

        state.jump.yaw_cmd = constrain((float)state.jump.turn_dir * ff + kp * err, -max_rate, max_rate);
        cmd.yaw_rate = state.jump.yaw_cmd;
    }

    if(state.jump.linear_dir == 0 || state.phase != controller::actions::PUSH){cmd.enable_linear_feedback = false;}
    if(!yaw_jump){cmd.enable_yaw_feedback = false;}
    return cmd;
}

/**
 * @brief 更新 JUMP 模式状态机并生成平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 * @param tick_ms 本次更新周期，单位毫秒
 *
 * @return 生成的平衡请求
 */
controller::balance_request controller::actions::update_jump(controller::action_state &state, controller::action_io &ctx, uint32_t tick_ms)
{
    controller::balance_request cmd = update_jump_command(state, ctx);

    switch(state.phase)
    {
        case controller::actions::PREPARE:
            state.jump.target_yaw = controller::actions::wrap_pi(ctx.status.yaw_angle +
                                            (float)state.jump.turn_dir * PI * 0.5f);
            controller::actions::set_pose(SERVO_LEFT_MIN + 60, SERVO_RIGHT_MIN - 60, 450, 250);
            cmd.reset_yaw_integral = true;
            state.phase = controller::actions::PUSH;
            state.timer = 0;
            break;

        case controller::actions::PUSH:
        {
            uint32_t wait_ms = state.jump.linear_dir > 0 ? 650 : (state.jump.linear_dir < 0 ? 700 : 200);
            if((state.timer += tick_ms) >= wait_ms)
            {
                controller::actions::set_pose(SERVO_LEFT_MAX + 20, SERVO_RIGHT_MAX - 20, 0, 0);
                state.timer = 0;
                state.phase = controller::actions::FLY;
            }
            break;
        }

        case controller::actions::FLY:
            if((state.timer += tick_ms) >= 130)
            {
                controller::actions::set_pose(SERVO_LEFT_MIN + 60, SERVO_RIGHT_MIN - 60, 0, 0);
                state.timer = 0;
                state.phase = controller::actions::LAND;
            }
            break;

        case controller::actions::LAND:
            if((state.timer += tick_ms) >= 260)
            {
                state.timer = 0;
                state.elapsed = 0;
                state.phase = controller::actions::RECOVER;
            }
            break;

        case controller::actions::RECOVER:
            state.elapsed += tick_ms;
            if(fabsf(ctx.status.pitch_angle) < 0.18f &&
               fabsf(ctx.status.pitch_rate) < 1.6f)
            {
                state.timer += tick_ms;
            }
            else
            {
                state.timer = 0;
            }

            if(state.timer >= 80 || state.elapsed >= 350)
            {
                cmd.reset_yaw_integral = true;
                controller::actions::begin_mode(state, controller::mode_id::BALANCE);
            }
            break;
    }

    return cmd;
}
