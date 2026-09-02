#include "action_internal.h"

#include "hw/servo.h"
#include <math.h>

/* ---- JUMP ---- */

/**
 * @brief 按跳跃方向初始化运行参数
 *
 * @param data 跳跃运行状态
 * @param direction 跳跃方向
 */
void control::action::internal::set_jump_direction(
    control::action::jump_runtime &data, control::action::jump_direction direction)
{
    data = control::action::jump_runtime{};
    if(direction == control::action::jump_direction::FORWARD){data.linear_dir = 1;}
    if(direction == control::action::jump_direction::BACKWARD){data.linear_dir = -1;}
    if(direction == control::action::jump_direction::TURN_LEFT){data.turn_dir = 1;}
    if(direction == control::action::jump_direction::TURN_RIGHT){data.turn_dir = -1;}
}

/**
 * @brief 生成跳跃阶段使用的平衡命令
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 *
 * @return 跳跃平衡命令
 */
static control::balance_command update_jump_command(
    control::action::state &state, control::action::context &ctx)
{
    control::action::jump_runtime &jump = state.jump_data;
    control::action::action_runtime &runtime = state.jump;
    control::balance_command command;
    bool linear_jump = jump.linear_dir != 0;
    bool yaw_jump = jump.turn_dir != 0 || linear_jump;
    command.mode = control::balance_mode::BALANCE;
    command.steering = yaw_jump;
    command.yaw_integral = yaw_jump;

    float push_velocity = 0.0f;
    uint32_t push_ramp_ms = 80;
    if(jump.linear_dir > 0)
    {
        push_velocity = min(ctx.max_linear_vel, 0.40f);
        push_ramp_ms = 160;
    }
    else if(jump.linear_dir < 0)
    {
        push_velocity = min(ctx.max_linear_vel, 0.34f);
        push_ramp_ms = 240;
    }

    if(runtime.current_phase == control::action::phase::PUSH)
    {
        float ramp = constrain((float)runtime.timer / (float)push_ramp_ms, 0.0f, 1.0f);
        jump.linear_cmd = (float)jump.linear_dir * push_velocity * ramp;
    }
    else
    {
        jump.linear_cmd = 0.0f;
    }
    command.linear_vel = jump.linear_cmd;

    if(yaw_jump)
    {
        float error = control::action::internal::angle_error(
            jump.target_yaw, ctx.status.yaw_angle);
        float feedforward = 0.0f;
        float proportional = jump.turn_dir == 0 ? 3.0f : 1.0f;
        float max_rate = jump.turn_dir == 0 ? 1.8f : 0.6f;
        if(jump.turn_dir != 0)
        {
            if(runtime.current_phase == control::action::phase::PUSH)
            {
                feedforward = 1.2f;
                proportional = 1.4f;
                max_rate = 1.8f;
            }
            if(runtime.current_phase == control::action::phase::FLY)
            {
                feedforward = 6.4f;
                proportional = 2.0f;
                max_rate = 6.4f;
            }
            if(runtime.current_phase == control::action::phase::LAND)
            {
                proportional = 0.35f;
                max_rate = 0.4f;
            }
            if(runtime.current_phase == control::action::phase::RECOVER)
            {
                proportional = 0.8f;
                max_rate = 0.5f;
            }
        }
        jump.yaw_cmd = constrain(
            (float)jump.turn_dir * feedforward + proportional * error,
            -max_rate, max_rate);
        command.yaw_rate = jump.yaw_cmd;
    }

    if(jump.linear_dir == 0 || runtime.current_phase != control::action::phase::PUSH)
    {
        command.linear_feedback = false;
    }
    if(!yaw_jump){command.yaw_feedback = false;}
    return command;
}

/**
 * @brief 执行 JUMP 状态机的一步
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 * @param tick_ms 周期，单位毫秒
 *
 * @return 本周期动作结果
 */
control::action::internal::step_result control::action::internal::step_jump(
    control::action::state &state, control::action::context &ctx, uint32_t tick_ms)
{
    control::action::internal::step_result result;
    result.balance = update_jump_command(state, ctx);
    control::action::action_runtime &runtime = state.jump;
    control::action::jump_runtime &jump = state.jump_data;
    switch(runtime.current_phase)
    {
        case control::action::phase::PREPARE:
            jump.target_yaw = control::action::internal::wrap_pi(
                ctx.status.yaw_angle + (float)jump.turn_dir * PI * 0.5f);
            control::action::internal::set_pose(
                hw::servo::LEG_LEFT_MIN + 60, hw::servo::LEG_RIGHT_MIN - 60, 450, 250);
            result.balance.reset_yaw_integral = true;
            runtime.current_phase = control::action::phase::PUSH;
            runtime.timer = 0;
            break;

        case control::action::phase::PUSH:
        {
            uint32_t wait_ms = jump.linear_dir > 0 ? 650 :
                (jump.linear_dir < 0 ? 700 : 200);
            runtime.timer += tick_ms;
            if(runtime.timer >= wait_ms)
            {
                control::action::internal::set_pose(
                    hw::servo::LEG_LEFT_MAX + 20, hw::servo::LEG_RIGHT_MAX - 20, 0, 0);
                runtime.timer = 0;
                runtime.current_phase = control::action::phase::FLY;
            }
            break;
        }

        case control::action::phase::FLY:
            runtime.timer += tick_ms;
            if(runtime.timer >= 130)
            {
                control::action::internal::set_pose(
                    hw::servo::LEG_LEFT_MIN + 60, hw::servo::LEG_RIGHT_MIN - 60, 0, 0);
                runtime.timer = 0;
                runtime.current_phase = control::action::phase::LAND;
            }
            break;

        case control::action::phase::LAND:
            runtime.timer += tick_ms;
            if(runtime.timer >= 260)
            {
                runtime.timer = 0;
                runtime.elapsed = 0;
                runtime.current_phase = control::action::phase::RECOVER;
            }
            break;

        case control::action::phase::RECOVER:
            runtime.elapsed += tick_ms;
            if(fabsf(ctx.status.pitch_angle) < 0.18f &&
               fabsf(ctx.status.pitch_rate) < 1.6f)
            {
                runtime.timer += tick_ms;
            }
            else
            {
                runtime.timer = 0;
            }
            if(runtime.timer >= 80 || runtime.elapsed >= 350)
            {
                result.balance.reset_yaw_integral = true;
                result.next = control::action::internal::transition::DONE;
            }
            break;

        default:
            break;
    }
    return result;
}
