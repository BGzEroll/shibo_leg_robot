#include "actions.h"

#include "actions/action_common.h"
#include "actions/action_jump.h"
#include "actions/action_kick.h"
#include "actions/action_sit.h"
#include "xbox.h"

namespace action = controller::actions;

using controller::action_io;
using controller::action_state;
using controller::balance_request;
using controller::jump_command;
using controller::mode_id;

/**
 * @brief 切换动作模式并初始化该模式的运行状态
 *
 * @param state 动作状态机状态
 * @param mode 目标动作模式
 * @param jump 跳跃动作类型
 */
void controller::actions::begin_mode(action_state &state, mode_id mode, jump_command jump)
{
    state.mode = mode;
    state.phase = action::PREPARE;
    state.timer = 0;
    state.ready_timer = 0;
    state.elapsed = 0;
    state.jump = jump_runtime{};
    state.kick = kick_runtime{};

    if(mode != mode_id::JUMP){return;}

    if(jump == jump_command::FORWARD){state.jump.linear_dir = 1;}
    if(jump == jump_command::BACKWARD){state.jump.linear_dir = -1;}
    if(jump == jump_command::TURN_LEFT){state.jump.turn_dir = 1;}
    if(jump == jump_command::TURN_RIGHT){state.jump.turn_dir = -1;}
}

/**
 * @brief 更新 BOOT 模式状态机并生成平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 * @param tick_ms 本次更新周期，单位毫秒
 *
 * @return 生成的平衡请求
 */
static balance_request update_boot(action_state &state, action_io &ctx, uint32_t tick_ms)
{
    balance_request cmd;
    switch(state.phase)
    {
        case action::PREPARE:
            action::set_torque(0);
            state.phase = action::WAIT_SIGNAL;
            break;

        case action::WAIT_SIGNAL:
            if(ctx.input.buttons & BTN_RB){state.phase = action::INIT;}
            break;

        case action::INIT:
            action::set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
            action::reset_leg(ctx.leg);
            cmd.command.reset_reference = true;
            state.phase = action::INIT_PREPARE;
            break;

        case action::INIT_PREPARE:
            if((state.timer += tick_ms) >= 350)
            {
                state.timer = 0;
                state.elapsed = 0;
                state.ready_timer = 0;
                cmd.command.reset_reference = true;
                state.phase = action::INIT_RECOVER;
            }
            break;

        case action::INIT_RECOVER:
            cmd = action::recover_command(state, ctx);
            if(action::recover_ready(state, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                cmd.command.reset_reference = true;
                action::begin_mode(state, mode_id::BALANCE);
            }
            break;
    }
    return cmd;
}

/**
 * @brief 更新 BALANCE 模式状态机并生成平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 *
 * @return 生成的平衡请求
 */
static balance_request update_balance(action_state &state, action_io &ctx)
{
    balance_request cmd;
    cmd.command.enable_balance = true;
    cmd.command.enable_motor = true;
    cmd.command.enable_steering = true;
    cmd.target.linear_vel = ctx.input.linear_cmd;
    cmd.target.yaw_rate = ctx.input.yaw_cmd;
    action::run_leg_control(ctx);

    if((ctx.input.pressed_buttons & BTN_LS) &&
       fabsf(ctx.input.linear_cmd) < ctx.max_linear_vel * 0.05f)
    {
        action::reset_leg(ctx.leg);
        cmd.command.reset_reference = true;
    }

    bool modifier = (ctx.input.buttons & BTN_SELECT) != 0;
    if(modifier && (ctx.input.pressed_buttons & BTN_X))
    {
        action::begin_mode(state, mode_id::KICK_PLACE);
        return cmd;
    }
    if(modifier && (ctx.input.pressed_buttons & BTN_Y))
    {
        action::begin_mode(state, mode_id::KICK_RUN);
        return cmd;
    }
    if(modifier && (ctx.input.pressed_buttons & BTN_B))
    {
        action::begin_mode(state, mode_id::BALANCE);
        return cmd;
    }

    if(ctx.input.pressed_buttons & BTN_LB){action::begin_mode(state, mode_id::SIT);}
    if(ctx.input.pressed_buttons & BTN_RS){action::begin_mode(state, mode_id::JUMP, jump_command::IN_PLACE);}
    if(ctx.input.pressed_buttons & BTN_Y){action::begin_mode(state, mode_id::JUMP, jump_command::FORWARD);}
    if(ctx.input.pressed_buttons & BTN_A){action::begin_mode(state, mode_id::JUMP, jump_command::BACKWARD);}
    if(ctx.input.pressed_buttons & BTN_X){action::begin_mode(state, mode_id::JUMP, jump_command::TURN_LEFT);}
    if(ctx.input.pressed_buttons & BTN_B){action::begin_mode(state, mode_id::JUMP, jump_command::TURN_RIGHT);}
    return cmd;
}

/**
 * @brief 初始化动作状态机
 *
 * @param state 动作状态机状态
 */
void controller::actions_init(action_state &state)
{
    action::begin_mode(state, mode_id::BOOT);
}

/**
 * @brief 获取当前动作模式
 *
 * @param state 动作状态机状态
 *
 * @return 当前动作模式
 */
controller::mode_id controller::actions_mode(const action_state &state)
{
    return state.mode;
}

/**
 * @brief 按当前动作模式更新状态机并生成平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 * @param tick_ms 本次更新周期，单位毫秒
 *
 * @return 生成的平衡请求
 */
controller::balance_request controller::actions_update(action_state &state, action_io &ctx, uint32_t tick_ms)
{
    if(state.mode != mode_id::STOP && (ctx.input.pressed_buttons & BTN_START))
    {
        action::begin_mode(state, mode_id::STOP);
        balance_request cmd;
        cmd.command.reset_reference = true;
        return cmd;
    }

    switch(state.mode)
    {
        case mode_id::BOOT:
            return update_boot(state, ctx, tick_ms);

        case mode_id::BALANCE:
            return update_balance(state, ctx);

        case mode_id::SIT:
            return action::update_sit(state, ctx, tick_ms);

        case mode_id::JUMP:
            return action::update_jump(state, ctx, tick_ms);

        case mode_id::KICK_PLACE:
            return action::update_kick_place(state, ctx, tick_ms);

        case mode_id::KICK_RUN:
            return action::update_kick_run(state, ctx, tick_ms);

        case mode_id::STOP:
            if(ctx.input.buttons & BTN_RB)
            {
                action::begin_mode(state, mode_id::BOOT);
            }
            return balance_request{};

        default:
            return balance_request{};
    }
}
