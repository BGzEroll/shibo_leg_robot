#include "actions.h"

#include "actions/action_common.h"
#include "actions/action_jump.h"
#include "actions/action_kick.h"
#include "actions/action_sit.h"
#include "xbox.h"

/**
 * @brief 切换动作模式并初始化该模式的运行状态
 *
 * @param state 动作状态机状态
 * @param mode 目标动作模式
 * @param jump 跳跃动作类型
 */
void controller::actions::begin_mode(controller::action_state &state, controller::mode_id mode, controller::jump_command jump)
{
    state.mode = mode;
    state.phase = controller::actions::PREPARE;
    state.timer = 0;
    state.ready_timer = 0;
    state.elapsed = 0;
    state.jump = controller::jump_runtime{};
    state.kick = controller::kick_runtime{};

    if(mode != controller::mode_id::JUMP){return;}

    if(jump == controller::jump_command::FORWARD){state.jump.linear_dir = 1;}
    if(jump == controller::jump_command::BACKWARD){state.jump.linear_dir = -1;}
    if(jump == controller::jump_command::TURN_LEFT){state.jump.turn_dir = 1;}
    if(jump == controller::jump_command::TURN_RIGHT){state.jump.turn_dir = -1;}
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
static controller::balance_request update_boot(controller::action_state &state, controller::action_io &ctx, uint32_t tick_ms)
{
    controller::balance_request cmd;
    switch(state.phase)
    {
        case controller::actions::PREPARE:
            controller::actions::set_torque(0);
            state.phase = controller::actions::WAIT_SIGNAL;
            break;

        case controller::actions::WAIT_SIGNAL:
            if(ctx.input.buttons & BTN_RB){state.phase = controller::actions::INIT;}
            break;

        case controller::actions::INIT:
            controller::actions::set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
            controller::actions::reset_leg(ctx.leg);
            state.phase = controller::actions::INIT_PREPARE;
            break;

        case controller::actions::INIT_PREPARE:
            if((state.timer += tick_ms) >= 350)
            {
                state.timer = 0;
                state.elapsed = 0;
                state.ready_timer = 0;
                state.phase = controller::actions::INIT_RECOVER;
            }
            break;

        case controller::actions::INIT_RECOVER:
            cmd = controller::actions::recover_command(state, ctx);
            if(controller::actions::recover_ready(state, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                controller::actions::begin_mode(state, controller::mode_id::BALANCE);
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
static controller::balance_request update_balance(controller::action_state &state, controller::action_io &ctx)
{
    controller::balance_request cmd;
    cmd.command.enable_balance = true;
    cmd.command.enable_motor = true;
    cmd.command.enable_steering = true;
    cmd.target.linear_vel = ctx.input.linear_cmd;
    cmd.target.yaw_rate = ctx.input.yaw_cmd;
    controller::actions::run_leg_control(ctx);

    if((ctx.input.pressed_buttons & BTN_LS) &&
       fabsf(ctx.input.linear_cmd) < ctx.max_linear_vel * 0.05f)
    {
        controller::actions::reset_leg(ctx.leg);
    }

    if(ctx.input.middle_calibration_request)
    {
        controller::actions::begin_mode(state, controller::mode_id::MIDDLE_CALIBRATION);
        return cmd;
    }

    bool modifier = (ctx.input.buttons & BTN_SELECT) != 0;
    if(modifier && (ctx.input.pressed_buttons & BTN_X))
    {
        controller::actions::begin_mode(state, controller::mode_id::KICK_PLACE);
        return cmd;
    }
    if(modifier && (ctx.input.pressed_buttons & BTN_Y))
    {
        controller::actions::begin_mode(state, controller::mode_id::KICK_RUN);
        return cmd;
    }
    if(modifier && (ctx.input.pressed_buttons & BTN_B))
    {
        controller::actions::begin_mode(state, controller::mode_id::BALANCE);
        return cmd;
    }

    if(ctx.input.pressed_buttons & BTN_LB){controller::actions::begin_mode(state, controller::mode_id::SIT);}
    if(ctx.input.pressed_buttons & BTN_RS){controller::actions::begin_mode(state, controller::mode_id::JUMP, controller::jump_command::IN_PLACE);}
    if(ctx.input.pressed_buttons & BTN_Y){controller::actions::begin_mode(state, controller::mode_id::JUMP, controller::jump_command::FORWARD);}
    if(ctx.input.pressed_buttons & BTN_A){controller::actions::begin_mode(state, controller::mode_id::JUMP, controller::jump_command::BACKWARD);}
    if(ctx.input.pressed_buttons & BTN_X){controller::actions::begin_mode(state, controller::mode_id::JUMP, controller::jump_command::TURN_LEFT);}
    if(ctx.input.pressed_buttons & BTN_B){controller::actions::begin_mode(state, controller::mode_id::JUMP, controller::jump_command::TURN_RIGHT);}
    return cmd;
}

/**
 * @brief 初始化动作状态机
 *
 * @param state 动作状态机状态
 */
void controller::actions_init(controller::action_state &state)
{
    controller::actions::begin_mode(state, controller::mode_id::BOOT);
}

/**
 * @brief 获取当前动作模式
 *
 * @param state 动作状态机状态
 *
 * @return 当前动作模式
 */
controller::mode_id controller::actions_mode(const controller::action_state &state)
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
controller::balance_request controller::actions_update(controller::action_state &state, controller::action_io &ctx, uint32_t tick_ms)
{
    if(state.mode != controller::mode_id::STOP && (ctx.input.pressed_buttons & BTN_START))
    {
        controller::actions::begin_mode(state, controller::mode_id::STOP);
        return controller::balance_request{};
    }

    switch(state.mode)
    {
        case controller::mode_id::BOOT:
            return update_boot(state, ctx, tick_ms);

        case controller::mode_id::BALANCE:
            return update_balance(state, ctx);

        case controller::mode_id::SIT:
            return controller::actions::update_sit(state, ctx, tick_ms);

        case controller::mode_id::JUMP:
            return controller::actions::update_jump(state, ctx, tick_ms);

        case controller::mode_id::KICK_PLACE:
            return controller::actions::update_kick_place(state, ctx, tick_ms);

        case controller::mode_id::KICK_RUN:
            return controller::actions::update_kick_run(state, ctx, tick_ms);

        case controller::mode_id::MIDDLE_CALIBRATION:
            return controller::actions::update_middle_calibration(state, ctx, tick_ms);

        case controller::mode_id::STOP:
            if(ctx.input.buttons & BTN_RB)
            {
                controller::actions::begin_mode(state, controller::mode_id::BOOT);
            }
            return controller::balance_request{};

        default:
            return controller::balance_request{};
    }
}
