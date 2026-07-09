#include "actions.h"

#include "actions/action_common.h"
#include "actions/action_jump.h"
#include "actions/action_kick.h"
#include "actions/action_sit.h"

/* ---- 动作模式切换 ---- */

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

/* ---- 基础动作状态机 ---- */

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
            if(ctx.input.request == controller::action_request::BOOT_CONFIRM)
            {
                state.phase = controller::actions::INIT;
            }
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
    cmd.mode = controller::balance_drive_mode::BALANCE;
    cmd.enable_steering = true;
    cmd.linear_vel = ctx.input.linear_cmd;
    cmd.yaw_rate = ctx.input.yaw_cmd;
    controller::actions::run_leg_control(ctx);

    if(ctx.input.reset_leg)
    {
        controller::actions::reset_leg(ctx.leg);
    }

    return cmd;
}

/**
 * @brief 应用 BALANCE 模式产生的动作切换请求
 *
 * @param state 动作状态机状态
 * @param request 动作切换请求
 */
static void route_balance_request(controller::action_state &state, controller::action_request request)
{
    switch(request)
    {
        case controller::action_request::RESET_BALANCE:
            controller::actions::begin_mode(state, controller::mode_id::BALANCE);
            break;

        case controller::action_request::SIT:
            controller::actions::begin_mode(state, controller::mode_id::SIT);
            break;

        case controller::action_request::JUMP_IN_PLACE:
            controller::actions::begin_mode(
                state, controller::mode_id::JUMP, controller::jump_command::IN_PLACE);
            break;

        case controller::action_request::JUMP_FORWARD:
            controller::actions::begin_mode(
                state, controller::mode_id::JUMP, controller::jump_command::FORWARD);
            break;

        case controller::action_request::JUMP_BACKWARD:
            controller::actions::begin_mode(
                state, controller::mode_id::JUMP, controller::jump_command::BACKWARD);
            break;

        case controller::action_request::JUMP_LEFT:
            controller::actions::begin_mode(
                state, controller::mode_id::JUMP, controller::jump_command::TURN_LEFT);
            break;

        case controller::action_request::JUMP_RIGHT:
            controller::actions::begin_mode(
                state, controller::mode_id::JUMP, controller::jump_command::TURN_RIGHT);
            break;

        case controller::action_request::KICK_PLACE:
            controller::actions::begin_mode(state, controller::mode_id::KICK_PLACE);
            break;

        case controller::action_request::KICK_RUN:
            controller::actions::begin_mode(state, controller::mode_id::KICK_RUN);
            break;

        default:
            break;
    }
}

/* ---- 动作调度 API ---- */

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
    if(state.mode != controller::mode_id::STOP &&
       ctx.input.request == controller::action_request::STOP)
    {
        controller::actions::begin_mode(state, controller::mode_id::STOP);
        return controller::balance_request{};
    }

    switch(state.mode)
    {
        case controller::mode_id::BOOT:
            return update_boot(state, ctx, tick_ms);

        case controller::mode_id::BALANCE:
        {
            controller::balance_request cmd = update_balance(state, ctx);
            if(ctx.input.middle_calibration_request)
            {
                controller::actions::begin_mode(state, controller::mode_id::MIDDLE_CALIBRATION);
                return cmd;
            }
            route_balance_request(state, ctx.input.request);
            return cmd;
        }

        case controller::mode_id::SIT:
            return controller::actions::update_sit(state, ctx, tick_ms);

        case controller::mode_id::JUMP:
            return controller::actions::update_jump(state, ctx, tick_ms);

        case controller::mode_id::KICK_PLACE:
            if(ctx.input.request == controller::action_request::KICK_EXIT)
            {
                controller::balance_request cmd = controller::actions::kick_base_command(ctx);
                controller::actions::begin_kick_exit(state);
                return cmd;
            }
            if(state.phase != controller::actions::EXIT_PREPARE &&
               ctx.input.request == controller::action_request::KICK_RUN)
            {
                controller::balance_request cmd = controller::actions::kick_base_command(ctx);
                controller::actions::begin_mode(state, controller::mode_id::KICK_RUN);
                return cmd;
            }
            return controller::actions::update_kick_place(state, ctx, tick_ms);

        case controller::mode_id::KICK_RUN:
            if(ctx.input.request == controller::action_request::KICK_EXIT)
            {
                controller::balance_request cmd = controller::actions::kick_base_command(ctx);
                controller::actions::begin_kick_exit(state);
                return cmd;
            }
            if(state.phase != controller::actions::EXIT_PREPARE &&
               ctx.input.request == controller::action_request::KICK_PLACE)
            {
                controller::balance_request cmd = controller::actions::kick_base_command(ctx);
                controller::actions::begin_mode(state, controller::mode_id::KICK_PLACE);
                return cmd;
            }
            return controller::actions::update_kick_run(state, ctx, tick_ms);

        case controller::mode_id::MIDDLE_CALIBRATION:
            return controller::actions::update_middle_calibration(state, ctx, tick_ms);

        case controller::mode_id::STOP:
            if(ctx.input.request == controller::action_request::BOOT)
            {
                controller::actions::begin_mode(state, controller::mode_id::BOOT);
            }
            return controller::balance_request{};

        default:
            return controller::balance_request{};
    }
}
