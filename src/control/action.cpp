#include "action.h"

#include "action_internal.h"
#include "hw/servo.h"

namespace action = control::action;
namespace action_internal = control::action::internal;

using action::action_runtime;
using action::context;
using action::jump_direction;
using action::mode;
using action::phase;
using action_internal::step_result;
using action_internal::transition;
using control::action_request;
using control::balance_mode;

/* ---- BOOT、BALANCE、STOP ---- */

/**
 * @brief 执行 BOOT 状态机的一步
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 * @param tick_ms 周期，单位毫秒
 *
 * @return 本周期动作结果
 */
step_result control::action::internal::step_boot(
    action::state &state, context &ctx, uint32_t tick_ms)
{
    step_result result;
    action_runtime &runtime = state.boot;
    switch(runtime.current_phase)
    {
        case phase::PREPARE:
            action_internal::set_torque(0);
            runtime.current_phase = phase::WAIT_SIGNAL;
            break;

        case phase::WAIT_SIGNAL:
            if(ctx.input.action == action_request::BOOT_CONFIRM &&
               ctx.battery_valid && !ctx.battery_low)
            {
                runtime.current_phase = phase::INIT;
            }
            break;

        case phase::INIT:
            action_internal::set_pose(
                hw::servo::LEG_LEFT_MIN, hw::servo::LEG_RIGHT_MIN, 450, 250);
            action_internal::reset_leg(ctx.leg);
            runtime.current_phase = phase::INIT_PREPARE;
            break;

        case phase::INIT_PREPARE:
            runtime.timer += tick_ms;
            if(runtime.timer >= 350)
            {
                runtime.timer = 0;
                runtime.elapsed = 0;
                runtime.ready_timer = 0;
                runtime.current_phase = phase::INIT_RECOVER;
            }
            break;

        case phase::INIT_RECOVER:
            result.balance = action_internal::recover_command(runtime, ctx);
            if(action_internal::recover_ready(
                   runtime, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                result.next = transition::DONE;
            }
            break;

        default:
            break;
    }
    return result;
}

/**
 * @brief 执行 BALANCE 状态机的一步
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 * @param tick_ms 周期，单位毫秒
 *
 * @return 本周期动作结果
 */
step_result control::action::internal::step_balance(
    action::state &, context &ctx, uint32_t)
{
    step_result result;
    result.balance.mode = balance_mode::BALANCE;
    result.balance.steering = true;
    result.balance.linear_vel = ctx.input.linear;
    result.balance.yaw_rate = ctx.input.yaw;
    action_internal::run_leg_control(ctx);

    if(ctx.input.reset_leg){action_internal::reset_leg(ctx.leg);}
    switch(ctx.input.action)
    {
        case action_request::RESET_BALANCE:
            result.next = transition::RESET_BALANCE;
            break;
        case action_request::SIT:
            result.next = transition::SIT;
            break;
        case action_request::MIDDLE_CALIBRATION:
            result.next = transition::MIDDLE_CALIBRATION;
            break;
        case action_request::JUMP_IN_PLACE:
            result.next = transition::JUMP;
            result.jump = jump_direction::IN_PLACE;
            break;
        case action_request::JUMP_FORWARD:
            result.next = transition::JUMP;
            result.jump = jump_direction::FORWARD;
            break;
        case action_request::JUMP_BACKWARD:
            result.next = transition::JUMP;
            result.jump = jump_direction::BACKWARD;
            break;
        case action_request::JUMP_LEFT:
            result.next = transition::JUMP;
            result.jump = jump_direction::TURN_LEFT;
            break;
        case action_request::JUMP_RIGHT:
            result.next = transition::JUMP;
            result.jump = jump_direction::TURN_RIGHT;
            break;
        case action_request::KICK_PLACE:
            result.next = transition::KICK_PLACE;
            break;
        case action_request::KICK_RUN:
            result.next = transition::KICK_RUN;
            break;
        default:
            break;
    }
    return result;
}

/**
 * @brief 执行 STOP 状态机的一步
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 * @param tick_ms 周期，单位毫秒
 *
 * @return 本周期动作结果
 */
step_result control::action::internal::step_stop(
    action::state &, context &ctx, uint32_t)
{
    step_result result;
    if(ctx.input.action == action_request::BOOT)
    {
        result.next = transition::BOOT;
    }
    return result;
}

/* ---- 显式状态切换 ---- */

/**
 * @brief 按目标模式初始化对应运行状态
 *
 * @param state 动作状态
 * @param next_mode 目标模式
 * @param jump 跳跃方向
 */
void control::action::internal::enter_mode(
    action::state &state, mode next_mode, jump_direction jump)
{
    mode previous = state.current_mode;
    state.current_mode = next_mode;
    switch(next_mode)
    {
        case mode::BOOT:
            state.boot = action_runtime{};
            break;
        case mode::SIT:
            state.sit = action_runtime{};
            break;
        case mode::JUMP:
            state.jump = action_runtime{};
            action_internal::set_jump_direction(state.jump_data, jump);
            break;
        case mode::KICK_PLACE:
            state.kick_place = action_runtime{};
            state.kick_place_data = action::kick_runtime{};
            break;
        case mode::KICK_RUN:
            state.kick_run = action_runtime{};
            state.kick_run_data = action::kick_runtime{};
            break;
        case mode::MIDDLE_CALIBRATION:
            state.middle_calibration_success = false;
            if(previous != mode::SIT)
            {
                state.sit = action_runtime{};
            }
            else
            {
                state.sit.timer = 0;
                state.sit.ready_timer = 0;
                state.sit.elapsed = 0;
            }
            break;
        case mode::BALANCE:
        case mode::STOP:
        default:
            break;
    }
}

/**
 * @brief 根据电池状态更新坐下后的起身锁
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 */
void control::action::internal::update_sit_exit_lock(
    action::state &state, const context &ctx)
{
    if(ctx.battery_valid && !ctx.battery_low)
    {
        state.sit_exit_locked = false;
        return;
    }

    bool sit_mode = state.current_mode == mode::SIT ||
        state.current_mode == mode::MIDDLE_CALIBRATION;
    if(sit_mode && ctx.battery_valid && ctx.battery_low)
    {
        state.sit_exit_locked = true;
    }
}

/**
 * @brief 应用动作结果产生的显式模式切换
 *
 * @param state 动作状态
 * @param result 动作结果
 */
void control::action::internal::apply_transition(
    action::state &state, const step_result &result)
{
    switch(result.next)
    {
        case transition::DONE:
            if(state.current_mode == mode::BOOT ||
               state.current_mode == mode::JUMP ||
               state.current_mode == mode::KICK_PLACE ||
               state.current_mode == mode::KICK_RUN)
            {
                action_internal::enter_mode(state, mode::BALANCE);
            }
            else if((state.current_mode == mode::SIT ||
                     state.current_mode == mode::MIDDLE_CALIBRATION) &&
                    !state.sit_exit_locked)
            {
                action_internal::enter_mode(state, mode::BALANCE);
            }
            break;
        case transition::BOOT:
            if(state.current_mode == mode::STOP &&
               !state.sit_exit_locked)
            {
                action_internal::enter_mode(state, mode::BOOT);
            }
            break;
        case transition::RESET_BALANCE:
            if(state.current_mode == mode::BALANCE)
            {
                action_internal::enter_mode(state, mode::BALANCE);
            }
            break;
        case transition::SIT:
            if(state.current_mode == mode::BALANCE)
            {
                action_internal::enter_mode(state, mode::SIT);
            }
            break;
        case transition::MIDDLE_CALIBRATION:
            if(state.current_mode == mode::BALANCE ||
               state.current_mode == mode::SIT)
            {
                action_internal::enter_mode(
                    state, mode::MIDDLE_CALIBRATION);
            }
            break;
        case transition::JUMP:
            if(state.current_mode == mode::BALANCE)
            {
                action_internal::enter_mode(
                    state, mode::JUMP, result.jump);
            }
            break;
        case transition::KICK_PLACE:
            if(state.current_mode == mode::BALANCE ||
               state.current_mode == mode::KICK_RUN)
            {
                action_internal::enter_mode(state, mode::KICK_PLACE);
            }
            break;
        case transition::KICK_RUN:
            if(state.current_mode == mode::BALANCE ||
               state.current_mode == mode::KICK_PLACE)
            {
                action_internal::enter_mode(state, mode::KICK_RUN);
            }
            break;
        case transition::NONE:
        default:
            break;
    }
}

/* ---- action 公共 API ---- */

/**
 * @brief 初始化显式动作状态机
 *
 * @param state 动作状态
 */
void control::action::init(action::state &state)
{
    state = action::state{};
    state.current_mode = mode::BOOT;
}

/**
 * @brief 获取当前动作模式
 *
 * @param state 动作状态
 *
 * @return 当前动作模式
 */
mode control::action::current_mode(const action::state &state)
{
    return state.current_mode;
}

/**
 * @brief 执行一个动作状态机周期并生成统一平衡命令
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 * @param tick_ms 周期，单位毫秒
 *
 * @return 统一平衡命令
 */
control::balance_command control::action::step(
    action::state &state, context &ctx, uint32_t tick_ms)
{
    action_internal::update_sit_exit_lock(state, ctx);

    if(state.current_mode != mode::STOP &&
       ctx.input.action == action_request::STOP)
    {
        action_internal::enter_mode(state, mode::STOP);
        return control::balance_command{};
    }

    step_result result;
    switch(state.current_mode)
    {
        case mode::BOOT:
            result = action_internal::step_boot(state, ctx, tick_ms);
            break;
        case mode::BALANCE:
            result = action_internal::step_balance(state, ctx, tick_ms);
            break;
        case mode::SIT:
            result = action_internal::step_sit(state, ctx, tick_ms);
            break;
        case mode::JUMP:
            result = action_internal::step_jump(state, ctx, tick_ms);
            break;
        case mode::STOP:
            result = action_internal::step_stop(state, ctx, tick_ms);
            break;
        case mode::KICK_PLACE:
            result = action_internal::step_kick(state, ctx, tick_ms, false);
            break;
        case mode::KICK_RUN:
            result = action_internal::step_kick(state, ctx, tick_ms, true);
            break;
        case mode::MIDDLE_CALIBRATION:
            result = action_internal::step_middle_calibration(
                state, ctx, tick_ms);
            break;
        default:
            action_internal::enter_mode(state, mode::STOP);
            break;
    }

    action_internal::apply_transition(state, result);
    return result.balance;
}
