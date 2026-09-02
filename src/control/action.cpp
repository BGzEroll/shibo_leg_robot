#include "action.h"

#include "action_internal.h"
#include "hw/servo.h"

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
control::action::internal::step_result control::action::internal::step_boot(
    control::action::state &state, control::action::context &ctx, uint32_t tick_ms)
{
    control::action::internal::step_result result;
    control::action::action_runtime &runtime = state.boot;
    switch(runtime.current_phase)
    {
        case control::action::phase::PREPARE:
            control::action::internal::set_torque(0);
            runtime.current_phase = control::action::phase::WAIT_SIGNAL;
            break;

        case control::action::phase::WAIT_SIGNAL:
            if(ctx.input.action == control::action_request::BOOT_CONFIRM &&
               ctx.battery_valid && !ctx.battery_low)
            {
                runtime.current_phase = control::action::phase::INIT;
            }
            break;

        case control::action::phase::INIT:
            control::action::internal::set_pose(
                hw::servo::LEG_LEFT_MIN, hw::servo::LEG_RIGHT_MIN, 450, 250);
            control::action::internal::reset_leg(ctx.leg);
            runtime.current_phase = control::action::phase::INIT_PREPARE;
            break;

        case control::action::phase::INIT_PREPARE:
            runtime.timer += tick_ms;
            if(runtime.timer >= 350)
            {
                runtime.timer = 0;
                runtime.elapsed = 0;
                runtime.ready_timer = 0;
                runtime.current_phase = control::action::phase::INIT_RECOVER;
            }
            break;

        case control::action::phase::INIT_RECOVER:
            result.balance = control::action::internal::recover_command(runtime, ctx);
            if(control::action::internal::recover_ready(
                   runtime, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                result.next = control::action::internal::transition::DONE;
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
control::action::internal::step_result control::action::internal::step_balance(
    control::action::state &, control::action::context &ctx, uint32_t)
{
    control::action::internal::step_result result;
    result.balance.mode = control::balance_mode::BALANCE;
    result.balance.steering = true;
    result.balance.linear_vel = ctx.input.linear;
    result.balance.yaw_rate = ctx.input.yaw;
    control::action::internal::run_leg_control(ctx);

    if(ctx.input.reset_leg){control::action::internal::reset_leg(ctx.leg);}
    switch(ctx.input.action)
    {
        case control::action_request::RESET_BALANCE:
            result.next = control::action::internal::transition::RESET_BALANCE;
            break;
        case control::action_request::SIT:
            result.next = control::action::internal::transition::SIT;
            break;
        case control::action_request::MIDDLE_CALIBRATION:
            result.next = control::action::internal::transition::MIDDLE_CALIBRATION;
            break;
        case control::action_request::JUMP_IN_PLACE:
            result.next = control::action::internal::transition::JUMP;
            result.jump = control::action::jump_direction::IN_PLACE;
            break;
        case control::action_request::JUMP_FORWARD:
            result.next = control::action::internal::transition::JUMP;
            result.jump = control::action::jump_direction::FORWARD;
            break;
        case control::action_request::JUMP_BACKWARD:
            result.next = control::action::internal::transition::JUMP;
            result.jump = control::action::jump_direction::BACKWARD;
            break;
        case control::action_request::JUMP_LEFT:
            result.next = control::action::internal::transition::JUMP;
            result.jump = control::action::jump_direction::TURN_LEFT;
            break;
        case control::action_request::JUMP_RIGHT:
            result.next = control::action::internal::transition::JUMP;
            result.jump = control::action::jump_direction::TURN_RIGHT;
            break;
        case control::action_request::KICK_PLACE:
            result.next = control::action::internal::transition::KICK_PLACE;
            break;
        case control::action_request::KICK_RUN:
            result.next = control::action::internal::transition::KICK_RUN;
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
control::action::internal::step_result control::action::internal::step_stop(
    control::action::state &, control::action::context &ctx, uint32_t)
{
    control::action::internal::step_result result;
    if(ctx.input.action == control::action_request::BOOT)
    {
        result.next = control::action::internal::transition::BOOT;
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
    control::action::state &state, control::action::mode next_mode,
    control::action::jump_direction jump)
{
    control::action::mode previous = state.current_mode;
    state.current_mode = next_mode;
    switch(next_mode)
    {
        case control::action::mode::BOOT:
            state.boot = control::action::action_runtime{};
            break;
        case control::action::mode::SIT:
            state.sit = control::action::action_runtime{};
            break;
        case control::action::mode::JUMP:
            state.jump = control::action::action_runtime{};
            control::action::internal::set_jump_direction(state.jump_data, jump);
            break;
        case control::action::mode::KICK_PLACE:
            state.kick_place = control::action::action_runtime{};
            state.kick_place_data = control::action::kick_runtime{};
            break;
        case control::action::mode::KICK_RUN:
            state.kick_run = control::action::action_runtime{};
            state.kick_run_data = control::action::kick_runtime{};
            break;
        case control::action::mode::MIDDLE_CALIBRATION:
            state.middle_calibration_success = false;
            if(previous != control::action::mode::SIT)
            {
                state.sit = control::action::action_runtime{};
            }
            else
            {
                state.sit.timer = 0;
                state.sit.ready_timer = 0;
                state.sit.elapsed = 0;
            }
            break;
        case control::action::mode::BALANCE:
        case control::action::mode::STOP:
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
    control::action::state &state, const control::action::context &ctx)
{
    if(ctx.battery_valid && !ctx.battery_low)
    {
        state.sit_exit_locked = false;
        return;
    }

    bool sit_mode = state.current_mode == control::action::mode::SIT ||
        state.current_mode == control::action::mode::MIDDLE_CALIBRATION;
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
    control::action::state &state, const control::action::internal::step_result &result)
{
    switch(result.next)
    {
        case control::action::internal::transition::DONE:
            if(state.current_mode == control::action::mode::BOOT ||
               state.current_mode == control::action::mode::JUMP ||
               state.current_mode == control::action::mode::KICK_PLACE ||
               state.current_mode == control::action::mode::KICK_RUN)
            {
                control::action::internal::enter_mode(
                    state, control::action::mode::BALANCE);
            }
            else if((state.current_mode == control::action::mode::SIT ||
                     state.current_mode == control::action::mode::MIDDLE_CALIBRATION) &&
                    !state.sit_exit_locked)
            {
                control::action::internal::enter_mode(
                    state, control::action::mode::BALANCE);
            }
            break;
        case control::action::internal::transition::BOOT:
            if(state.current_mode == control::action::mode::STOP &&
               !state.sit_exit_locked)
            {
                control::action::internal::enter_mode(
                    state, control::action::mode::BOOT);
            }
            break;
        case control::action::internal::transition::RESET_BALANCE:
            if(state.current_mode == control::action::mode::BALANCE)
            {
                control::action::internal::enter_mode(
                    state, control::action::mode::BALANCE);
            }
            break;
        case control::action::internal::transition::SIT:
            if(state.current_mode == control::action::mode::BALANCE)
            {
                control::action::internal::enter_mode(
                    state, control::action::mode::SIT);
            }
            break;
        case control::action::internal::transition::MIDDLE_CALIBRATION:
            if(state.current_mode == control::action::mode::BALANCE ||
               state.current_mode == control::action::mode::SIT)
            {
                control::action::internal::enter_mode(
                    state, control::action::mode::MIDDLE_CALIBRATION);
            }
            break;
        case control::action::internal::transition::JUMP:
            if(state.current_mode == control::action::mode::BALANCE)
            {
                control::action::internal::enter_mode(
                    state, control::action::mode::JUMP, result.jump);
            }
            break;
        case control::action::internal::transition::KICK_PLACE:
            if(state.current_mode == control::action::mode::BALANCE ||
               state.current_mode == control::action::mode::KICK_RUN)
            {
                control::action::internal::enter_mode(
                    state, control::action::mode::KICK_PLACE);
            }
            break;
        case control::action::internal::transition::KICK_RUN:
            if(state.current_mode == control::action::mode::BALANCE ||
               state.current_mode == control::action::mode::KICK_PLACE)
            {
                control::action::internal::enter_mode(
                    state, control::action::mode::KICK_RUN);
            }
            break;
        case control::action::internal::transition::NONE:
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
void control::action::init(control::action::state &state)
{
    state = control::action::state{};
    state.current_mode = control::action::mode::BOOT;
}

/**
 * @brief 获取当前动作模式
 *
 * @param state 动作状态
 *
 * @return 当前动作模式
 */
control::action::mode control::action::current_mode(
    const control::action::state &state)
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
    control::action::state &state, control::action::context &ctx, uint32_t tick_ms)
{
    control::action::internal::update_sit_exit_lock(state, ctx);

    if(state.current_mode != control::action::mode::STOP &&
       ctx.input.action == control::action_request::STOP)
    {
        control::action::internal::enter_mode(state, control::action::mode::STOP);
        return control::balance_command{};
    }

    control::action::internal::step_result result;
    switch(state.current_mode)
    {
        case control::action::mode::BOOT:
            result = control::action::internal::step_boot(state, ctx, tick_ms);
            break;
        case control::action::mode::BALANCE:
            result = control::action::internal::step_balance(state, ctx, tick_ms);
            break;
        case control::action::mode::SIT:
            result = control::action::internal::step_sit(state, ctx, tick_ms);
            break;
        case control::action::mode::JUMP:
            result = control::action::internal::step_jump(state, ctx, tick_ms);
            break;
        case control::action::mode::STOP:
            result = control::action::internal::step_stop(state, ctx, tick_ms);
            break;
        case control::action::mode::KICK_PLACE:
            result = control::action::internal::step_kick(state, ctx, tick_ms, false);
            break;
        case control::action::mode::KICK_RUN:
            result = control::action::internal::step_kick(state, ctx, tick_ms, true);
            break;
        case control::action::mode::MIDDLE_CALIBRATION:
            result = control::action::internal::step_middle_calibration(
                state, ctx, tick_ms);
            break;
        default:
            control::action::internal::enter_mode(
                state, control::action::mode::STOP);
            break;
    }

    control::action::internal::apply_transition(state, result);
    return result.balance;
}
