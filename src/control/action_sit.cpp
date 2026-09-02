#include "action_internal.h"

#include "hw/servo.h"
#include <math.h>

/* ---- SIT 与中位校准参数 ---- */

static constexpr int16_t SERVO_MIDDLE_COUNT = 2048;
static constexpr int16_t SIT_MIDDLE_READY_ERROR = 50;
static constexpr uint32_t MIDDLE_CALIBRATION_TORQUE_OFF_MS = 500;
static constexpr uint32_t MIDDLE_CALIBRATION_START_MS = 2000;
static constexpr uint32_t MIDDLE_CALIBRATION_LEFT_OFF_DELAY_MS = 100;
static constexpr uint32_t MIDDLE_CALIBRATION_RIGHT_OFF_DELAY_MS = 1000;
static constexpr uint32_t MIDDLE_CALIBRATION_PREPARE_WAIT_MS =
    MIDDLE_CALIBRATION_START_MS - MIDDLE_CALIBRATION_TORQUE_OFF_MS;

/* ---- SIT 与中位校准内部流程 ---- */

/**
 * @brief 判断左右腿舵机当前位置是否接近中位
 *
 * @param ctx 动作上下文
 *
 * @return 左右腿舵机都接近中位时返回 true
 */
static bool servo_middle_ready(const control::action::context &ctx)
{
    int16_t left_error = abs(ctx.servo_left_position - SERVO_MIDDLE_COUNT);
    int16_t right_error = abs(ctx.servo_right_position - SERVO_MIDDLE_COUNT);
    return left_error <= SIT_MIDDLE_READY_ERROR && right_error <= SIT_MIDDLE_READY_ERROR;
}

/**
 * @brief 生成坐下直出电机命令
 *
 * @param command 平衡命令
 */
static void set_sit_direct_output(control::balance_command &command)
{
    command.mode = control::balance_mode::DIRECT;
    command.direct_left = -0.05f;
    command.direct_right = -0.05f;
}

/**
 * @brief 判断当前坐下阶段是否允许进入中位校准
 *
 * @param current_phase 当前阶段
 *
 * @return 允许进入时返回 true
 */
static bool sit_phase_can_enter_middle_calibration(
    control::action::phase current_phase)
{
    return current_phase == control::action::phase::PREPARE ||
           current_phase == control::action::phase::INIT_PREPARE ||
           current_phase == control::action::phase::MOVING ||
           current_phase == control::action::phase::DONE;
}

/**
 * @brief 判断当前阶段是否属于中位校准流程
 *
 * @param current_phase 当前动作阶段
 *
 * @return 属于中位校准阶段时返回 true
 */
static bool is_middle_calibration_phase(control::action::phase current_phase)
{
    return current_phase == control::action::phase::CALIBRATION_WAIT ||
           current_phase == control::action::phase::CALIBRATION_RIGHT_OFF ||
           current_phase == control::action::phase::CALIBRATION_APPLY ||
           current_phase == control::action::phase::CALIBRATION_DONE;
}

/**
 * @brief 更新坐下类流程状态机
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 * @param tick_ms 周期，单位毫秒
 * @param calibration 是否执行中位校准
 *
 * @return 本周期动作结果
 */
static control::action::internal::step_result step_sit_flow(
    control::action::state &state, control::action::context &ctx,
    uint32_t tick_ms, bool calibration)
{
    control::action::internal::step_result result;
    control::action::action_runtime &runtime = state.sit;

    bool exit_ready = runtime.current_phase == control::action::phase::DONE ||
        (calibration && is_middle_calibration_phase(runtime.current_phase));
    if(ctx.input.action == control::action_request::EXIT &&
       !state.sit_exit_locked && exit_ready)
    {
        control::action::internal::set_pose(
            hw::servo::LEG_LEFT_MIN, hw::servo::LEG_RIGHT_MIN, 450, 250);
        control::action::internal::reset_leg(ctx.leg);
        runtime.current_phase = control::action::phase::EXIT_PREPARE;
        return result;
    }

    switch(runtime.current_phase)
    {
        case control::action::phase::PREPARE:
            result.balance.mode = control::balance_mode::BALANCE;
            result.balance.steering = true;
            control::action::internal::set_pose(
                hw::servo::LEG_LEFT_MIN, hw::servo::LEG_RIGHT_MIN, 450, 250);
            runtime.timer = 0;
            runtime.current_phase = control::action::phase::INIT_PREPARE;
            break;

        case control::action::phase::INIT_PREPARE:
            result.balance.mode = control::balance_mode::BALANCE;
            result.balance.steering = true;
            if(servo_middle_ready(ctx))
            {
                runtime.timer = 0;
                control::action::internal::set_torque(2);
                set_sit_direct_output(result.balance);
                runtime.current_phase = control::action::phase::MOVING;
            }
            break;

        case control::action::phase::MOVING:
            runtime.timer += tick_ms;
            if(fabsf(ctx.status.pitch_angle) >= 0.25f)
            {
                runtime.timer = 0;
                result.balance.mode = control::balance_mode::OFF;
                runtime.current_phase = control::action::phase::DONE;
                break;
            }
            set_sit_direct_output(result.balance);
            break;

        case control::action::phase::DONE:
            if(calibration)
            {
                runtime.timer += tick_ms;
                if(runtime.timer >= MIDDLE_CALIBRATION_TORQUE_OFF_MS)
                {
                    control::action::internal::set_torque(0);
                    runtime.timer = 0;
                    runtime.current_phase = control::action::phase::CALIBRATION_WAIT;
                }
            }
            else if((runtime.timer += tick_ms) >= 10000 ||
                    ctx.input.disable_leg_torque)
            {
                control::action::internal::set_torque(0);
                runtime.timer = 10000;
            }
            break;

        case control::action::phase::CALIBRATION_WAIT:
            if(!calibration){break;}
            runtime.timer += tick_ms;
            if(runtime.timer >= MIDDLE_CALIBRATION_PREPARE_WAIT_MS)
            {
                hw::servo::set_torque(hw::servo::LEG_LEFT, 0);
                runtime.timer = 0;
                runtime.current_phase = control::action::phase::CALIBRATION_RIGHT_OFF;
            }
            break;

        case control::action::phase::CALIBRATION_RIGHT_OFF:
            if(!calibration){break;}
            runtime.timer += tick_ms;
            if(runtime.timer >= MIDDLE_CALIBRATION_LEFT_OFF_DELAY_MS)
            {
                hw::servo::set_torque(hw::servo::LEG_RIGHT, 0);
                runtime.timer = 0;
                runtime.current_phase = control::action::phase::CALIBRATION_APPLY;
            }
            break;

        case control::action::phase::CALIBRATION_APPLY:
            if(!calibration){break;}
            runtime.timer += tick_ms;
            if(runtime.timer >= MIDDLE_CALIBRATION_RIGHT_OFF_DELAY_MS)
            {
                control::action::internal::set_torque(128);
                runtime.timer = 0;
                runtime.current_phase = control::action::phase::CALIBRATION_DONE;
            }
            break;

        case control::action::phase::CALIBRATION_DONE:
            if(calibration){state.middle_calibration_success = true;}
            break;

        case control::action::phase::EXIT_PREPARE:
            runtime.timer += tick_ms;
            if(runtime.timer >= 350)
            {
                runtime.timer = 0;
                runtime.elapsed = 0;
                runtime.ready_timer = 0;
                runtime.current_phase = control::action::phase::EXIT_RECOVER;
            }
            break;

        case control::action::phase::EXIT_RECOVER:
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
 * @brief 执行 SIT 模式的一步
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 * @param tick_ms 周期，单位毫秒
 *
 * @return 本周期动作结果
 */
control::action::internal::step_result control::action::internal::step_sit(
    control::action::state &state, control::action::context &ctx,
    uint32_t tick_ms)
{
    control::action::internal::step_result result =
        step_sit_flow(state, ctx, tick_ms, false);
    if(ctx.input.action == control::action_request::MIDDLE_CALIBRATION &&
       sit_phase_can_enter_middle_calibration(state.sit.current_phase))
    {
        result.next = control::action::internal::transition::MIDDLE_CALIBRATION;
    }
    return result;
}

/**
 * @brief 执行 MIDDLE_CALIBRATION 模式的一步
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 * @param tick_ms 周期，单位毫秒
 *
 * @return 本周期动作结果
 */
control::action::internal::step_result
control::action::internal::step_middle_calibration(
    control::action::state &state, control::action::context &ctx,
    uint32_t tick_ms)
{
    return step_sit_flow(state, ctx, tick_ms, true);
}
