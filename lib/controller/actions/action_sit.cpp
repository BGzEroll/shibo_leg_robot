#include "action_sit.h"

#include "controller.h"
#include "sts3032.h"
#include "xbox.h"

namespace action = controller::actions;

static constexpr int16_t SERVO_MIDDLE_COUNT = 2048;
static constexpr int16_t SIT_MIDDLE_READY_ERROR = 50;
static constexpr uint32_t MIDDLE_CALIBRATION_TORQUE_OFF_MS = 500;
static constexpr uint32_t MIDDLE_CALIBRATION_RUN_MS = 2000;
static constexpr uint32_t MIDDLE_CALIBRATION_SUCCESS_MS = 2500;

/**
 * @brief 判断左右腿舵机当前位置是否已经接近中位
 *
 * @return 左右腿舵机都接近中位时返回 true
 */
static bool servo_middle_ready()
{
    sts3032::get_position_and_load();
    int16_t left_error = abs(sts3032::status[0].position - SERVO_MIDDLE_COUNT);
    int16_t right_error = abs(sts3032::status[1].position - SERVO_MIDDLE_COUNT);
    return left_error <= SIT_MIDDLE_READY_ERROR && right_error <= SIT_MIDDLE_READY_ERROR;
}

/**
 * @brief 生成坐下直出电机请求
 *
 * @param cmd 平衡请求
 */
static void set_sit_direct_output(controller::balance_request &cmd)
{
    cmd.command.enable_balance = false;
    cmd.command.enable_motor = true;
    cmd.command.enable_steering = false;
    cmd.command.direct_output = true;
    cmd.target.direct_left = -0.05f;
    cmd.target.direct_right = -0.05f;
}

/**
 * @brief 判断当前坐下阶段是否允许切换到中位校准
 *
 * @param phase 当前动作阶段
 *
 * @return 允许切换时返回 true
 */
static bool sit_phase_can_enter_middle_calibration(uint8_t phase)
{
    return phase == action::PREPARE ||
           phase == action::INIT_PREPARE ||
           phase == action::MOVING ||
           phase == action::DONE;
}

/**
 * @brief 将坐下流程切换为中位校准流程
 *
 * @param state 动作状态机状态
 */
static void enter_middle_calibration_from_sit(controller::action_state &state)
{
    state.mode = controller::mode_id::MIDDLE_CALIBRATION;
    state.timer = 0;
    state.ready_timer = 0;
    state.elapsed = 0;
}

/**
 * @brief 更新坐下类流程状态机并生成平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 * @param tick_ms 本次更新周期，单位毫秒
 * @param calibration 是否在坐下完成后执行中位校准
 *
 * @return 生成的平衡请求
 */
static controller::balance_request update_sit_flow(controller::action_state &state, controller::action_io &ctx,
    uint32_t tick_ms, bool calibration)
{
    controller::balance_request cmd;
    switch(state.phase)
    {
        case action::PREPARE:
            cmd.command.enable_balance = true;
            cmd.command.enable_motor = true;
            cmd.command.enable_steering = true;
            action::set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
            state.timer = 0;
            state.phase = action::INIT_PREPARE;
            break;

        case action::INIT_PREPARE:
            cmd.command.enable_balance = true;
            cmd.command.enable_motor = true;
            cmd.command.enable_steering = true;
            if(servo_middle_ready())
            {
                state.timer = 0;
                action::set_torque(2);
                set_sit_direct_output(cmd);
                state.phase = action::MOVING;
            }
            break;

        case action::MOVING:
            state.timer += tick_ms;
            if(fabsf(ctx.status.pitch_angle) >= 0.25f)
            {
                state.timer = 0;
                cmd.command.enable_motor = false;
                state.phase = action::DONE;
                break;
            }
            set_sit_direct_output(cmd);
            break;

        case action::DONE:
            if(calibration)
            {
                state.timer += tick_ms;
                if(state.timer >= MIDDLE_CALIBRATION_TORQUE_OFF_MS && state.ready_timer == 0)
                {
                    action::set_torque(0);
                    state.ready_timer = 1;
                }
                if(state.timer >= MIDDLE_CALIBRATION_RUN_MS && state.elapsed == 0)
                {
                    sts3032::calibrate_middle();
                    state.elapsed = 1;
                }
                if(state.timer >= MIDDLE_CALIBRATION_SUCCESS_MS && state.elapsed == 1)
                {
                    controller::mark_middle_calibration_success();
                    state.elapsed = 2;
                }
            }
            else if((state.timer += tick_ms) >= 10000 || (ctx.input.buttons & BTN_LS))
            {
                action::set_torque(0);
                state.timer = 10000;
            }
            if(ctx.input.buttons & BTN_RB)
            {
                action::set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
                action::reset_leg(ctx.leg);
                state.phase = action::EXIT_PREPARE;
            }
            break;

        case action::EXIT_PREPARE:
            if((state.timer += tick_ms) >= 350)
            {
                state.timer = 0;
                state.elapsed = 0;
                state.ready_timer = 0;
                state.phase = action::EXIT_RECOVER;
            }
            break;

        case action::EXIT_RECOVER:
            cmd = action::recover_command(state, ctx);
            if(action::recover_ready(state, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                action::begin_mode(state, controller::mode_id::BALANCE);
            }
            break;
    }

    return cmd;
}

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
    if(ctx.input.middle_calibration_request && sit_phase_can_enter_middle_calibration(state.phase))
    {
        enter_middle_calibration_from_sit(state);
    }
    return update_sit_flow(state, ctx, tick_ms, state.mode == mode_id::MIDDLE_CALIBRATION);
}

/**
 * @brief 更新舵机中位校准流程并生成平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 * @param tick_ms 本次更新周期，单位毫秒
 *
 * @return 生成的平衡请求
 */
controller::balance_request controller::actions::update_middle_calibration(action_state &state, action_io &ctx, uint32_t tick_ms)
{
    return update_sit_flow(state, ctx, tick_ms, true);
}
