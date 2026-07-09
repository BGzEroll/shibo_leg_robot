#include "action_sit.h"

#include "controller.h"
#include "sts3032.h"

static constexpr int16_t SERVO_MIDDLE_COUNT = 2048;
static constexpr int16_t SIT_MIDDLE_READY_ERROR = 50;
static constexpr uint32_t MIDDLE_CALIBRATION_TORQUE_OFF_MS = 500;
static constexpr uint32_t MIDDLE_CALIBRATION_RUN_MS = 2000;
static constexpr uint32_t MIDDLE_CALIBRATION_SUCCESS_MS = 2500;

/* ---- 坐下与校准内部流程 ---- */

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
    cmd.mode = controller::balance_drive_mode::DIRECT_OUTPUT;
    cmd.direct_left = -0.05f;
    cmd.direct_right = -0.05f;
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
    return phase == controller::actions::PREPARE ||
           phase == controller::actions::INIT_PREPARE ||
           phase == controller::actions::MOVING ||
           phase == controller::actions::DONE;
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
        case controller::actions::PREPARE:
            cmd.mode = controller::balance_drive_mode::BALANCE;
            cmd.enable_steering = true;
            controller::actions::set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
            state.timer = 0;
            state.phase = controller::actions::INIT_PREPARE;
            break;

        case controller::actions::INIT_PREPARE:
            cmd.mode = controller::balance_drive_mode::BALANCE;
            cmd.enable_steering = true;
            if(servo_middle_ready())
            {
                state.timer = 0;
                controller::actions::set_torque(2);
                set_sit_direct_output(cmd);
                state.phase = controller::actions::MOVING;
            }
            break;

        case controller::actions::MOVING:
            state.timer += tick_ms;
            if(fabsf(ctx.status.pitch_angle) >= 0.25f)
            {
                state.timer = 0;
                cmd.mode = controller::balance_drive_mode::STOP;
                state.phase = controller::actions::DONE;
                break;
            }
            set_sit_direct_output(cmd);
            break;

        case controller::actions::DONE:
            if(calibration)
            {
                state.timer += tick_ms;
                if(state.timer >= MIDDLE_CALIBRATION_TORQUE_OFF_MS && state.ready_timer == 0)
                {
                    controller::actions::set_torque(0);
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
            else if((state.timer += tick_ms) >= 10000 || ctx.input.disable_leg_torque)
            {
                controller::actions::set_torque(0);
                state.timer = 10000;
            }
            if(ctx.input.exit_action)
            {
                controller::actions::set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
                controller::actions::reset_leg(ctx.leg);
                state.phase = controller::actions::EXIT_PREPARE;
            }
            break;

        case controller::actions::EXIT_PREPARE:
            if((state.timer += tick_ms) >= 350)
            {
                state.timer = 0;
                state.elapsed = 0;
                state.ready_timer = 0;
                state.phase = controller::actions::EXIT_RECOVER;
            }
            break;

        case controller::actions::EXIT_RECOVER:
            cmd = controller::actions::recover_command(state, ctx);
            if(controller::actions::recover_ready(state, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                controller::actions::begin_mode(state, controller::mode_id::BALANCE);
            }
            break;
    }

    return cmd;
}

/* ---- 坐下与校准动作 API ---- */

/**
 * @brief 更新 SIT 模式状态机并生成平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 * @param tick_ms 本次更新周期，单位毫秒
 *
 * @return 生成的平衡请求
 */
controller::balance_request controller::actions::update_sit(controller::action_state &state, controller::action_io &ctx, uint32_t tick_ms)
{
    if(ctx.input.middle_calibration_request && sit_phase_can_enter_middle_calibration(state.phase))
    {
        enter_middle_calibration_from_sit(state);
    }
    return update_sit_flow(state, ctx, tick_ms, state.mode == controller::mode_id::MIDDLE_CALIBRATION);
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
controller::balance_request controller::actions::update_middle_calibration(controller::action_state &state, controller::action_io &ctx, uint32_t tick_ms)
{
    return update_sit_flow(state, ctx, tick_ms, true);
}
