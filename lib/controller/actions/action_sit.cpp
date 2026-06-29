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
    cmd.target.direct_left = -0.15f;
    cmd.target.direct_right = -0.15f;
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
    balance_request cmd;
    switch(state.phase)
    {
        case action::PREPARE:
            cmd.command.enable_balance = true;
            cmd.command.enable_motor = true;
            cmd.command.enable_steering = true;
            set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
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
                set_torque(2);
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
            if((state.timer += tick_ms) >= 10000 || (ctx.input.buttons & BTN_LS))
            {
                set_torque(0);
                state.timer = 10000;
            }
            if(ctx.input.buttons & BTN_RB)
            {
                set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
                reset_leg(ctx.leg);
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
            cmd = recover_command(state, ctx);
            if(recover_ready(state, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                begin_mode(state, mode_id::BALANCE);
            }
            break;
    }

    return cmd;
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
    balance_request cmd;
    switch(state.phase)
    {
        case action::PREPARE:
            cmd.command.enable_balance = true;
            cmd.command.enable_motor = true;
            cmd.command.enable_steering = true;
            set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
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
                set_torque(2);
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
            state.timer += tick_ms;
            if(state.timer >= MIDDLE_CALIBRATION_TORQUE_OFF_MS && state.ready_timer == 0)
            {
                set_torque(0);
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
            if(ctx.input.buttons & BTN_RB)
            {
                set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
                reset_leg(ctx.leg);
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
            cmd = recover_command(state, ctx);
            if(recover_ready(state, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                begin_mode(state, mode_id::BALANCE);
            }
            break;
    }

    return cmd;
}
