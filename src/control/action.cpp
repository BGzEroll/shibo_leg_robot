#include "action.h"

#include "hw/servo.h"
#include <math.h>

/* ---- 动作参数与内部结果 ---- */

static constexpr float LEG_HEIGHT_BASE_MIN = -10.0f;
static constexpr float LEG_HEIGHT_BASE_MAX = 52.0f;
static constexpr int16_t SERVO_MIDDLE_COUNT = 2048;
static constexpr int16_t SIT_MIDDLE_READY_ERROR = 50;
static constexpr uint32_t MIDDLE_CALIBRATION_TORQUE_OFF_MS = 500;
static constexpr uint32_t MIDDLE_CALIBRATION_RUN_MS = 2000;
static constexpr uint32_t MIDDLE_CALIBRATION_SUCCESS_MS = 2500;

static constexpr float CAM_INITIAL_ANGLE = 45.0f;
static constexpr float CAM_LOST_ANGLE = 45.0f;
static constexpr float CAM_PD_P = 0.07f;
static constexpr float CAM_PD_D = 0.05f;
static constexpr float CAM_PD_STEP_LIMIT = 10.0f;
static constexpr int32_t CAM_AIM_DEADBAND = 10;
static constexpr float YAW_AIM_P = 0.7f;
static constexpr float YAW_ALIGN_KP = 1.2f;
static constexpr float YAW_RATE_LIMIT = 0.9f;
static constexpr float YAW_ALIGN_LIMIT = 10.0f * PI / 180.0f;
static constexpr int32_t YAW_AIM_DEADBAND = 40;
static constexpr float JOY_RATE_LIMIT = 100.0f;
static constexpr float CHASE_LINEAR_KP = 0.002f;
static constexpr float RUN_FORWARD_MAX = 0.25f;
static constexpr float RUN_BACK_VEL = -0.12f;
static constexpr int16_t PLACE_BALL_S2 = 20;
static constexpr int16_t PLACE_KICK_DY = -10;
static constexpr int16_t CHASE_BALL_S2 = 10;
static constexpr int16_t RUN_KICK_DY = -5;
static constexpr int16_t KICK_DY_MAX = 120;
static constexpr int16_t OB_BALL_DY = 120;
static constexpr uint16_t FRONTIER_READY_ANGLE = 100;
static constexpr uint16_t FRONTIER_KICK_ANGLE = 0;
static constexpr uint32_t KICK_HOLD_MS = 0;
static constexpr uint32_t KICK_COOLDOWN_MS = 2000;
static constexpr uint32_t KICK_EXIT_DELAY_MS = 500;
static constexpr uint32_t RUN_AFTER_KICK_MS = 700;
static constexpr float KICK_LEG_HEIGHT_COUNT_OFFSET = 50.0f;

enum class transition : uint8_t
{
    NONE = 0,
    DONE,
    BOOT,
    KICK_PLACE,
    KICK_RUN,
    SIT,
    MIDDLE_CALIBRATION,
    JUMP,
    RESET_BALANCE
};

struct step_result
{
    control::balance_command balance;
    transition next = transition::NONE;
    control::action::jump_direction jump = control::action::jump_direction::IN_PLACE;
};

/* ---- 基础动作工具 ---- */

/**
 * @brief 将角度归一化到 -PI 到 PI 范围内
 *
 * @param angle 角度值
 *
 * @return 归一化后的角度
 */
static float wrap_pi(float angle)
{
    while(angle > PI){angle -= 2.0f * PI;}
    while(angle < -PI){angle += 2.0f * PI;}
    return angle;
}

/**
 * @brief 计算目标角度和当前角度之间的最短误差
 *
 * @param target 目标角度
 * @param current 当前角度
 *
 * @return 最短角度误差
 */
static float angle_error(float target, float current)
{
    return wrap_pi(target - current);
}

/**
 * @brief 设置左右腿舵机目标姿态并触发同步移动
 *
 * @param left 左侧目标位置
 * @param right 右侧目标位置
 * @param speed 舵机速度
 * @param acceleration 舵机加速度
 */
static void set_pose(int16_t left, int16_t right, uint16_t speed, uint8_t acceleration)
{
    hw::servo::set(hw::servo::LEG_LEFT, left, speed, acceleration);
    hw::servo::set(hw::servo::LEG_RIGHT, right, speed, acceleration);
    hw::servo::move();
}

/**
 * @brief 设置左右腿舵机扭矩模式
 *
 * @param type 扭矩模式类型
 */
static void set_torque(uint8_t type)
{
    hw::servo::set_torque(hw::servo::LEG_LEFT, type);
    hw::servo::set_torque(hw::servo::LEG_RIGHT, type);
}

/**
 * @brief 复位腿部运行状态和横滚 PID
 *
 * @param leg 腿部运行状态
 */
static void reset_leg(control::action::leg_runtime &leg)
{
    leg.roll_adjust = 0.0f;
    leg.height_base = (float)control::action::LEG_HEIGHT_BASE;
    leg.reset_roll_pid();
}

/**
 * @brief 根据输入和横滚姿态更新腿部舵机控制
 *
 * @param ctx 动作输入输出上下文
 * @param height_count_offset 腿高目标的舵机计数偏移量
 */
static void run_leg_control(control::action::context &ctx, float height_count_offset = 0.0f)
{
    ctx.leg.roll_adjust += (float)ctx.input.roll_direction * 0.025f;
    ctx.leg.height_base += (float)ctx.input.leg_height_direction * 0.025f;
    ctx.leg.height_base = constrain(
        ctx.leg.height_base, LEG_HEIGHT_BASE_MIN, LEG_HEIGHT_BASE_MAX);

    float roll_angle = ctx.leg.roll_lpf(ctx.status.roll_angle / (float)PI * 180.0f);
    float leg_add = ctx.leg.roll_pid(roll_angle - ctx.leg.roll_adjust);
    int16_t left = (int16_t)(2048.0f + 8.4f * (30.0f - ctx.leg.height_base) - leg_add);
    int16_t right = (int16_t)(2048.0f - 8.4f * (30.0f - ctx.leg.height_base) - leg_add);
    left = (int16_t)((float)left + height_count_offset);
    right = (int16_t)((float)right - height_count_offset);
    left = constrain(left, hw::servo::LEG_LEFT_MIN, hw::servo::LEG_LEFT_MAX - 100);
    right = constrain(right, hw::servo::LEG_RIGHT_MAX + 100, hw::servo::LEG_RIGHT_MIN);
    set_pose(left, right, 1000, 0);
}

/**
 * @brief 判断恢复阶段是否已经满足稳定条件或等待超时
 *
 * @param runtime 动作运行状态
 * @param status 平衡状态
 * @param tick_ms 本次更新周期，单位毫秒
 * @param pitch_limit 俯仰角允许阈值
 * @param rate_limit 俯仰角速度允许阈值
 * @param hold_ms 稳定保持时间，单位毫秒
 * @param timeout_ms 恢复等待超时时间，单位毫秒
 *
 * @return 可以退出恢复阶段时返回 true
 */
static bool recover_ready(control::action::action_runtime &runtime,
    const control::status &status, uint32_t tick_ms, float pitch_limit,
    float rate_limit, uint32_t hold_ms, uint32_t timeout_ms)
{
    runtime.elapsed += tick_ms;
    if(fabsf(status.pitch_angle) < pitch_limit && fabsf(status.pitch_rate) < rate_limit)
    {
        runtime.ready_timer += tick_ms;
    }
    else
    {
        runtime.ready_timer = 0;
    }
    return runtime.ready_timer >= hold_ms || runtime.elapsed >= timeout_ms;
}

/**
 * @brief 生成恢复阶段使用的平衡命令
 *
 * @param runtime 动作运行状态
 * @param ctx 动作输入输出上下文
 *
 * @return 恢复平衡命令
 */
static control::balance_command recover_command(control::action::action_runtime &runtime,
    control::action::context &ctx)
{
    control::balance_command command;
    command.mode = control::balance_mode::RECOVER;
    command.recover_blend = constrain(
        (float)runtime.elapsed * 1.0e-3f / 0.22f, 0.0f, 1.0f);
    run_leg_control(ctx);
    return command;
}

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
static step_result step_boot(control::action::state &state,
    control::action::context &ctx, uint32_t tick_ms)
{
    step_result result;
    control::action::action_runtime &runtime = state.boot;
    switch(runtime.current_phase)
    {
        case control::action::phase::PREPARE:
            set_torque(0);
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
            set_pose(hw::servo::LEG_LEFT_MIN, hw::servo::LEG_RIGHT_MIN, 450, 250);
            reset_leg(ctx.leg);
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
            result.balance = recover_command(runtime, ctx);
            if(recover_ready(runtime, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
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
static step_result step_balance(control::action::state &,
    control::action::context &ctx, uint32_t)
{
    step_result result;
    result.balance.mode = control::balance_mode::BALANCE;
    result.balance.steering = true;
    result.balance.linear_vel = ctx.input.linear;
    result.balance.yaw_rate = ctx.input.yaw;
    run_leg_control(ctx);

    if(ctx.input.reset_leg){reset_leg(ctx.leg);}
    switch(ctx.input.action)
    {
        case control::action_request::RESET_BALANCE:
            result.next = transition::RESET_BALANCE;
            break;
        case control::action_request::SIT:
            result.next = transition::SIT;
            break;
        case control::action_request::MIDDLE_CALIBRATION:
            result.next = transition::MIDDLE_CALIBRATION;
            break;
        case control::action_request::JUMP_IN_PLACE:
            result.next = transition::JUMP;
            result.jump = control::action::jump_direction::IN_PLACE;
            break;
        case control::action_request::JUMP_FORWARD:
            result.next = transition::JUMP;
            result.jump = control::action::jump_direction::FORWARD;
            break;
        case control::action_request::JUMP_BACKWARD:
            result.next = transition::JUMP;
            result.jump = control::action::jump_direction::BACKWARD;
            break;
        case control::action_request::JUMP_LEFT:
            result.next = transition::JUMP;
            result.jump = control::action::jump_direction::TURN_LEFT;
            break;
        case control::action_request::JUMP_RIGHT:
            result.next = transition::JUMP;
            result.jump = control::action::jump_direction::TURN_RIGHT;
            break;
        case control::action_request::KICK_PLACE:
            result.next = transition::KICK_PLACE;
            break;
        case control::action_request::KICK_RUN:
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
static step_result step_stop(control::action::state &,
    control::action::context &ctx, uint32_t)
{
    step_result result;
    if(ctx.input.action == control::action_request::BOOT)
    {
        result.next = transition::BOOT;
    }
    return result;
}

/* ---- SIT 与中位校准 ---- */

/**
 * @brief 判断左右腿舵机当前位置是否接近中位
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
static bool sit_phase_can_enter_middle_calibration(control::action::phase current_phase)
{
    return current_phase == control::action::phase::PREPARE ||
           current_phase == control::action::phase::INIT_PREPARE ||
           current_phase == control::action::phase::MOVING ||
           current_phase == control::action::phase::DONE;
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
static step_result step_sit_flow(control::action::state &state,
    control::action::context &ctx, uint32_t tick_ms, bool calibration)
{
    step_result result;
    control::action::action_runtime &runtime = state.sit;
    switch(runtime.current_phase)
    {
        case control::action::phase::PREPARE:
            result.balance.mode = control::balance_mode::BALANCE;
            result.balance.steering = true;
            set_pose(hw::servo::LEG_LEFT_MIN, hw::servo::LEG_RIGHT_MIN, 450, 250);
            runtime.timer = 0;
            runtime.current_phase = control::action::phase::INIT_PREPARE;
            break;

        case control::action::phase::INIT_PREPARE:
            result.balance.mode = control::balance_mode::BALANCE;
            result.balance.steering = true;
            if(servo_middle_ready(ctx))
            {
                runtime.timer = 0;
                set_torque(2);
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
                if(runtime.timer >= MIDDLE_CALIBRATION_TORQUE_OFF_MS && runtime.ready_timer == 0)
                {
                    set_torque(0);
                    runtime.ready_timer = 1;
                }
                if(runtime.timer >= MIDDLE_CALIBRATION_RUN_MS && runtime.elapsed == 0)
                {
                    hw::servo::calibrate_middle();
                    runtime.elapsed = 1;
                }
                if(runtime.timer >= MIDDLE_CALIBRATION_SUCCESS_MS && runtime.elapsed == 1)
                {
                    state.middle_calibration_success = true;
                    runtime.elapsed = 2;
                }
            }
            else if((runtime.timer += tick_ms) >= 10000 ||
                    ctx.input.disable_leg_torque)
            {
                set_torque(0);
                runtime.timer = 10000;
            }

            if(ctx.input.action == control::action_request::EXIT &&
               !state.sit_exit_locked)
            {
                set_pose(hw::servo::LEG_LEFT_MIN, hw::servo::LEG_RIGHT_MIN, 450, 250);
                reset_leg(ctx.leg);
                runtime.current_phase = control::action::phase::EXIT_PREPARE;
            }
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
            result.balance = recover_command(runtime, ctx);
            if(recover_ready(runtime, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
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
 * @brief 执行 SIT 模式的一步
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 * @param tick_ms 周期，单位毫秒
 *
 * @return 本周期动作结果
 */
static step_result step_sit(control::action::state &state,
    control::action::context &ctx, uint32_t tick_ms)
{
    step_result result = step_sit_flow(state, ctx, tick_ms, false);
    if(ctx.input.action == control::action_request::MIDDLE_CALIBRATION &&
       sit_phase_can_enter_middle_calibration(state.sit.current_phase))
    {
        result.next = transition::MIDDLE_CALIBRATION;
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
static step_result step_middle_calibration(control::action::state &state,
    control::action::context &ctx, uint32_t tick_ms)
{
    return step_sit_flow(state, ctx, tick_ms, true);
}

/* ---- JUMP ---- */

/**
 * @brief 按跳跃方向初始化运行参数
 *
 * @param data 跳跃运行状态
 * @param direction 跳跃方向
 */
static void set_jump_direction(control::action::jump_runtime &data,
    control::action::jump_direction direction)
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
static control::balance_command update_jump_command(control::action::state &state,
    control::action::context &ctx)
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
        float error = angle_error(jump.target_yaw, ctx.status.yaw_angle);
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
static step_result step_jump(control::action::state &state,
    control::action::context &ctx, uint32_t tick_ms)
{
    step_result result;
    result.balance = update_jump_command(state, ctx);
    control::action::action_runtime &runtime = state.jump;
    control::action::jump_runtime &jump = state.jump_data;
    switch(runtime.current_phase)
    {
        case control::action::phase::PREPARE:
            jump.target_yaw = wrap_pi(
                ctx.status.yaw_angle + (float)jump.turn_dir * PI * 0.5f);
            set_pose(hw::servo::LEG_LEFT_MIN + 60, hw::servo::LEG_RIGHT_MIN - 60, 450, 250);
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
                set_pose(hw::servo::LEG_LEFT_MAX + 20, hw::servo::LEG_RIGHT_MAX - 20, 0, 0);
                runtime.timer = 0;
                runtime.current_phase = control::action::phase::FLY;
            }
            break;
        }

        case control::action::phase::FLY:
            runtime.timer += tick_ms;
            if(runtime.timer >= 130)
            {
                set_pose(hw::servo::LEG_LEFT_MIN + 60, hw::servo::LEG_RIGHT_MIN - 60, 0, 0);
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
                result.next = transition::DONE;
            }
            break;

        default:
            break;
    }
    return result;
}

/* ---- KICK ---- */

/**
 * @brief 设置踢球摄像头角度
 *
 * @param data 踢球运行状态
 * @param angle 目标角度
 */
static void set_camera(control::action::kick_runtime &data, float angle)
{
    data.cam_angle = constrain(angle, (float)hw::servo::CAMERA_MIN,
        (float)hw::servo::CAMERA_MAX);
    hw::servo::camera.set_angle((uint16_t)data.cam_angle);
}

/**
 * @brief 设置踢球挡板角度
 *
 * @param data 踢球运行状态
 * @param angle 目标角度
 */
static void set_frontier(control::action::kick_runtime &data, uint16_t angle)
{
    angle = constrain(angle, hw::servo::FRONTIER_MIN, hw::servo::FRONTIER_MAX);
    if(data.frontier_angle == angle){return;}
    data.frontier_angle = angle;
    hw::servo::frontier.set_angle(angle);
}

/**
 * @brief 生成踢球基础平衡命令
 *
 * @param ctx 动作上下文
 *
 * @return 踢球基础平衡命令
 */
static control::balance_command kick_base_command(control::action::context &ctx)
{
    control::balance_command command;
    command.mode = control::balance_mode::BALANCE;
    command.steering = true;
    command.linear_vel = ctx.input.linear;
    command.yaw_rate = ctx.input.yaw;
    run_leg_control(ctx, KICK_LEG_HEIGHT_COUNT_OFFSET);
    return command;
}

/**
 * @brief 判断一个视觉快照是否为新的视觉步
 *
 * @param data 踢球运行状态
 * @param sequence 当前视觉序号
 *
 * @return 新视觉步时返回 true
 */
static bool consume_vision_step(control::action::kick_runtime &data, uint32_t sequence)
{
    if(data.last_vision_seq == sequence){return false;}
    data.last_vision_seq = sequence;
    return true;
}

/**
 * @brief 根据视觉上下偏差调整摄像头
 *
 * @param data 踢球运行状态
 * @param dy 视觉上下偏差
 */
static void aim_camera(control::action::kick_runtime &data, int16_t dy)
{
    if(abs((int32_t)dy) <= CAM_AIM_DEADBAND)
    {
        data.last_dy = 0;
        data.cam_error = 0.0f;
        data.cam_rate = 0.0f;
        return;
    }

    uint32_t now_ms = millis();
    if(!data.last_dy_time)
    {
        data.last_dy_time = now_ms;
        data.last_dy = dy;
    }
    uint32_t dt_ms = max((uint32_t)1, (uint32_t)(now_ms - data.last_dy_time));
    float p_term = CAM_PD_P * (float)dy;
    float d_term = CAM_PD_D * (float)(dy - data.last_dy) / (float)dt_ms * 10.0f;
    float pd_output = constrain(p_term + d_term, -CAM_PD_STEP_LIMIT, CAM_PD_STEP_LIMIT);
    data.last_dy = dy;
    data.last_dy_time = now_ms;
    data.cam_error = (float)dy;
    data.cam_rate = -pd_output;
    set_camera(data, data.cam_angle - pd_output);
}

/**
 * @brief 根据视觉左右偏差生成瞄准转向速度
 *
 * @param dx 视觉左右偏差
 *
 * @return 瞄准转向速度
 */
static float aim_yaw_rate(int16_t dx)
{
    if(abs((int32_t)dx) < YAW_AIM_DEADBAND){return 0.0f;}
    float direction = dx > 0 ? 1.0f : -1.0f;
    float joy_rate = constrain((float)abs((int32_t)dx) * YAW_AIM_P,
        0.0f, JOY_RATE_LIMIT) * direction;
    return constrain(joy_rate * (YAW_RATE_LIMIT / JOY_RATE_LIMIT),
        -YAW_RATE_LIMIT, YAW_RATE_LIMIT);
}

/**
 * @brief 合并手动和视觉转向速度
 *
 * @param ctx 动作上下文
 * @param vision_yaw_rate 视觉转向速度
 *
 * @return 限幅后的转向速度
 */
static float combine_yaw_rate(const control::action::context &ctx, float vision_yaw_rate)
{
    return constrain(ctx.input.yaw + vision_yaw_rate,
        -ctx.max_steer_vel, ctx.max_steer_vel);
}

/**
 * @brief 判断手动线速度是否有效
 *
 * @param ctx 动作上下文
 *
 * @return 手动线速度有效时返回 true
 */
static bool manual_linear_active(const control::action::context &ctx)
{
    return fabsf(ctx.input.linear) > ctx.max_linear_vel * 0.05f;
}

/**
 * @brief 清理视觉丢失后的踢球控制状态
 *
 * @param data 踢球运行状态
 */
static void reset_lost_target(control::action::kick_runtime &data)
{
    data.cam_error = 0.0f;
    data.cam_rate = 0.0f;
    data.yaw_rate = 0.0f;
    data.last_dy = 0;
    data.last_dy_time = 0;
    set_camera(data, CAM_LOST_ANGLE);
    set_frontier(data, FRONTIER_KICK_ANGLE);
}

/**
 * @brief 在未踢球时把挡板置于准备角度
 *
 * @param data 踢球运行状态
 */
static void ready_kick(control::action::kick_runtime &data)
{
    if(!data.kicking){set_frontier(data, FRONTIER_READY_ANGLE);}
}

/**
 * @brief 触发一次踢球
 *
 * @param data 踢球运行状态
 */
static void trigger_kick(control::action::kick_runtime &data)
{
    set_frontier(data, FRONTIER_KICK_ANGLE);
    data.kicking = true;
    data.kick_timer = 0;
    data.kick_cooldown_timer = KICK_COOLDOWN_MS;
}

/**
 * @brief 判断是否允许触发踢球
 *
 * @param data 踢球运行状态
 *
 * @return 允许触发时返回 true
 */
static bool can_trigger_kick(const control::action::kick_runtime &data)
{
    return !data.kicking && data.kick_cooldown_timer == 0;
}

/**
 * @brief 更新踢球保持计时
 *
 * @param data 踢球运行状态
 * @param tick_ms 周期，单位毫秒
 */
static void update_kick_hold(control::action::kick_runtime &data, uint32_t tick_ms)
{
    if(!data.kicking){return;}
    data.kick_timer += tick_ms;
    if(data.kick_timer >= KICK_HOLD_MS)
    {
        data.kicking = false;
        data.kick_timer = 0;
    }
}

/**
 * @brief 更新踢球冷却计时
 *
 * @param data 踢球运行状态
 * @param tick_ms 周期，单位毫秒
 */
static void update_kick_cooldown(control::action::kick_runtime &data, uint32_t tick_ms)
{
    if(!data.kick_cooldown_timer){return;}
    if(data.kick_cooldown_timer <= tick_ms)
    {
        data.kick_cooldown_timer = 0;
        return;
    }
    data.kick_cooldown_timer -= tick_ms;
}

/**
 * @brief 开始踢球动作退出等待
 *
 * @param runtime 动作运行状态
 * @param data 踢球运行状态
 */
static void begin_kick_exit(control::action::action_runtime &runtime,
    control::action::kick_runtime &data)
{
    set_frontier(data, FRONTIER_KICK_ANGLE);
    data.kicking = false;
    data.kick_timer = 0;
    runtime.timer = 0;
    runtime.current_phase = control::action::phase::EXIT_PREPARE;
}

/**
 * @brief 更新踢球动作退出等待
 *
 * @param runtime 动作运行状态
 * @param ctx 动作上下文
 * @param result 动作结果
 * @param tick_ms 周期，单位毫秒
 *
 * @return 仍处于退出流程时返回 true
 */
static bool update_kick_exit(control::action::action_runtime &runtime,
    control::action::context &ctx, step_result &result, uint32_t tick_ms)
{
    if(runtime.current_phase != control::action::phase::EXIT_PREPARE){return false;}
    runtime.timer += tick_ms;
    if(runtime.timer < KICK_EXIT_DELAY_MS){return true;}
    reset_leg(ctx.leg);
    result.next = transition::DONE;
    return true;
}

/**
 * @brief 初始化一次踢球运行状态
 *
 * @param runtime 动作运行状态
 * @param data 踢球运行状态
 * @param ctx 动作上下文
 */
static void prepare_kick(control::action::action_runtime &runtime,
    control::action::kick_runtime &data, const control::action::context &ctx)
{
    data = control::action::kick_runtime{};
    data.target_yaw = ctx.status.yaw_angle;
    set_camera(data, CAM_INITIAL_ANGLE);
    runtime.current_phase = control::action::phase::MOVING;
}

/**
 * @brief 执行 KICK_PLACE 或 KICK_RUN 状态机的一步
 *
 * @param state 动作状态
 * @param ctx 动作上下文
 * @param tick_ms 周期，单位毫秒
 * @param run_mode 是否为追球踢模式
 *
 * @return 本周期动作结果
 */
static step_result step_kick(control::action::state &state,
    control::action::context &ctx, uint32_t tick_ms, bool run_mode)
{
    step_result result;
    control::action::action_runtime &runtime = run_mode ? state.kick_run : state.kick_place;
    control::action::kick_runtime &data = run_mode ?
        state.kick_run_data : state.kick_place_data;
    result.balance = kick_base_command(ctx);

    if(ctx.input.action == control::action_request::KICK_EXIT)
    {
        begin_kick_exit(runtime, data);
        return result;
    }
    if(runtime.current_phase != control::action::phase::EXIT_PREPARE)
    {
        control::action_request toggle = run_mode ?
            control::action_request::KICK_PLACE : control::action_request::KICK_RUN;
        if(ctx.input.action == toggle)
        {
            result.next = run_mode ? transition::KICK_PLACE : transition::KICK_RUN;
            return result;
        }
    }
    if(update_kick_exit(runtime, ctx, result, tick_ms)){return result;}
    if(runtime.current_phase == control::action::phase::PREPARE)
    {
        prepare_kick(runtime, data, ctx);
    }

    if(!run_mode){update_kick_hold(data, tick_ms);}
    update_kick_cooldown(data, tick_ms);

    if(run_mode && data.post_kick)
    {
        if(data.kicking)
        {
            data.kick_timer += tick_ms;
            if(data.kick_timer >= KICK_HOLD_MS)
            {
                data.kicking = false;
                data.kick_timer = 0;
            }
            return result;
        }

        data.post_timer += tick_ms;
        if(data.post_timer < RUN_AFTER_KICK_MS)
        {
            float error = angle_error(data.target_yaw, ctx.status.yaw_angle);
            if(fabsf(error) < YAW_ALIGN_LIMIT)
            {
                data.aligned = true;
                if(!manual_linear_active(ctx)){result.balance.linear_vel = RUN_BACK_VEL;}
            }
            else
            {
                float align_yaw_rate = constrain(
                    error * YAW_ALIGN_KP, -YAW_RATE_LIMIT, YAW_RATE_LIMIT);
                result.balance.yaw_rate = combine_yaw_rate(ctx, align_yaw_rate);
            }
            return result;
        }
        data.post_kick = false;
        data.post_timer = 0;
        data.chased = false;
        data.aligned = false;
    }

    if(!ctx.vision_valid)
    {
        reset_lost_target(data);
        return result;
    }

    if(consume_vision_step(data, ctx.vision_sequence))
    {
        aim_camera(data, ctx.vision_dy);
    }
    result.balance.yaw_rate = combine_yaw_rate(ctx, aim_yaw_rate(ctx.vision_dx));
    data.yaw_rate = result.balance.yaw_rate;

    if(!run_mode)
    {
        if(data.cam_angle < (float)PLACE_BALL_S2 &&
           ctx.vision_dy > PLACE_KICK_DY && ctx.vision_dy < KICK_DY_MAX)
        {
            if(can_trigger_kick(data)){trigger_kick(data);}
        }
        else
        {
            ready_kick(data);
        }
        return result;
    }

    if(data.chased)
    {
        if(can_trigger_kick(data))
        {
            trigger_kick(data);
            data.post_kick = true;
        }
        return result;
    }
    if(data.cam_angle < (float)CHASE_BALL_S2 &&
       ctx.vision_dy > RUN_KICK_DY && ctx.vision_dy < KICK_DY_MAX)
    {
        data.chased = true;
        return result;
    }

    int16_t obstacle_y = OB_BALL_DY - ctx.vision_dy;
    float forward = constrain((float)obstacle_y * CHASE_LINEAR_KP,
        0.0f, min(ctx.max_linear_vel, RUN_FORWARD_MAX));
    if(!manual_linear_active(ctx)){result.balance.linear_vel = forward;}
    ready_kick(data);
    return result;
}

/* ---- 显式状态切换与主入口 ---- */

/**
 * @brief 按目标模式初始化对应运行状态
 *
 * @param state 动作状态
 * @param next_mode 目标模式
 * @param jump 跳跃方向
 */
static void enter_mode(control::action::state &state, control::action::mode next_mode,
    control::action::jump_direction jump = control::action::jump_direction::IN_PLACE)
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
            set_jump_direction(state.jump_data, jump);
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
static void update_sit_exit_lock(control::action::state &state,
    const control::action::context &ctx)
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
static void apply_transition(control::action::state &state, const step_result &result)
{
    switch(result.next)
    {
        case transition::DONE:
            if(state.current_mode == control::action::mode::BOOT ||
               state.current_mode == control::action::mode::JUMP ||
               state.current_mode == control::action::mode::KICK_PLACE ||
               state.current_mode == control::action::mode::KICK_RUN)
            {
                enter_mode(state, control::action::mode::BALANCE);
            }
            else if((state.current_mode == control::action::mode::SIT ||
                     state.current_mode == control::action::mode::MIDDLE_CALIBRATION) &&
                    !state.sit_exit_locked)
            {
                enter_mode(state, control::action::mode::BALANCE);
            }
            break;
        case transition::BOOT:
            if(state.current_mode == control::action::mode::STOP && !state.sit_exit_locked)
            {
                enter_mode(state, control::action::mode::BOOT);
            }
            break;
        case transition::RESET_BALANCE:
            if(state.current_mode == control::action::mode::BALANCE)
            {
                enter_mode(state, control::action::mode::BALANCE);
            }
            break;
        case transition::SIT:
            if(state.current_mode == control::action::mode::BALANCE)
            {
                enter_mode(state, control::action::mode::SIT);
            }
            break;
        case transition::MIDDLE_CALIBRATION:
            if(state.current_mode == control::action::mode::BALANCE ||
               state.current_mode == control::action::mode::SIT)
            {
                enter_mode(state, control::action::mode::MIDDLE_CALIBRATION);
            }
            break;
        case transition::JUMP:
            if(state.current_mode == control::action::mode::BALANCE)
            {
                enter_mode(state, control::action::mode::JUMP, result.jump);
            }
            break;
        case transition::KICK_PLACE:
            if(state.current_mode == control::action::mode::BALANCE ||
               state.current_mode == control::action::mode::KICK_RUN)
            {
                enter_mode(state, control::action::mode::KICK_PLACE);
            }
            break;
        case transition::KICK_RUN:
            if(state.current_mode == control::action::mode::BALANCE ||
               state.current_mode == control::action::mode::KICK_PLACE)
            {
                enter_mode(state, control::action::mode::KICK_RUN);
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
control::action::mode control::action::current_mode(const control::action::state &state)
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
control::balance_command control::action::step(control::action::state &state,
    control::action::context &ctx, uint32_t tick_ms)
{
    update_sit_exit_lock(state, ctx);

    if(state.current_mode != control::action::mode::STOP &&
       ctx.input.action == control::action_request::STOP)
    {
        enter_mode(state, control::action::mode::STOP);
        return control::balance_command{};
    }

    step_result result;
    switch(state.current_mode)
    {
        case control::action::mode::BOOT:
            result = step_boot(state, ctx, tick_ms);
            break;
        case control::action::mode::BALANCE:
            result = step_balance(state, ctx, tick_ms);
            break;
        case control::action::mode::SIT:
            result = step_sit(state, ctx, tick_ms);
            break;
        case control::action::mode::JUMP:
            result = step_jump(state, ctx, tick_ms);
            break;
        case control::action::mode::STOP:
            result = step_stop(state, ctx, tick_ms);
            break;
        case control::action::mode::KICK_PLACE:
            result = step_kick(state, ctx, tick_ms, false);
            break;
        case control::action::mode::KICK_RUN:
            result = step_kick(state, ctx, tick_ms, true);
            break;
        case control::action::mode::MIDDLE_CALIBRATION:
            result = step_middle_calibration(state, ctx, tick_ms);
            break;
        default:
            enter_mode(state, control::action::mode::STOP);
            break;
    }

    apply_transition(state, result);
    return result.balance;
}
