#include "action_internal.h"

#include "hw/servo.h"
#include <math.h>

/* ---- 公共动作参数 ---- */

static constexpr float LEG_HEIGHT_BASE_MIN = -10.0f;
static constexpr float LEG_HEIGHT_BASE_MAX = 52.0f;

/* ---- 公共动作工具 ---- */

/**
 * @brief 将角度归一化到 -PI 到 PI 范围内
 *
 * @param angle 角度值
 *
 * @return 归一化后的角度
 */
float control::action::internal::wrap_pi(float angle)
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
float control::action::internal::angle_error(float target, float current)
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
void control::action::internal::set_pose(int16_t left, int16_t right,
    uint16_t speed, uint8_t acceleration)
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
void control::action::internal::set_torque(uint8_t type)
{
    hw::servo::set_torque(hw::servo::LEG_LEFT, type);
    hw::servo::set_torque(hw::servo::LEG_RIGHT, type);
}

/**
 * @brief 复位腿部运行状态和横滚 PID
 *
 * @param leg 腿部运行状态
 */
void control::action::internal::reset_leg(control::action::leg_runtime &leg)
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
void control::action::internal::run_leg_control(control::action::context &ctx,
    float height_count_offset)
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
bool control::action::internal::recover_ready(
    control::action::action_runtime &runtime, const control::status &status,
    uint32_t tick_ms, float pitch_limit, float rate_limit, uint32_t hold_ms,
    uint32_t timeout_ms)
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
control::balance_command control::action::internal::recover_command(
    control::action::action_runtime &runtime, control::action::context &ctx)
{
    control::balance_command command;
    command.mode = control::balance_mode::RECOVER;
    command.recover_blend = constrain(
        (float)runtime.elapsed * 1.0e-3f / 0.22f, 0.0f, 1.0f);
    run_leg_control(ctx);
    return command;
}
