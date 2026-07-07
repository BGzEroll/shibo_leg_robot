#include "action_common.h"

#include "sts3032.h"
#include "xbox.h"

static constexpr float LEG_HEIGHT_BASE_MIN = -10.0f;
static constexpr float LEG_HEIGHT_BASE_MAX = 52.0f;

/**
 * @brief 将角度归一化到 -PI 到 PI 范围内
 *
 * @param angle 角度值
 *
 * @return 归一化后的角度
 */
float controller::actions::wrap_pi(float angle)
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
float controller::actions::angle_error(float target, float current)
{
    return wrap_pi(target - current);
}

/**
 * @brief 设置左右腿舵机目标姿态并触发同步移动
 *
 * @param left 左侧目标位置
 * @param right 右侧目标位置
 * @param speed 舵机速度
 * @param accel 舵机加速度
 */
void controller::actions::set_pose(int16_t left, int16_t right, uint16_t speed, uint8_t accel)
{
    sts3032::set(SERVO_LEFT, left, speed, accel);
    sts3032::set(SERVO_RIGHT, right, speed, accel);
    sts3032::move();
}

/**
 * @brief 设置左右腿舵机扭矩模式
 *
 * @param type 扭矩模式类型
 */
void controller::actions::set_torque(uint8_t type)
{
    sts3032::set_torque_switch(SERVO_LEFT, type);
    sts3032::set_torque_switch(SERVO_RIGHT, type);
}

/**
 * @brief 复位腿部运行状态和横滚 PID
 *
 * @param leg 腿部运行状态
 */
void controller::actions::reset_leg(leg_runtime &leg)
{
    leg.roll_adjust = 0.0f;
    leg.height_base = (float)LEG_HEIGHT_BASE;
    leg.reset_roll_pid();
}

/**
 * @brief 根据输入和横滚姿态更新腿部舵机控制
 *
 * @param ctx 动作输入输出上下文
 * @param height_count_offset 腿高目标的舵机计数偏移量，正值抬高机身
 */
void controller::actions::run_leg_control(action_io &ctx, float height_count_offset)
{
    if((ctx.input.buttons & BTN_RIGHT) && !(ctx.input.buttons & ~BTN_RIGHT)){ctx.leg.roll_adjust += 0.025f;}
    if((ctx.input.buttons & BTN_LEFT) && !(ctx.input.buttons & ~BTN_LEFT)){ctx.leg.roll_adjust -= 0.025f;}
    if((ctx.input.buttons & BTN_UP) && !(ctx.input.buttons & ~BTN_UP)){ctx.leg.height_base -= 0.025f;}
    if((ctx.input.buttons & BTN_DOWN) && !(ctx.input.buttons & ~BTN_DOWN)){ctx.leg.height_base += 0.025f;}
    ctx.leg.height_base = constrain(ctx.leg.height_base, LEG_HEIGHT_BASE_MIN, LEG_HEIGHT_BASE_MAX);

    float roll_angle = ctx.leg.roll_lpf(ctx.status.roll_angle / (float)PI * 180.0f);
    float leg_add = ctx.leg.roll_pid(roll_angle - ctx.leg.roll_adjust);
    int16_t left = (int16_t)(2048.0f + 8.4f * (30.0f - ctx.leg.height_base) - leg_add);
    int16_t right = (int16_t)(2048.0f - 8.4f * (30.0f - ctx.leg.height_base) - leg_add);
    left = (int16_t)((float)left + height_count_offset);
    right = (int16_t)((float)right - height_count_offset);

    left = constrain(left, SERVO_LEFT_MIN, SERVO_LEFT_MAX - 100);
    right = constrain(right, SERVO_RIGHT_MAX + 100, SERVO_RIGHT_MIN);
    set_pose(left, right, 1000, 0);
}

/**
 * @brief 判断恢复阶段是否已经满足稳定条件或等待超时
 *
 * @param state 动作状态机状态
 * @param status 状态快照
 * @param tick_ms 本次更新周期，单位毫秒
 * @param pitch_limit 俯仰角允许阈值
 * @param rate_limit 俯仰角速度允许阈值
 * @param hold_ms 稳定保持时间，单位毫秒
 * @param timeout_ms 恢复等待超时时间，单位毫秒
 *
 * @return 已经可以退出恢复阶段时返回 true
 */
bool controller::actions::recover_ready(action_state &state, const balance_core::status_snapshot &status, uint32_t tick_ms,
    float pitch_limit, float rate_limit, uint32_t hold_ms, uint32_t timeout_ms)
{
    state.elapsed += tick_ms;
    if(fabsf(status.pitch_angle) < pitch_limit &&
       fabsf(status.pitch_rate) < rate_limit)
    {
        state.ready_timer += tick_ms;
    }
    else
    {
        state.ready_timer = 0;
    }

    return state.ready_timer >= hold_ms || state.elapsed >= timeout_ms;
}

/**
 * @brief 生成恢复阶段使用的平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 *
 * @return 生成的平衡请求
 */
controller::balance_request controller::actions::recover_command(action_state &state, action_io &ctx)
{
    balance_request cmd;
    cmd.command.enable_balance = true;
    cmd.command.enable_motor = true;
    cmd.command.recover_active = true;
    cmd.command.output_blend = constrain((float)state.elapsed * 1.0e-3f / 0.22f, 0.0f, 1.0f);
    run_leg_control(ctx);
    return cmd;
}

/**
 * @brief 按姿态序列推进舵机动作
 *
 * @param state 动作状态机状态
 * @param steps 姿态步骤列表
 * @param count 姿态步骤数量
 * @param tick_ms 本次更新周期，单位毫秒
 *
 * @return 姿态序列执行完成时返回 true
 */
bool controller::actions::run_pose_sequence(action_state &state, const pose_step *steps, uint8_t count, uint32_t tick_ms)
{
    if(steps == nullptr || count == 0){return true;}
    if(state.phase >= count){return true;}

    const pose_step &step = steps[state.phase];
    if(state.timer == 0)
    {
        set_pose(step.left, step.right, step.speed, step.accel);
    }

    state.timer += tick_ms;
    if(state.timer < step.hold_ms){return false;}

    state.timer = 0;
    state.phase++;
    return state.phase >= count;
}
