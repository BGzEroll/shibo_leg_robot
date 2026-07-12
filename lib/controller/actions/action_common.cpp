#include "action_common.h"

static constexpr float LEG_HEIGHT_BASE_MIN = -10.0f;
static constexpr float LEG_HEIGHT_BASE_MAX = 52.0f;
static actuator_port::services actuator_services;

/* ---- 基础动作工具 ---- */

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
 * @brief 绑定动作模块使用的同步执行端口
 *
 * @param actuators 同步执行端口
 */
void controller::actions::bind_actuators(const actuator_port::services &actuators)
{
    actuator_services = actuators;
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
    if(actuator_services.set_leg_pose)
    {
        actuator_services.set_leg_pose(left, right, speed, accel);
    }
}

/**
 * @brief 设置左右腿舵机扭矩模式
 *
 * @param type 扭矩模式类型
 */
void controller::actions::set_torque(uint8_t type)
{
    if(actuator_services.set_leg_torque){actuator_services.set_leg_torque(type);}
}

/**
 * @brief 执行左右腿中位校准
 */
void controller::actions::calibrate_leg_middle()
{
    if(actuator_services.calibrate_leg_middle){actuator_services.calibrate_leg_middle();}
}

/**
 * @brief 设置摄像头舵机角度
 *
 * @param angle 舵机角度
 */
void controller::actions::set_camera_angle(uint16_t angle)
{
    if(actuator_services.set_camera_angle){actuator_services.set_camera_angle(angle);}
}

/**
 * @brief 设置前挡板舵机角度
 *
 * @param angle 舵机角度
 */
void controller::actions::set_frontier_angle(uint16_t angle)
{
    if(actuator_services.set_frontier_angle){actuator_services.set_frontier_angle(angle);}
}

/* ---- 腿部与恢复控制 ---- */

/**
 * @brief 复位腿部运行状态和横滚 PID
 *
 * @param leg 腿部运行状态
 */
void controller::actions::reset_leg(leg_runtime &leg)
{
    leg.roll_adjust = 0.0f;
    leg.height_base = leg_contract::HEIGHT_BASE;
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
    ctx.leg.roll_adjust += (float)ctx.input.roll_direction * 0.025f;
    ctx.leg.height_base += (float)ctx.input.leg_height_direction * 0.025f;
    ctx.leg.height_base = constrain(ctx.leg.height_base, LEG_HEIGHT_BASE_MIN, LEG_HEIGHT_BASE_MAX);

    float roll_angle = ctx.leg.roll_lpf(ctx.status.roll_angle / (float)PI * 180.0f);
    float leg_add = ctx.leg.roll_pid(roll_angle - ctx.leg.roll_adjust);
    int16_t left = (int16_t)(2048.0f + 8.4f * (30.0f - ctx.leg.height_base) - leg_add);
    int16_t right = (int16_t)(2048.0f - 8.4f * (30.0f - ctx.leg.height_base) - leg_add);
    left = (int16_t)((float)left + height_count_offset);
    right = (int16_t)((float)right - height_count_offset);

    left = constrain(left, leg_contract::LEFT_MIN, leg_contract::LEFT_MAX - 100);
    right = constrain(right, leg_contract::RIGHT_MAX + 100, leg_contract::RIGHT_MIN);
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
bool controller::actions::recover_ready(action_runtime &runtime, const balance_core::motion_status &status, uint32_t tick_ms,
    float pitch_limit, float rate_limit, uint32_t hold_ms, uint32_t timeout_ms)
{
    runtime.elapsed += tick_ms;
    if(fabsf(status.pitch_angle) < pitch_limit &&
       fabsf(status.pitch_rate) < rate_limit)
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
 * @brief 生成恢复阶段使用的平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 *
 * @return 生成的平衡请求
 */
controller::balance_request controller::actions::recover_command(action_runtime &runtime, action_io &ctx)
{
    balance_request cmd;
    cmd.mode = controller::balance_drive_mode::RECOVER;
    cmd.recover_blend = constrain((float)runtime.elapsed * 1.0e-3f / 0.22f, 0.0f, 1.0f);
    run_leg_control(ctx);
    return cmd;
}

/* ---- 姿态序列控制 ---- */

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
bool controller::actions::run_pose_sequence(action_runtime &runtime, const pose_step *steps, uint8_t count, uint32_t tick_ms)
{
    if(steps == nullptr || count == 0){return true;}
    if(runtime.phase >= count){return true;}

    const pose_step &step = steps[runtime.phase];
    if(runtime.timer == 0)
    {
        set_pose(step.left, step.right, step.speed, step.accel);
    }

    runtime.timer += tick_ms;
    if(runtime.timer < step.hold_ms){return false;}

    runtime.timer = 0;
    runtime.phase++;
    return runtime.phase >= count;
}
