#include "action_internal.h"

#include "hw/servo.h"
#include <math.h>

namespace action = control::action;
namespace action_internal = control::action::internal;

using action::action_runtime;
using action::context;
using action::kick_runtime;
using action::phase;
using action_internal::step_result;
using action_internal::transition;
using control::action_request;
using control::balance_mode;

/* ---- KICK 参数 ---- */

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

/* ---- KICK 内部流程 ---- */

/**
 * @brief 设置踢球摄像头角度
 *
 * @param data 踢球运行状态
 * @param angle 目标角度
 */
static void set_camera(kick_runtime &data, float angle)
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
static void set_frontier(kick_runtime &data, uint16_t angle)
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
static control::balance_command kick_base_command(context &ctx)
{
    control::balance_command command;
    command.mode = balance_mode::BALANCE;
    command.steering = true;
    command.linear_vel = ctx.input.linear;
    command.yaw_rate = ctx.input.yaw;
    action_internal::run_leg_control(ctx, KICK_LEG_HEIGHT_COUNT_OFFSET);
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
static bool consume_vision_step(kick_runtime &data, uint32_t sequence)
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
static void aim_camera(kick_runtime &data, int16_t dy)
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
static float combine_yaw_rate(const context &ctx,
    float vision_yaw_rate)
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
static bool manual_linear_active(const context &ctx)
{
    return fabsf(ctx.input.linear) > ctx.max_linear_vel * 0.05f;
}

/**
 * @brief 清理视觉丢失后的踢球控制状态
 *
 * @param data 踢球运行状态
 */
static void reset_lost_target(kick_runtime &data)
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
static void ready_kick(kick_runtime &data)
{
    if(!data.kicking){set_frontier(data, FRONTIER_READY_ANGLE);}
}

/**
 * @brief 触发一次踢球
 *
 * @param data 踢球运行状态
 */
static void trigger_kick(kick_runtime &data)
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
static bool can_trigger_kick(const kick_runtime &data)
{
    return !data.kicking && data.kick_cooldown_timer == 0;
}

/**
 * @brief 更新踢球保持计时
 *
 * @param data 踢球运行状态
 * @param tick_ms 周期，单位毫秒
 */
static void update_kick_hold(kick_runtime &data, uint32_t tick_ms)
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
static void update_kick_cooldown(kick_runtime &data, uint32_t tick_ms)
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
static void begin_kick_exit(action_runtime &runtime, kick_runtime &data)
{
    set_frontier(data, FRONTIER_KICK_ANGLE);
    data.kicking = false;
    data.kick_timer = 0;
    runtime.timer = 0;
    runtime.current_phase = phase::EXIT_PREPARE;
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
static bool update_kick_exit(action_runtime &runtime, context &ctx,
    step_result &result, uint32_t tick_ms)
{
    if(runtime.current_phase != phase::EXIT_PREPARE){return false;}
    runtime.timer += tick_ms;
    if(runtime.timer < KICK_EXIT_DELAY_MS){return true;}
    action_internal::reset_leg(ctx.leg);
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
static void prepare_kick(action_runtime &runtime, kick_runtime &data,
    const context &ctx)
{
    data = kick_runtime{};
    data.target_yaw = ctx.status.yaw_angle;
    set_camera(data, CAM_INITIAL_ANGLE);
    runtime.current_phase = phase::MOVING;
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
step_result control::action::internal::step_kick(
    action::state &state, context &ctx,
    uint32_t tick_ms, bool run_mode)
{
    step_result result;
    action_runtime &runtime = run_mode ? state.kick_run : state.kick_place;
    kick_runtime &data = run_mode ?
        state.kick_run_data : state.kick_place_data;
    result.balance = kick_base_command(ctx);

    if(ctx.input.action == action_request::KICK_EXIT)
    {
        begin_kick_exit(runtime, data);
        return result;
    }
    if(runtime.current_phase != phase::EXIT_PREPARE)
    {
        action_request toggle = run_mode ?
            action_request::KICK_PLACE : action_request::KICK_RUN;
        if(ctx.input.action == toggle)
        {
            result.next = run_mode ?
                transition::KICK_PLACE : transition::KICK_RUN;
            return result;
        }
    }
    if(update_kick_exit(runtime, ctx, result, tick_ms)){return result;}
    if(runtime.current_phase == phase::PREPARE)
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
            float error = action_internal::angle_error(
                data.target_yaw, ctx.status.yaw_angle);
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
