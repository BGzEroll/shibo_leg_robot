#include "action_kick.h"

#include "action_common.h"
#include "host_comm.h"
#include "ptk7350.h"
#include "xbox.h"

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
static constexpr uint32_t RUN_AFTER_KICK_MS = 700;
static constexpr float KICK_LEG_HEIGHT_COUNT_OFFSET = 50.0f;

/**
 * @brief 设置摄像头舵机角度
 *
 * @param state 动作状态机状态
 * @param angle 目标角度
 */
static void set_camera(controller::action_state &state, float angle)
{
    state.kick.cam_angle = constrain(angle, (float)CAMSERVO_MIN, (float)CAMSERVO_MAX);
    ptk7350::cam_servo.set_angle((uint16_t)state.kick.cam_angle);
}

/**
 * @brief 设置前挡舵机角度
 *
 * @param state 动作状态机状态
 * @param angle 目标角度
 */
static void set_frontier(controller::action_state &state, uint16_t angle)
{
    angle = constrain(angle, (uint16_t)FRONTIERSERVO_MIN, (uint16_t)FRONTIERSERVO_MAX);
    if(state.kick.frontier_angle == angle){return;}

    state.kick.frontier_angle = angle;
    ptk7350::frontier_servo.set_angle(angle);
}

/**
 * @brief 生成踢球模式的基础平衡请求
 *
 * @param ctx 动作输入输出上下文
 *
 * @return 生成的平衡请求
 */
static controller::balance_request base_command(controller::action_io &ctx)
{
    controller::balance_request cmd;
    cmd.command.enable_balance = true;
    cmd.command.enable_motor = true;
    cmd.command.enable_steering = true;
    controller::actions::run_leg_control(ctx, KICK_LEG_HEIGHT_COUNT_OFFSET);
    return cmd;
}

/**
 * @brief 获取有效视觉测量
 *
 * @param out 视觉测量输出
 *
 * @return 当前视觉有效时返回 true
 */
static bool read_vision(host_comm::vision_measurement &out)
{
    return host_comm::vision_latest(out);
}

/**
 * @brief 判断视觉测量是否是尚未用于相机步进的新帧
 *
 * @param state 动作状态机状态
 * @param vision 视觉测量
 *
 * @return 新视觉帧时返回 true
 */
static bool consume_vision_step(controller::action_state &state, const host_comm::vision_measurement &vision)
{
    if(state.kick.last_vision_seq == vision.seq){return false;}

    state.kick.last_vision_seq = vision.seq;
    return true;
}

/**
 * @brief 根据 dy 按 Tommy 的 PD 步进调整摄像头俯仰角
 *
 * @param state 动作状态机状态
 * @param dy 目标纵向偏差
 */
static void aim_camera(controller::action_state &state, int16_t dy)
{
    if(abs((int32_t)dy) <= CAM_AIM_DEADBAND)
    {
        state.kick.last_dy = 0;
        state.kick.cam_error = 0.0f;
        state.kick.cam_rate = 0.0f;
        return;
    }

    uint32_t now = millis();
    if(!state.kick.last_dy_time)
    {
        state.kick.last_dy_time = now;
        state.kick.last_dy = dy;
    }

    uint32_t dt = max((uint32_t)1, (uint32_t)(now - state.kick.last_dy_time));
    float p_term = CAM_PD_P * (float)dy;
    float d_term = CAM_PD_D * (float)(dy - state.kick.last_dy) / (float)dt * 10.0f;
    float pd_output = constrain(p_term + d_term, -CAM_PD_STEP_LIMIT, CAM_PD_STEP_LIMIT);

    state.kick.last_dy = dy;
    state.kick.last_dy_time = now;
    state.kick.cam_error = (float)dy;
    state.kick.cam_rate = -pd_output;
    set_camera(state, state.kick.cam_angle - pd_output);
}

/**
 * @brief 根据 dx 生成瞄准转向速度
 *
 * @param dx 目标横向偏差
 *
 * @return 目标偏航角速度
 */
static float aim_yaw_rate(int16_t dx)
{
    if(abs((int32_t)dx) < YAW_AIM_DEADBAND){return 0.0f;}

    float direction = (dx > 0) ? 1.0f : -1.0f;
    float joy_rate = constrain((float)abs((int32_t)dx) * YAW_AIM_P, 0.0f, JOY_RATE_LIMIT) * direction;
    return constrain(joy_rate * (YAW_RATE_LIMIT / JOY_RATE_LIMIT), -YAW_RATE_LIMIT, YAW_RATE_LIMIT);
}

/**
 * @brief 处理丢失目标后的追踪复位
 *
 * @param state 动作状态机状态
 */
static void reset_lost_target(controller::action_state &state)
{
    state.kick.cam_error = 0.0f;
    state.kick.cam_rate = 0.0f;
    state.kick.yaw_rate = 0.0f;
    state.kick.last_dy = 0;
    state.kick.last_dy_time = 0;
    set_camera(state, CAM_LOST_ANGLE);
    set_frontier(state, FRONTIER_KICK_ANGLE);
}

/**
 * @brief 让踢球机构进入准备状态
 *
 * @param state 动作状态机状态
 */
static void ready_kick(controller::action_state &state)
{
    if(!state.kick.kicking)
    {
        set_frontier(state, FRONTIER_READY_ANGLE);
    }
}

/**
 * @brief 触发一次踢球动作
 *
 * @param state 动作状态机状态
 */
static void trigger_kick(controller::action_state &state)
{
    set_frontier(state, FRONTIER_KICK_ANGLE);
    state.kick.kicking = true;
    state.kick.kick_timer = 0;
    state.kick.kick_cooldown_timer = KICK_COOLDOWN_MS;
}

/**
 * @brief 判断当前是否允许再次触发踢球
 *
 * @param state 动作状态机状态
 *
 * @return 冷却结束且不在踢球保持阶段时返回 true
 */
static bool can_trigger_kick(const controller::action_state &state)
{
    return !state.kick.kicking && state.kick.kick_cooldown_timer == 0;
}

/**
 * @brief 推进踢球保持计时
 *
 * @param state 动作状态机状态
 * @param tick_ms 本次更新周期，单位毫秒
 */
static void update_kick_hold(controller::action_state &state, uint32_t tick_ms)
{
    if(!state.kick.kicking){return;}

    state.kick.kick_timer += tick_ms;
    if(state.kick.kick_timer >= KICK_HOLD_MS)
    {
        state.kick.kicking = false;
        state.kick.kick_timer = 0;
    }
}

/**
 * @brief 推进踢球触发冷却计时
 *
 * @param state 动作状态机状态
 * @param tick_ms 本次更新周期，单位毫秒
 */
static void update_kick_cooldown(controller::action_state &state, uint32_t tick_ms)
{
    if(!state.kick.kick_cooldown_timer){return;}

    if(state.kick.kick_cooldown_timer <= tick_ms)
    {
        state.kick.kick_cooldown_timer = 0;
        return;
    }

    state.kick.kick_cooldown_timer -= tick_ms;
}

/**
 * @brief 取消踢球模式并回到普通平衡
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 */
static void cancel_kick(controller::action_state &state, controller::action_io &ctx)
{
    set_frontier(state, FRONTIER_KICK_ANGLE);
    controller::actions::reset_leg(ctx.leg);
    controller::actions::begin_mode(state, controller::mode_id::BALANCE);
}

/**
 * @brief 初始化踢球模式状态
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 */
static void prepare_kick(controller::action_state &state, controller::action_io &ctx)
{
    state.kick = controller::kick_runtime{};
    state.kick.target_yaw = ctx.status.yaw_angle;
    set_camera(state, CAM_INITIAL_ANGLE);
    state.phase = controller::actions::MOVING;
}

/**
 * @brief 更新原地踢球模式状态机并生成平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 * @param tick_ms 本次更新周期，单位毫秒
 *
 * @return 生成的平衡请求
 */
controller::balance_request controller::actions::update_kick_place(controller::action_state &state, controller::action_io &ctx, uint32_t tick_ms)
{
    controller::balance_request cmd = base_command(ctx);
    if((ctx.input.buttons & BTN_SELECT) && (ctx.input.pressed_buttons & BTN_B))
    {
        cancel_kick(state, ctx);
        return cmd;
    }

    if(state.phase == controller::actions::PREPARE){prepare_kick(state, ctx);}
    update_kick_hold(state, tick_ms);
    update_kick_cooldown(state, tick_ms);

    host_comm::vision_measurement vision;
    if(!read_vision(vision))
    {
        reset_lost_target(state);
        return cmd;
    }

    if(consume_vision_step(state, vision)){aim_camera(state, vision.dy);}
    cmd.target.yaw_rate = aim_yaw_rate(vision.dx);
    state.kick.yaw_rate = cmd.target.yaw_rate;

    if(state.kick.cam_angle < (float)PLACE_BALL_S2 && vision.dy > PLACE_KICK_DY && vision.dy < KICK_DY_MAX)
    {
        if(can_trigger_kick(state)){trigger_kick(state);}
    }
    else
    {
        ready_kick(state);
    }
    return cmd;
}

/**
 * @brief 更新运动踢球模式状态机并生成平衡请求
 *
 * @param state 动作状态机状态
 * @param ctx 动作输入输出上下文
 * @param tick_ms 本次更新周期，单位毫秒
 *
 * @return 生成的平衡请求
 */
controller::balance_request controller::actions::update_kick_run(controller::action_state &state, controller::action_io &ctx, uint32_t tick_ms)
{
    controller::balance_request cmd = base_command(ctx);
    if((ctx.input.buttons & BTN_SELECT) && (ctx.input.pressed_buttons & BTN_B))
    {
        cancel_kick(state, ctx);
        return cmd;
    }

    if(state.phase == controller::actions::PREPARE){prepare_kick(state, ctx);}
    update_kick_cooldown(state, tick_ms);
    if(state.kick.post_kick)
    {
        if(state.kick.kicking)
        {
            state.kick.kick_timer += tick_ms;
            if(state.kick.kick_timer >= KICK_HOLD_MS)
            {
                state.kick.kicking = false;
                state.kick.kick_timer = 0;
            }
            return cmd;
        }

        state.kick.post_timer += tick_ms;
        if(state.kick.post_timer < RUN_AFTER_KICK_MS)
        {
            float err = controller::actions::angle_error(state.kick.target_yaw, ctx.status.yaw_angle);
            if(fabsf(err) < YAW_ALIGN_LIMIT)
            {
                state.kick.aligned = true;
                cmd.target.linear_vel = RUN_BACK_VEL;
            }
            else
            {
                cmd.target.yaw_rate = constrain(err * YAW_ALIGN_KP, -YAW_RATE_LIMIT, YAW_RATE_LIMIT);
            }
            return cmd;
        }

        state.kick.post_kick = false;
        state.kick.post_timer = 0;
        state.kick.chased = false;
        state.kick.aligned = false;
    }

    host_comm::vision_measurement vision;
    if(!read_vision(vision))
    {
        reset_lost_target(state);
        return cmd;
    }

    if(consume_vision_step(state, vision)){aim_camera(state, vision.dy);}
    cmd.target.yaw_rate = aim_yaw_rate(vision.dx);
    state.kick.yaw_rate = cmd.target.yaw_rate;

    if(state.kick.chased)
    {
        if(can_trigger_kick(state))
        {
            trigger_kick(state);
            state.kick.post_kick = true;
        }
        return cmd;
    }

    if(state.kick.cam_angle < (float)CHASE_BALL_S2 && vision.dy > RUN_KICK_DY && vision.dy < KICK_DY_MAX)
    {
        state.kick.chased = true;
        return cmd;
    }

    int16_t ob_y = OB_BALL_DY - vision.dy;
    float forward = constrain((float)ob_y * CHASE_LINEAR_KP, 0.0f, min(ctx.max_linear_vel, RUN_FORWARD_MAX));
    cmd.target.linear_vel = forward;
    ready_kick(state);
    return cmd;
}
