#include "action_kick.h"

#include "host_comm.h"

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

struct kick_runtime
{
    float cam_angle = 90.0f;
    float target_yaw = 0.0f;
    int16_t last_dy = 0;
    uint16_t frontier_angle = 181;
    uint32_t last_dy_time = 0;
    uint32_t last_vision_seq = 0;
    uint32_t kick_timer = 0;
    uint32_t kick_cooldown_timer = 0;
    uint32_t post_timer = 0;
    bool chased = false;
    bool aligned = false;
    bool kicking = false;
    bool post_kick = false;
    float cam_error = 0.0f;
    float cam_rate = 0.0f;
    float yaw_rate = 0.0f;
};

/* ---- 踢球基础控制 ---- */

// 管理踢球动作的共同视觉、舵机和退出流程。
class kick_action_base : public controller::actions::action
{
    public:
        void enter(controller::action_io &ctx, controller::mode_id previous,
            const controller::action_enter_params &params) override
        {
            runtime = controller::actions::action_runtime{};
            kick = kick_runtime{};
        }

        void exit(controller::action_io &ctx, controller::mode_id next) override
        {
        }

    protected:
        void set_camera(controller::action_io &ctx, float angle)
        {
            kick.cam_angle = constrain(
                angle,
                (float)controller::robot_model::CAMERA_SERVO_MIN,
                (float)controller::robot_model::CAMERA_SERVO_MAX);
            ctx.actuator.accessory.camera_valid = true;
            ctx.actuator.accessory.camera_angle = (uint16_t)kick.cam_angle;
        }

        void set_frontier(controller::action_io &ctx, uint16_t angle)
        {
            angle = constrain(
                angle,
                controller::robot_model::FRONTIER_SERVO_MIN,
                controller::robot_model::FRONTIER_SERVO_MAX);
            if(kick.frontier_angle == angle){return;}

            kick.frontier_angle = angle;
            ctx.actuator.accessory.frontier_valid = true;
            ctx.actuator.accessory.frontier_angle = angle;
        }

        controller::balance_request kick_base_command(controller::action_io &ctx)
        {
            controller::balance_request cmd;
            cmd.mode = controller::balance_drive_mode::BALANCE;
            cmd.enable_steering = true;
            cmd.linear_vel = ctx.input.linear_cmd;
            cmd.yaw_rate = ctx.input.yaw_cmd;
            controller::actions::run_leg_control(ctx, KICK_LEG_HEIGHT_COUNT_OFFSET);
            return cmd;
        }

        bool read_vision(host_comm::vision_measurement &out)
        {
            return host_comm::vision_latest(out);
        }

        bool consume_vision_step(const host_comm::vision_measurement &vision)
        {
            if(kick.last_vision_seq == vision.seq){return false;}

            kick.last_vision_seq = vision.seq;
            return true;
        }

        void aim_camera(controller::action_io &ctx, int16_t dy)
        {
            if(abs((int32_t)dy) <= CAM_AIM_DEADBAND)
            {
                kick.last_dy = 0;
                kick.cam_error = 0.0f;
                kick.cam_rate = 0.0f;
                return;
            }

            uint32_t now = millis();
            if(!kick.last_dy_time)
            {
                kick.last_dy_time = now;
                kick.last_dy = dy;
            }

            uint32_t dt = max((uint32_t)1, (uint32_t)(now - kick.last_dy_time));
            float p_term = CAM_PD_P * (float)dy;
            float d_term = CAM_PD_D * (float)(dy - kick.last_dy) / (float)dt * 10.0f;
            float pd_output = constrain(p_term + d_term, -CAM_PD_STEP_LIMIT, CAM_PD_STEP_LIMIT);

            kick.last_dy = dy;
            kick.last_dy_time = now;
            kick.cam_error = (float)dy;
            kick.cam_rate = -pd_output;
            set_camera(ctx, kick.cam_angle - pd_output);
        }

        float aim_yaw_rate(int16_t dx)
        {
            if(abs((int32_t)dx) < YAW_AIM_DEADBAND){return 0.0f;}

            float direction = (dx > 0) ? 1.0f : -1.0f;
            float joy_rate = constrain((float)abs((int32_t)dx) * YAW_AIM_P, 0.0f, JOY_RATE_LIMIT) * direction;
            return constrain(joy_rate * (YAW_RATE_LIMIT / JOY_RATE_LIMIT), -YAW_RATE_LIMIT, YAW_RATE_LIMIT);
        }

        float combine_yaw_rate(const controller::action_io &ctx, float vision_yaw_rate)
        {
            return constrain(ctx.input.yaw_cmd + vision_yaw_rate, -ctx.max_steer_vel, ctx.max_steer_vel);
        }

        bool manual_linear_active(const controller::action_io &ctx)
        {
            return fabsf(ctx.input.linear_cmd) > ctx.max_linear_vel * 0.05f;
        }

        void reset_lost_target(controller::action_io &ctx)
        {
            kick.cam_error = 0.0f;
            kick.cam_rate = 0.0f;
            kick.yaw_rate = 0.0f;
            kick.last_dy = 0;
            kick.last_dy_time = 0;
            set_camera(ctx, CAM_LOST_ANGLE);
            set_frontier(ctx, FRONTIER_KICK_ANGLE);
        }

        void ready_kick(controller::action_io &ctx)
        {
            if(!kick.kicking)
            {
                set_frontier(ctx, FRONTIER_READY_ANGLE);
            }
        }

        void trigger_kick(controller::action_io &ctx)
        {
            set_frontier(ctx, FRONTIER_KICK_ANGLE);
            kick.kicking = true;
            kick.kick_timer = 0;
            kick.kick_cooldown_timer = KICK_COOLDOWN_MS;
        }

        bool can_trigger_kick() const
        {
            return !kick.kicking && kick.kick_cooldown_timer == 0;
        }

        void update_kick_hold(uint32_t tick_ms)
        {
            if(!kick.kicking){return;}

            kick.kick_timer += tick_ms;
            if(kick.kick_timer >= KICK_HOLD_MS)
            {
                kick.kicking = false;
                kick.kick_timer = 0;
            }
        }

        void update_kick_cooldown(uint32_t tick_ms)
        {
            if(!kick.kick_cooldown_timer){return;}

            if(kick.kick_cooldown_timer <= tick_ms)
            {
                kick.kick_cooldown_timer = 0;
                return;
            }

            kick.kick_cooldown_timer -= tick_ms;
        }

        void begin_kick_exit(controller::action_io &ctx)
        {
            set_frontier(ctx, FRONTIER_KICK_ANGLE);
            kick.kicking = false;
            kick.kick_timer = 0;
            runtime.timer = 0;
            runtime.phase = controller::actions::EXIT_PREPARE;
        }

        bool update_kick_exit(controller::action_result &result, controller::action_io &ctx, uint32_t tick_ms)
        {
            if(runtime.phase != controller::actions::EXIT_PREPARE){return false;}

            if((runtime.timer += tick_ms) < KICK_EXIT_DELAY_MS){return true;}

            controller::actions::reset_leg(ctx.leg);
            result.request = controller::action_request::ACTION_DONE;
            return true;
        }

        void prepare_kick(controller::action_io &ctx)
        {
            kick = kick_runtime{};
            kick.target_yaw = ctx.status.yaw_angle;
            set_camera(ctx, CAM_INITIAL_ANGLE);
            runtime.phase = controller::actions::MOVING;
        }

    protected:
        controller::actions::action_runtime runtime;
        kick_runtime kick;
};

/* ---- 踢球动作对象 ---- */

class kick_place_action_impl : public kick_action_base
{
    public:
        controller::mode_id mode() const override
        {
            return controller::mode_id::KICK_PLACE;
        }

        controller::action_result update(controller::action_io &ctx, uint32_t tick_ms) override
        {
            controller::action_result result;
            result.balance = kick_base_command(ctx);

            if(ctx.input.request == controller::action_request::KICK_EXIT)
            {
                begin_kick_exit(ctx);
                return result;
            }
            if(runtime.phase != controller::actions::EXIT_PREPARE &&
               ctx.input.request == controller::action_request::KICK_RUN)
            {
                result.request = controller::action_request::KICK_RUN;
                return result;
            }
            if(update_kick_exit(result, ctx, tick_ms)){return result;}

            if(runtime.phase == controller::actions::PREPARE){prepare_kick(ctx);}
            update_kick_hold(tick_ms);
            update_kick_cooldown(tick_ms);

            host_comm::vision_measurement vision;
            if(!read_vision(vision))
            {
                reset_lost_target(ctx);
                return result;
            }

            if(consume_vision_step(vision)){aim_camera(ctx, vision.dy);}
            result.balance.yaw_rate = combine_yaw_rate(ctx, aim_yaw_rate(vision.dx));
            kick.yaw_rate = result.balance.yaw_rate;

            if(kick.cam_angle < (float)PLACE_BALL_S2 && vision.dy > PLACE_KICK_DY && vision.dy < KICK_DY_MAX)
            {
                if(can_trigger_kick()){trigger_kick(ctx);}
            }
            else
            {
                ready_kick(ctx);
            }
            return result;
        }
};

class kick_run_action_impl : public kick_action_base
{
    public:
        controller::mode_id mode() const override
        {
            return controller::mode_id::KICK_RUN;
        }

        controller::action_result update(controller::action_io &ctx, uint32_t tick_ms) override
        {
            controller::action_result result;
            result.balance = kick_base_command(ctx);

            if(ctx.input.request == controller::action_request::KICK_EXIT)
            {
                begin_kick_exit(ctx);
                return result;
            }
            if(runtime.phase != controller::actions::EXIT_PREPARE &&
               ctx.input.request == controller::action_request::KICK_PLACE)
            {
                result.request = controller::action_request::KICK_PLACE;
                return result;
            }
            if(update_kick_exit(result, ctx, tick_ms)){return result;}

            if(runtime.phase == controller::actions::PREPARE){prepare_kick(ctx);}
            update_kick_cooldown(tick_ms);
            if(kick.post_kick)
            {
                if(kick.kicking)
                {
                    kick.kick_timer += tick_ms;
                    if(kick.kick_timer >= KICK_HOLD_MS)
                    {
                        kick.kicking = false;
                        kick.kick_timer = 0;
                    }
                    return result;
                }

                kick.post_timer += tick_ms;
                if(kick.post_timer < RUN_AFTER_KICK_MS)
                {
                    float err = controller::actions::angle_error(kick.target_yaw, ctx.status.yaw_angle);
                    if(fabsf(err) < YAW_ALIGN_LIMIT)
                    {
                        kick.aligned = true;
                        if(!manual_linear_active(ctx)){result.balance.linear_vel = RUN_BACK_VEL;}
                    }
                    else
                    {
                        float align_yaw_rate = constrain(err * YAW_ALIGN_KP, -YAW_RATE_LIMIT, YAW_RATE_LIMIT);
                        result.balance.yaw_rate = combine_yaw_rate(ctx, align_yaw_rate);
                    }
                    return result;
                }

                kick.post_kick = false;
                kick.post_timer = 0;
                kick.chased = false;
                kick.aligned = false;
            }

            host_comm::vision_measurement vision;
            if(!read_vision(vision))
            {
                reset_lost_target(ctx);
                return result;
            }

            if(consume_vision_step(vision)){aim_camera(ctx, vision.dy);}
            result.balance.yaw_rate = combine_yaw_rate(ctx, aim_yaw_rate(vision.dx));
            kick.yaw_rate = result.balance.yaw_rate;

            if(kick.chased)
            {
                if(can_trigger_kick())
                {
                    trigger_kick(ctx);
                    kick.post_kick = true;
                }
                return result;
            }

            if(kick.cam_angle < (float)CHASE_BALL_S2 && vision.dy > RUN_KICK_DY && vision.dy < KICK_DY_MAX)
            {
                kick.chased = true;
                return result;
            }

            int16_t ob_y = OB_BALL_DY - vision.dy;
            float forward = constrain((float)ob_y * CHASE_LINEAR_KP, 0.0f, min(ctx.max_linear_vel, RUN_FORWARD_MAX));
            if(!manual_linear_active(ctx)){result.balance.linear_vel = forward;}
            ready_kick(ctx);
            return result;
        }
};

static kick_place_action_impl kick_place_action_instance;
static kick_run_action_impl kick_run_action_instance;

/**
 * @brief 获取 KICK_PLACE 动作对象
 *
 * @return KICK_PLACE 动作对象
 */
controller::actions::action &controller::actions::kick_place_action()
{
    return kick_place_action_instance;
}

/**
 * @brief 获取 KICK_RUN 动作对象
 *
 * @return KICK_RUN 动作对象
 */
controller::actions::action &controller::actions::kick_run_action()
{
    return kick_run_action_instance;
}
