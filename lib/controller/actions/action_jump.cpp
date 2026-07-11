#include "action_jump.h"

struct jump_runtime
{
    int8_t linear_dir = 0;
    int8_t turn_dir = 0;
    float target_yaw = 0.0f;
    float linear_cmd = 0.0f;
    float yaw_cmd = 0.0f;
};

// 管理 JUMP 模式的阶段状态和跳跃方向参数。
class jump_action_impl : public controller::actions::action
{
    public:
        controller::mode_id mode() const override
        {
            return controller::mode_id::JUMP;
        }

        void enter(controller::action_io &ctx, controller::mode_id previous,
            const controller::action_enter_params &params) override
        {
            runtime = controller::actions::action_runtime{};
            jump = jump_runtime{};

            if(params.jump == controller::jump_command::FORWARD){jump.linear_dir = 1;}
            if(params.jump == controller::jump_command::BACKWARD){jump.linear_dir = -1;}
            if(params.jump == controller::jump_command::TURN_LEFT){jump.turn_dir = 1;}
            if(params.jump == controller::jump_command::TURN_RIGHT){jump.turn_dir = -1;}
        }

        controller::action_result update(controller::action_io &ctx, uint32_t tick_ms) override
        {
            controller::action_result result;
            result.balance = update_jump_command(ctx);

            switch(runtime.phase)
            {
                case controller::actions::PREPARE:
                    jump.target_yaw = controller::actions::wrap_pi(ctx.status.yaw_angle +
                                          (float)jump.turn_dir * PI * 0.5f);
                    controller::actions::set_pose(
                        ctx,
                        controller::robot_model::SERVO_LEFT_MIN + 60,
                        controller::robot_model::SERVO_RIGHT_MIN - 60,
                        450,
                        250);
                    result.balance.reset_yaw_integral = true;
                    runtime.phase = controller::actions::PUSH;
                    runtime.timer = 0;
                    break;

                case controller::actions::PUSH:
                {
                    uint32_t wait_ms = jump.linear_dir > 0 ? 650 : (jump.linear_dir < 0 ? 700 : 200);
                    if((runtime.timer += tick_ms) >= wait_ms)
                    {
                        controller::actions::set_pose(
                            ctx,
                            controller::robot_model::SERVO_LEFT_MAX + 20,
                            controller::robot_model::SERVO_RIGHT_MAX - 20,
                            0,
                            0);
                        runtime.timer = 0;
                        runtime.phase = controller::actions::FLY;
                    }
                    break;
                }

                case controller::actions::FLY:
                    if((runtime.timer += tick_ms) >= 130)
                    {
                        controller::actions::set_pose(
                            ctx,
                            controller::robot_model::SERVO_LEFT_MIN + 60,
                            controller::robot_model::SERVO_RIGHT_MIN - 60,
                            0,
                            0);
                        runtime.timer = 0;
                        runtime.phase = controller::actions::LAND;
                    }
                    break;

                case controller::actions::LAND:
                    if((runtime.timer += tick_ms) >= 260)
                    {
                        runtime.timer = 0;
                        runtime.elapsed = 0;
                        runtime.phase = controller::actions::RECOVER;
                    }
                    break;

                case controller::actions::RECOVER:
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
                        result.request = controller::action_request::ACTION_DONE;
                    }
                    break;
            }

            return result;
        }

        void exit(controller::action_io &ctx, controller::mode_id next) override
        {
        }

    private:
        controller::balance_request update_jump_command(controller::action_io &ctx)
        {
            controller::balance_request cmd;
            bool linear_jump = jump.linear_dir != 0;
            bool yaw_jump = jump.turn_dir != 0 || linear_jump;
            cmd.mode = controller::balance_drive_mode::BALANCE;
            cmd.enable_steering = yaw_jump;
            cmd.enable_yaw_integral = yaw_jump;

            float push_vel = 0.0f;
            uint32_t push_ramp_ms = 80;
            if(jump.linear_dir > 0)
            {
                push_vel = min(ctx.max_linear_vel, 0.40f);
                push_ramp_ms = 160;
            }
            else if(jump.linear_dir < 0)
            {
                push_vel = min(ctx.max_linear_vel, 0.34f);
                push_ramp_ms = 240;
            }

            if(runtime.phase == controller::actions::PUSH)
            {
                float ramp = constrain((float)runtime.timer / (float)push_ramp_ms, 0.0f, 1.0f);
                jump.linear_cmd = (float)jump.linear_dir * push_vel * ramp;
            }
            else
            {
                jump.linear_cmd = 0.0f;
            }

            cmd.linear_vel = jump.linear_cmd;
            if(yaw_jump)
            {
                float err = controller::actions::angle_error(jump.target_yaw, ctx.status.yaw_angle);
                float ff = 0.0f;
                float kp = jump.turn_dir == 0 ? 3.0f : 1.0f;
                float max_rate = jump.turn_dir == 0 ? 1.8f : 0.6f;

                if(jump.turn_dir != 0)
                {
                    if(runtime.phase == controller::actions::PUSH){ff = 1.2f; kp = 1.4f; max_rate = 1.8f;}
                    if(runtime.phase == controller::actions::FLY){ff = 6.4f; kp = 2.0f; max_rate = 6.4f;}
                    if(runtime.phase == controller::actions::LAND){kp = 0.35f; max_rate = 0.4f;}
                    if(runtime.phase == controller::actions::RECOVER){kp = 0.8f; max_rate = 0.5f;}
                }

                jump.yaw_cmd = constrain((float)jump.turn_dir * ff + kp * err, -max_rate, max_rate);
                cmd.yaw_rate = jump.yaw_cmd;
            }

            if(jump.linear_dir == 0 || runtime.phase != controller::actions::PUSH){cmd.enable_linear_feedback = false;}
            if(!yaw_jump){cmd.enable_yaw_feedback = false;}
            return cmd;
        }

    private:
        controller::actions::action_runtime runtime;
        jump_runtime jump;
};

static jump_action_impl jump_action_instance;

/**
 * @brief 获取 JUMP 动作对象
 *
 * @return JUMP 动作对象
 */
controller::actions::action &controller::actions::jump_action()
{
    return jump_action_instance;
}
