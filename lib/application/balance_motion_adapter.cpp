#include "balance_motion_adapter.h"

#include "balance_core.h"

// 将控制器运动契约适配到当前 LQI 平衡核心。
class balance_motion_adapter_impl : public controller::motion_port
{
    public:
        bool latest_status(controller::motion_status &out) override
        {
            balance_core::motion_status status;
            if(!balance_core::get_motion_status(status)){return false;}

            out.timestamp_us = status.timestamp_us;
            out.pitch_angle = status.pitch_angle;
            out.pitch_rate = status.pitch_rate;
            out.avg_linear_vel = status.avg_linear_vel;
            out.yaw_angle = status.yaw_angle;
            out.yaw_rate = status.yaw_rate;
            out.roll_angle = status.roll_angle;
            out.avg_leg_height = status.avg_leg_height;
            return true;
        }

        controller::motion_info info() const override
        {
            balance_core::info core_info = balance_core::get_info();
            controller::motion_info info;
            info.max_linear_vel = core_info.max_linear_vel;
            info.max_steer_vel = core_info.max_steer_vel;
            return info;
        }

        void apply(const controller::balance_request &request) override
        {
            balance_core::motion_control motion;
            balance_core::direct_output_control direct_output;
            balance_core::recover_control recover;
            balance_core::feedback_override feedback;

            switch(request.mode)
            {
                case controller::balance_drive_mode::BALANCE:
                    motion.enable_motor = true;
                    motion.enable_balance = true;
                    motion.enable_steering = request.enable_steering;
                    motion.linear_vel = request.linear_vel;
                    motion.yaw_rate = request.yaw_rate;
                    break;

                case controller::balance_drive_mode::DIRECT_OUTPUT:
                    motion.enable_motor = true;
                    direct_output.enable = true;
                    direct_output.left = request.direct_left;
                    direct_output.right = request.direct_right;
                    break;

                case controller::balance_drive_mode::RECOVER:
                    motion.enable_motor = true;
                    motion.enable_balance = true;
                    motion.enable_steering = request.enable_steering;
                    recover.enable = true;
                    recover.output_blend = request.recover_blend;
                    break;

                case controller::balance_drive_mode::STOP:
                default:
                    break;
            }

            motion.reset_reference = request.reset_reference;
            motion.reset_yaw_integral = request.reset_yaw_integral;
            feedback.enable_linear_feedback = request.enable_linear_feedback;
            feedback.enable_yaw_feedback = request.enable_yaw_feedback;
            feedback.enable_yaw_integral = request.enable_yaw_integral;

            balance_core::apply_motion_control(motion);
            balance_core::apply_direct_output(direct_output);
            balance_core::apply_recover_control(recover);
            balance_core::apply_feedback_override(feedback);
        }

        void init() override
        {
            balance_core::init();
        }
};

static balance_motion_adapter_impl motion;

/**
 * @brief 获取当前 LQI 平衡核心运动适配器
 *
 * @return 运动适配器
 */
controller::motion_port &application::balance_motion()
{
    return motion;
}
