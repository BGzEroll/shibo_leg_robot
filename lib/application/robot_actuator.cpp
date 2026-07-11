#include "robot_actuator.h"

#include "ptk7350.h"
#include "sts3032.h"

// 将控制器执行器契约适配到当前机器人具体设备。
class robot_actuator_impl : public controller::actuator_port
{
    public:
        controller::actuator_feedback feedback() const override
        {
            controller::actuator_feedback feedback;
            feedback.left_leg_position = sts3032::status[0].position;
            feedback.right_leg_position = sts3032::status[1].position;
            return feedback;
        }

        void apply(const controller::actuator_intent &intent) override
        {
            if(intent.leg_pose.valid)
            {
                sts3032::set(
                    SERVO_LEFT,
                    intent.leg_pose.left,
                    (int16_t)intent.leg_pose.speed,
                    intent.leg_pose.accel);
                sts3032::set(
                    SERVO_RIGHT,
                    intent.leg_pose.right,
                    (int16_t)intent.leg_pose.speed,
                    intent.leg_pose.accel);
                sts3032::move();
            }
            if(intent.leg_torque.valid)
            {
                sts3032::set_torque_switch(SERVO_LEFT, intent.leg_torque.type);
                sts3032::set_torque_switch(SERVO_RIGHT, intent.leg_torque.type);
            }
            if(intent.accessory.camera_valid)
            {
                ptk7350::cam_servo.set_angle(intent.accessory.camera_angle);
            }
            if(intent.accessory.frontier_valid)
            {
                ptk7350::frontier_servo.set_angle(intent.accessory.frontier_angle);
            }
            if(intent.calibrate_middle)
            {
                sts3032::calibrate_middle();
            }
        }
};

static robot_actuator_impl actuator;

/**
 * @brief 获取当前机器人执行器适配器
 *
 * @return 执行器适配器
 */
controller::actuator_port &application::robot_actuator()
{
    return actuator;
}
