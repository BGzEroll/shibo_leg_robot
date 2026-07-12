#include "actuator_adapter.h"

#include "ptk7350.h"
#include "sts3032.h"

static port::latest_writer<actuator_port::leg_status> leg_status_output;
static uint32_t last_sample_us = 0;

/**
 * @brief 设置左右腿目标姿态
 *
 * @param left 左腿目标位置
 * @param right 右腿目标位置
 * @param speed 舵机速度
 * @param accel 舵机加速度
 */
static void set_leg_pose(int16_t left, int16_t right, uint16_t speed, uint8_t accel)
{
    sts3032::set(SERVO_LEFT, left, speed, accel);
    sts3032::set(SERVO_RIGHT, right, speed, accel);
    sts3032::move();
}

/**
 * @brief 设置左右腿扭矩模式
 *
 * @param type 扭矩模式
 */
static void set_leg_torque(uint8_t type)
{
    sts3032::set_torque_switch(SERVO_LEFT, type);
    sts3032::set_torque_switch(SERVO_RIGHT, type);
}

/**
 * @brief 执行左右腿中位校准
 */
static void calibrate_leg_middle()
{
    sts3032::calibrate_middle();
}

/**
 * @brief 设置摄像头舵机角度
 *
 * @param angle 舵机角度
 */
static void set_camera_angle(uint16_t angle)
{
    ptk7350::cam_servo.set_angle(angle);
}

/**
 * @brief 设置前挡板舵机角度
 *
 * @param angle 舵机角度
 */
static void set_frontier_angle(uint16_t angle)
{
    ptk7350::frontier_servo.set_angle(angle);
}

/**
 * @brief 获取动作模块使用的同步执行端口
 *
 * @return 同步执行端口
 */
actuator_port::services actuator_adapter::services()
{
    actuator_port::services ports;
    ports.set_leg_pose = set_leg_pose;
    ports.set_leg_torque = set_leg_torque;
    ports.calibrate_leg_middle = calibrate_leg_middle;
    ports.set_camera_angle = set_camera_angle;
    ports.set_frontier_angle = set_frontier_angle;
    return ports;
}

/**
 * @brief 按20毫秒周期采样左右腿状态并发布
 *
 * @param now_us 当前微秒时间戳
 */
void actuator_adapter::sample_leg_status(uint32_t now_us)
{
    if((uint32_t)(now_us - last_sample_us) < 20000){return;}
    last_sample_us = now_us;

    sts3032::get_position_and_load();
    actuator_port::leg_status status;
    status.timestamp_us = now_us;
    status.left_position = sts3032::status[0].position;
    status.right_position = sts3032::status[1].position;
    leg_status_output.publish(status);
}

/**
 * @brief 初始化执行器适配模块
 *
 * @param status_output 舵机状态输出端口
 */
void actuator_adapter::init(port::latest_writer<actuator_port::leg_status> status_output)
{
    leg_status_output = status_output;
    last_sample_us = 0;
}
