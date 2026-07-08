#include "controller.h"

#include "actions.h"
#include "balance_core.h"
#include "host_comm.h"
#include "ptk7350.h"
#include "xbox.h"
#include "xbox_dev.h"

namespace host_comm
{
    void init();
}

/* ---- 控制器运行状态 ---- */

static controller::action_state action;
static controller::leg_runtime leg;
static controller::control_input input;
static balance_core::motion_status status;
static balance_core::info balance_info;
static uint16_t last_buttons = 0;
static float cam_angle = 90.0f;
static float cam_speed = 0.0f;
static int16_t cam_last_angle = -1;
static bool middle_calibration_requested = false;
static bool middle_calibration_finished = false;
static portMUX_TYPE request_lock = portMUX_INITIALIZER_UNLOCKED;

/* ---- 控制器内部流程 ---- */

/**
 * @brief 将动作层语义请求转换为平衡核心当前控制命令
 *
 * @param request 动作层平衡请求
 */
static void apply_balance_request(const controller::balance_request &request)
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

/**
 * @brief 对输入值应用死区并重新归一化
 *
 * @param value 需要积分的值
 * @param deadband deadband
 *
 * @return 计算结果
 */
static float apply_deadband(float value, float deadband)
{
    if(fabsf(value) <= deadband){return 0.0f;}
    float mag = (fabsf(value) - deadband) / (1.0f - deadband);
    return value > 0.0f ? mag : -mag;
}

/**
 * @brief 采样手柄或上位机遥控输入
 */
static void sample_input()
{
    input = controller::control_input{};

    if(xbox_dev::connected())
    {
        xbox_dev::data gamepad_data;
        if(xbox_dev::queue() && xQueuePeek(xbox_dev::queue(), &gamepad_data, 0) == pdTRUE)
        {
            input.raw_buttons = gamepad_data.buttons;
            memcpy(input.axes, gamepad_data.axes, sizeof(input.axes));
        }
    }
    else
    {
        host_comm::remote_data remote;
        if(host_comm::remote_queue() && xQueuePeek(host_comm::remote_queue(), &remote, 0) == pdTRUE)
        {
            input.raw_buttons = remote.buttons;
            memcpy(input.axes, remote.axes, sizeof(input.axes));
        }
    }

    input.buttons = input.raw_buttons;
    if(input.buttons & BTN_SELECT)
    {
        input.buttons &= (uint16_t)~(BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT);
    }

    input.pressed_buttons = input.buttons & (uint16_t)(~last_buttons);
    last_buttons = input.buttons;

    float linear_axis = apply_deadband(input.axes[3], 0.05f);
    float yaw_axis = apply_deadband(input.axes[0], 0.05f);

    input.linear_cmd = linear_axis * balance_info.max_linear_vel;
    if(linear_axis < 0.0f){input.linear_cmd *= 0.8f;}
    input.yaw_cmd = -yaw_axis * balance_info.max_steer_vel;
}

/**
 * @brief 在可校准模式下取出舵机中位校准请求
 *
 * @return 有待处理请求时返回 true
 */
static bool consume_middle_calibration_request()
{
    controller::mode_id mode = controller::actions_mode(action);
    if(mode != controller::mode_id::BALANCE && mode != controller::mode_id::SIT){return false;}

    bool requested = false;
    portENTER_CRITICAL(&request_lock);
    requested = middle_calibration_requested;
    middle_calibration_requested = false;
    portEXIT_CRITICAL(&request_lock);
    return requested;
}

/**
 * @brief 更新摄像头舵机控制
 *
 * @param tick_ms 本次更新周期，单位毫秒
 */
static void update_camera(uint32_t tick_ms)
{
    bool modifier = (input.raw_buttons & BTN_SELECT) != 0;
    bool up = (input.raw_buttons & BTN_UP) != 0;
    bool down = (input.raw_buttons & BTN_DOWN) != 0;
    float target_speed = 0.0f;

    if(modifier)
    {
        if(up && !down){target_speed = 120.0f;}
        if(down && !up){target_speed = -120.0f;}
    }

    float dt = (float)tick_ms * 1.0e-3f;
    cam_speed += (target_speed - cam_speed) * (1.0f - expf(-dt / 0.05f));
    cam_angle = constrain(cam_angle + cam_speed * dt, (float)CAMSERVO_MIN, (float)CAMSERVO_MAX);

    if((int16_t)cam_angle != cam_last_angle)
    {
        cam_last_angle = (int16_t)cam_angle;
        ptk7350::cam_servo.set_angle((uint16_t)cam_angle);
    }
}

/* ---- controller 公共 API ---- */

/**
 * @brief 查询舵机中位校准流程是否已经成功执行
 *
 * @return 已成功执行时返回 true
 */
bool controller::middle_calibration_success()
{
    bool finished = false;
    portENTER_CRITICAL(&request_lock);
    finished = middle_calibration_finished;
    portEXIT_CRITICAL(&request_lock);
    return finished;
}

/**
 * @brief 标记舵机中位校准流程已经成功执行
 */
void controller::mark_middle_calibration_success()
{
    portENTER_CRITICAL(&request_lock);
    middle_calibration_finished = true;
    portEXIT_CRITICAL(&request_lock);
}

/**
 * @brief 请求执行舵机中位校准动作流程
 *
 * @return 请求写入成功时返回 true
 */
bool controller::request_middle_calibration()
{
    portENTER_CRITICAL(&request_lock);
    middle_calibration_requested = true;
    middle_calibration_finished = false;
    portEXIT_CRITICAL(&request_lock);
    return true;
}

/**
 * @brief 执行一次上层控制器更新
 *
 * @param tick_ms 本次更新周期，单位毫秒
 */
void controller::update(uint32_t tick_ms)
{
    balance_core::get_motion_status(status);
    sample_input();
    input.middle_calibration_request = consume_middle_calibration_request();
    update_camera(tick_ms);

    controller::action_io ctx{input, status, leg, balance_info.max_linear_vel, balance_info.max_steer_vel};
    controller::balance_request request = controller::actions_update(action, ctx, tick_ms);

    apply_balance_request(request);
}

/**
 * @brief 初始化控制器模块及其内部子模块
 */
void controller::init()
{
    host_comm::init();
    balance_core::init();
    balance_info = balance_core::get_info();
    controller::actions_init(action);
    leg = controller::leg_runtime{};
}
