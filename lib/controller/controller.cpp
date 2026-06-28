#include "controller.h"

#include "actions.h"
#include "balance_core.h"
#include "host_comm.h"
#include "ptk7350.h"
#include "xbox_dev.h"

namespace balance_core
{
	void init();
}

namespace host_comm
{
	void init();
}

static controller::action_state action;
static controller::leg_runtime leg;
static controller::control_input input;
static balance_core::status_snapshot status;
static balance_core::info_t balance_info;
static uint16_t last_buttons = 0;
static float cam_angle = 90.0f;
static float cam_speed = 0.0f;
static int16_t cam_last_angle = -1;

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

/**
 * @brief 执行一次上层控制器更新
 *
 * @param tick_ms 本次更新周期，单位毫秒
 */
void controller::update(uint32_t tick_ms)
{
    balance_core::get_status(status);
    sample_input();
    update_camera(tick_ms);

    controller::action_io ctx{input, status, leg, balance_info.max_linear_vel};
    controller::balance_request request = controller::actions_update(action, ctx, tick_ms);

    balance_core::set_target(request.target);
    balance_core::set_command(request.command);
}

#ifdef DEBUG_MODE
/**
 * @brief 获取控制器调试快照
 *
 * @param out 调试快照输出
 *
 * @return 获取成功时返回 true
 */
bool controller::debug_snapshot(debug_snapshot_t &out)
{
	out.mode = (uint8_t)action.mode;
	out.phase = action.phase;
	out.kick_mode = action.mode == mode_id::KICK_PLACE || action.mode == mode_id::KICK_RUN;
	out.kick_cam_angle = action.kick.cam_angle;
	out.kick_cam_error = action.kick.cam_error;
	out.kick_cam_rate = action.kick.cam_rate;
	out.kick_yaw_rate = action.kick.yaw_rate;
	return true;
}
#endif

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
