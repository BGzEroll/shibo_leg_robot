#include "input_router.h"

#include "actions.h"
#include "esp_timer.h"
#include "host_comm.h"
#include "web_server.h"
#include "xbox.h"
#include "xbox_dev.h"

/* ---- 输入路由运行状态 ---- */

static constexpr uint32_t INPUT_TIMEOUT_US = 250000;
static constexpr uint8_t AXIS_YAW = 0;
static constexpr uint8_t AXIS_LINEAR = 3;

struct input_snapshot
{
    controller::input_source source = controller::input_source::NONE;
    uint32_t timestamp_us = 0;
    uint16_t buttons = 0;
    uint16_t pressed_buttons = 0;
    float axes[6]{};
    bool fresh = false;
};

static controller::input_source last_source = controller::input_source::NONE;
static uint16_t last_buttons = 0;
static bool last_fresh = false;

/* ---- 输入采样与归一化 ---- */

/**
 * @brief 对输入轴应用死区并重新归一化
 *
 * @param value 原始输入值
 * @param deadband 死区比例
 *
 * @return 归一化后的输入值
 */
static float apply_deadband(float value, float deadband)
{
    if(fabsf(value) <= deadband){return 0.0f;}
    float magnitude = (fabsf(value) - deadband) / (1.0f - deadband);
    return value > 0.0f ? magnitude : -magnitude;
}

/**
 * @brief 判断输入时间戳是否仍在有效期内
 *
 * @param timestamp_us 输入时间戳
 * @param now_us 当前时间戳
 *
 * @return 输入仍然新鲜时返回 true
 */
static bool input_fresh(uint32_t timestamp_us, uint32_t now_us)
{
    return timestamp_us != 0 &&
           (uint32_t)(now_us - timestamp_us) <= INPUT_TIMEOUT_US;
}

/**
 * @brief 读取当前优先输入源的最新快照
 *
 * @return 输入快照
 */
static input_snapshot read_snapshot()
{
    input_snapshot snapshot;
    uint32_t now_us = (uint32_t)esp_timer_get_time();

    if(xbox_dev::connected())
    {
        snapshot.source = controller::input_source::XBOX;

        xbox_dev::data data;
        if(xbox_dev::queue() && xQueuePeek(xbox_dev::queue(), &data, 0) == pdTRUE)
        {
            snapshot.timestamp_us = data.timestamp_us;
            snapshot.buttons = data.buttons;
            memcpy(snapshot.axes, data.axes, sizeof(snapshot.axes));
        }
    }
    else
    {
        web_server::remote_input web_data;
        if(web_server::take_remote_input(web_data) &&
           input_fresh(web_data.timestamp_us, now_us))
        {
            snapshot.source = controller::input_source::WEB;
            snapshot.timestamp_us = web_data.timestamp_us;
            snapshot.buttons = web_data.held_buttons;
            snapshot.pressed_buttons = web_data.pressed_buttons;
            memcpy(snapshot.axes, web_data.axes, sizeof(snapshot.axes));
            snapshot.fresh = true;
            return snapshot;
        }

        snapshot.source = controller::input_source::HOST;
        host_comm::remote_data data;
        if(host_comm::remote_queue() && xQueuePeek(host_comm::remote_queue(), &data, 0) == pdTRUE)
        {
            snapshot.timestamp_us = data.timestamp_us;
            snapshot.buttons = data.buttons;
            memcpy(snapshot.axes, data.axes, sizeof(snapshot.axes));
        }
    }

    snapshot.fresh = input_fresh(snapshot.timestamp_us, now_us);
    if(!snapshot.fresh)
    {
        snapshot.buttons = 0;
        memset(snapshot.axes, 0, sizeof(snapshot.axes));
    }

    return snapshot;
}

/**
 * @brief 按当前动作模式生成唯一离散动作请求
 *
 * @param mode 当前动作模式
 * @param held_buttons 当前按住按钮
 * @param pressed_buttons 本周期新按下按钮
 * @param out 路由后的控制输入
 */
static void route_action_request(controller::mode_id mode, uint16_t held_buttons,
    uint16_t pressed_buttons, float max_linear_vel, controller::control_input &out)
{
    if(mode != controller::mode_id::STOP && (pressed_buttons & xbox::BUTTON_START))
    {
        out.request = controller::action_request::STOP;
        return;
    }

    bool modifier = (held_buttons & xbox::BUTTON_SELECT) != 0;
    switch(mode)
    {
        case controller::mode_id::BOOT:
            if(held_buttons & xbox::BUTTON_RB)
            {
                out.request = controller::action_request::BOOT_CONFIRM;
            }
            break;

        case controller::mode_id::STOP:
            if(held_buttons & xbox::BUTTON_RB)
            {
                out.request = controller::action_request::BOOT;
            }
            break;

        case controller::mode_id::BALANCE:
            out.reset_leg =
                (pressed_buttons & xbox::BUTTON_LS) &&
                fabsf(out.linear_cmd) < max_linear_vel * 0.05f;

            if(modifier)
            {
                if(pressed_buttons & xbox::BUTTON_X)
                {
                    out.request = controller::action_request::KICK_PLACE;
                }
                else if(pressed_buttons & xbox::BUTTON_Y)
                {
                    out.request = controller::action_request::KICK_RUN;
                }
                else if(pressed_buttons & xbox::BUTTON_B)
                {
                    out.request = controller::action_request::RESET_BALANCE;
                }
                break;
            }

            if(pressed_buttons & xbox::BUTTON_B)
            {
                out.request = controller::action_request::JUMP_RIGHT;
            }
            else if(pressed_buttons & xbox::BUTTON_X)
            {
                out.request = controller::action_request::JUMP_LEFT;
            }
            else if(pressed_buttons & xbox::BUTTON_A)
            {
                out.request = controller::action_request::JUMP_BACKWARD;
            }
            else if(pressed_buttons & xbox::BUTTON_Y)
            {
                out.request = controller::action_request::JUMP_FORWARD;
            }
            else if(pressed_buttons & xbox::BUTTON_RS)
            {
                out.request = controller::action_request::JUMP_IN_PLACE;
            }
            else if(pressed_buttons & xbox::BUTTON_LB)
            {
                out.request = controller::action_request::SIT;
            }
            break;

        case controller::mode_id::SIT:
            out.disable_leg_torque = (held_buttons & xbox::BUTTON_LS) != 0;
            out.exit_action = (held_buttons & xbox::BUTTON_RB) != 0;
            break;

        case controller::mode_id::MIDDLE_CALIBRATION:
            out.exit_action = (held_buttons & xbox::BUTTON_RB) != 0;
            break;

        case controller::mode_id::KICK_PLACE:
            if(modifier && (pressed_buttons & xbox::BUTTON_B))
            {
                out.request = controller::action_request::KICK_EXIT;
            }
            else if(modifier && (pressed_buttons & xbox::BUTTON_Y))
            {
                out.request = controller::action_request::KICK_RUN;
            }
            break;

        case controller::mode_id::KICK_RUN:
            if(modifier && (pressed_buttons & xbox::BUTTON_B))
            {
                out.request = controller::action_request::KICK_EXIT;
            }
            else if(modifier && (pressed_buttons & xbox::BUTTON_X))
            {
                out.request = controller::action_request::KICK_PLACE;
            }
            break;

        case controller::mode_id::JUMP:
        default:
            break;
    }
}

/* ---- input_router 公共 API ---- */

/**
 * @brief 更新统一控制输入和语义动作请求
 *
 * @param mode 当前动作模式
 * @param max_linear_vel 最大线速度
 * @param max_steer_vel 最大转向角速度
 * @param out 控制输入输出
 */
void controller::input_router::update(controller::mode_id mode, float max_linear_vel,
    float max_steer_vel, controller::control_input &out)
{
    out = controller::control_input{};
    input_snapshot snapshot = read_snapshot();

    out.source = snapshot.source;
    out.timestamp_us = snapshot.timestamp_us;
    out.fresh = snapshot.fresh;

    uint16_t raw_buttons = snapshot.buttons;
    out.camera_direction = 0;
    if(raw_buttons & xbox::BUTTON_SELECT)
    {
        bool up = (raw_buttons & xbox::BUTTON_UP) != 0;
        bool down = (raw_buttons & xbox::BUTTON_DOWN) != 0;
        if(up && !down){out.camera_direction = 1;}
        if(down && !up){out.camera_direction = -1;}
    }

    uint16_t held_buttons = raw_buttons;
    if(held_buttons & xbox::BUTTON_SELECT)
    {
        held_buttons &= (uint16_t)~(
            xbox::BUTTON_UP |
            xbox::BUTTON_DOWN |
            xbox::BUTTON_LEFT |
            xbox::BUTTON_RIGHT);
    }

    uint16_t pressed_buttons = 0;
    if(snapshot.source == controller::input_source::WEB)
    {
        pressed_buttons = snapshot.pressed_buttons;
    }
    else if(snapshot.source == last_source && last_fresh && snapshot.fresh)
    {
        pressed_buttons = held_buttons & (uint16_t)(~last_buttons);
    }

    last_source = snapshot.source;
    last_buttons = held_buttons;
    last_fresh = snapshot.fresh;

    float linear_axis = apply_deadband(snapshot.axes[AXIS_LINEAR], 0.05f);
    float yaw_axis = apply_deadband(snapshot.axes[AXIS_YAW], 0.05f);
    out.linear_cmd = linear_axis * max_linear_vel;
    if(linear_axis < 0.0f){out.linear_cmd *= 0.8f;}
    out.yaw_cmd = -yaw_axis * max_steer_vel;

    if(held_buttons == xbox::BUTTON_RIGHT){out.roll_direction = 1;}
    if(held_buttons == xbox::BUTTON_LEFT){out.roll_direction = -1;}
    if(held_buttons == xbox::BUTTON_UP){out.leg_height_direction = -1;}
    if(held_buttons == xbox::BUTTON_DOWN){out.leg_height_direction = 1;}

    route_action_request(mode, held_buttons, pressed_buttons, max_linear_vel, out);
}

/**
 * @brief 初始化输入路由运行状态
 */
void controller::input_router::init()
{
    last_source = controller::input_source::NONE;
    last_buttons = 0;
    last_fresh = false;
}
