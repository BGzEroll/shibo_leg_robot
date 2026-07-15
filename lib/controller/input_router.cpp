#include "input_router.h"

#include "actions.h"
#include "esp_timer.h"
#include "host_comm.h"
#include "web_server.h"
#include "xbox.h"
#include "xbox_dev.h"

/* ---- 输入路由运行状态 ---- */

static constexpr uint32_t INPUT_TIMEOUT_US = 250000;
static constexpr uint8_t INPUT_BUTTON_COUNT = 16;
static constexpr uint8_t AXIS_YAW = 0;
static constexpr uint8_t AXIS_LINEAR = 3;

struct sampled_input
{
    controller::input_source source = controller::input_source::NONE;
    uint32_t timestamp_us = 0;
    uint16_t buttons = 0;
    uint16_t pressed_buttons = 0;
    float axes[6]{};
    bool fresh = false;
};

struct source_tracker
{
    uint32_t stream_id = 0;
    uint32_t sequence = 0;
    uint16_t press_count[INPUT_BUTTON_COUNT]{};
    bool initialized = false;
};

static controller::input_source last_source = controller::input_source::NONE;
static bool last_fresh = false;
static source_tracker xbox_tracker;
static source_tracker web_tracker;
static source_tracker host_tracker;

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
 * @brief 更新指定输入源的按钮累计计数跟踪状态
 *
 * @param stream_id 输入流代次
 * @param sequence 输入快照序号
 * @param valid 输入快照是否有效
 * @param press_count 按钮累计按下计数
 * @param tracker 输入源消费跟踪状态
 * @param ordered 快照顺序是否有效输出
 *
 * @return 本次检测到的按下按钮位
 */
static uint16_t update_press_tracker(uint32_t stream_id, uint32_t sequence,
    bool valid, const uint16_t *press_count, source_tracker &tracker, bool &ordered)
{
    bool stream_changed = !tracker.initialized || tracker.stream_id != stream_id;
    ordered = stream_changed || !valid ||
        (int32_t)(sequence - tracker.sequence) >= 0;
    if(!ordered){return 0;}

    if(stream_changed)
    {
        tracker = source_tracker{};
        tracker.stream_id = stream_id;
        tracker.initialized = true;
    }

    uint16_t pressed_buttons = 0;
    for(uint8_t i = 0; i < INPUT_BUTTON_COUNT; i++)
    {
        if(press_count[i] != tracker.press_count[i])
        {
            pressed_buttons |= (uint16_t)(1U << i);
            tracker.press_count[i] = press_count[i];
        }
    }
    tracker.sequence = sequence;
    return pressed_buttons;
}

/**
 * @brief 将输入端口快照转换为路由采样结果
 *
 * @param source 输入源
 * @param available 是否读到快照
 * @param stream_id 输入流代次
 * @param sequence 输入快照序号
 * @param timestamp_us 输入时间戳
 * @param buttons 当前按住按钮
 * @param press_count 按钮累计按下计数
 * @param axes 六轴输入
 * @param valid 输入快照是否有效
 * @param tracker 输入源消费跟踪状态
 * @param now_us 当前时间戳
 *
 * @return 路由采样结果
 */
static sampled_input sample_source(controller::input_source source, bool available,
    uint32_t stream_id, uint32_t sequence, uint32_t timestamp_us,
    uint16_t buttons, const uint16_t *press_count, const float *axes,
    bool valid, source_tracker &tracker, uint32_t now_us)
{
    sampled_input sample;
    sample.source = source;
    if(!available){return sample;}

    bool ordered = false;
    sample.pressed_buttons = update_press_tracker(
        stream_id, sequence, valid, press_count, tracker, ordered);
    if(!ordered){return sample;}
    sample.timestamp_us = timestamp_us;
    sample.fresh = valid && input_fresh(timestamp_us, now_us);
    if(!sample.fresh)
    {
        sample.pressed_buttons = 0;
        return sample;
    }

    sample.buttons = buttons;
    memcpy(sample.axes, axes, sizeof(sample.axes));
    return sample;
}

/**
 * @brief 读取三路输入并选择当前最高优先级快照
 *
 * @return 当前优先输入源的采样结果
 */
static sampled_input read_snapshot()
{
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    xbox_dev::input xbox_data;
    web_server::input web_data;
    host_comm::input host_data;

    bool xbox_available = xbox_dev::peek_input(xbox_data);
    bool web_available = web_server::peek_input(web_data);
    bool host_available = host_comm::peek_input(host_data);

    sampled_input xbox_sample = sample_source(
        controller::input_source::XBOX,
        xbox_available,
        xbox_data.stream_id,
        xbox_data.sequence,
        xbox_data.timestamp_us,
        xbox_data.buttons,
        xbox_data.press_count,
        xbox_data.axes,
        xbox_data.valid,
        xbox_tracker,
        now_us);
    sampled_input web_sample = sample_source(
        controller::input_source::WEB,
        web_available,
        web_data.stream_id,
        web_data.sequence,
        web_data.timestamp_us,
        web_data.held_buttons,
        web_data.press_count,
        web_data.axes,
        web_data.valid,
        web_tracker,
        now_us);
    sampled_input host_sample = sample_source(
        controller::input_source::HOST,
        host_available,
        host_data.stream_id,
        host_data.sequence,
        host_data.timestamp_us,
        host_data.buttons,
        host_data.press_count,
        host_data.axes,
        host_data.valid,
        host_tracker,
        now_us);

    if(xbox_dev::connected()){return xbox_sample;}
    if(web_sample.fresh){return web_sample;}
    return host_sample;
}

/**
 * @brief 在输入源切换或恢复新鲜度时抑制历史按键边沿
 *
 * @param sample 当前选中的输入采样
 */
static void suppress_replayed_edges(sampled_input &sample)
{
    if(sample.source != last_source || !last_fresh || !sample.fresh)
    {
        sample.pressed_buttons = 0;
    }
    last_source = sample.source;
    last_fresh = sample.fresh;
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
    sampled_input snapshot = read_snapshot();
    suppress_replayed_edges(snapshot);

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

    uint16_t pressed_buttons = snapshot.pressed_buttons;

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
    last_fresh = false;
    xbox_tracker = source_tracker{};
    web_tracker = source_tracker{};
    host_tracker = source_tracker{};
}
